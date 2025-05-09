#include <iostream>
#include <fstream>
#include <vector>
#include <array>
#include <cmath>
#include <string>
#include <cmath>
#include <complex>
#include <stdexcept>
#include <Eigen/Dense>
#include "leg_model.hpp"

/// Free function: 根據當前姿態 (theta,beta)、弧段編號 rim 和參數化角 alpha，
/// 計算輪緣上該點在世界座標系下的 (x,y)。
/// rim: 1=left upper, 2=left lower, 3=G, 4=right lower, 5=right upper
inline Eigen::Vector2d pointOnRimByAlpha(
  LegModel &legmodel,
  double theta,
  double beta,
  int rim,
  double alpha
) {
  using namespace std::complex_literals;

  // 1) 更新機構到當前姿態（含旋轉）
  legmodel.forward(theta, beta, /*vector=*/false);

  // 2) 取出對應弧段的圓心 O 與弧起點向量 V0 = P0 - O
  std::complex<double> O, V0;
  switch (rim) {
    case 1: O  = legmodel.U_l_c; V0 = legmodel.H_l_c - legmodel.U_l_c; break;
    case 2: O  = legmodel.L_l_c; V0 = legmodel.F_l_c - legmodel.L_l_c; break;
    case 3: O  = legmodel.L_l_c; V0 = legmodel.G_c   - legmodel.L_l_c; break;
    case 4: O  = legmodel.L_r_c; V0 = legmodel.F_r_c - legmodel.L_r_c; break;
    case 5: O  = legmodel.U_r_c; V0 = legmodel.H_r_c - legmodel.U_r_c; break;
    default: throw std::runtime_error("pointOnRimByAlpha: invalid rim id");
  }

  // 3) 根據 alpha 範圍選擇旋轉角度
  const double deg50 = 50.0 * M_PI / 180.0;
  std::complex<double> rotated;
  if      (alpha >= -M_PI    && alpha < -deg50) rotated = std::exp(1i*(alpha + M_PI)) * V0;
  else if (alpha >= -deg50   && alpha <  0.0)  rotated = std::exp(1i*(alpha + deg50)) * V0;
  else if (alpha >=  0.0     && alpha <  deg50)  rotated = std::exp(1i*(alpha))        * V0;
  else if (alpha >=  deg50   && alpha <  M_PI)   rotated = std::exp(1i*(alpha - deg50)) * V0;
  else {
      // 超出輪緣範圍，按 Eq.(2.28) 處理：腳尖
      std::complex<double> VG = legmodel.G_c - legmodel.L_l_c;
      std::complex<double> tip = legmodel.L_l_c + 
          (legmodel.r/legmodel.R) * (std::exp(1i*alpha) * VG);
      return { tip.real(), tip.imag() };
  }

  // 4) 一般輪緣：P = O + (r/R)·rotated
  std::complex<double> P = O + (legmodel.radius/legmodel.R) * rotated;
  return { P.real(), P.imag() };
}
        
int main() {
    LegModel legmodel(true);

    // 1) 初始姿態下，算出 rim 和 alpha
    double theta0 = M_PI * 50.0 / 180.0;
    double beta0  = M_PI * 45.0 / 180.0;
    legmodel.contact_map(theta0, beta0);
    int rim_id    = legmodel.rim;
    double alpha0 = legmodel.alpha;

    std::cout << "Initial rim:   " << rim_id    << std::endl;
    std::cout << "Initial alpha: " << alpha0    << std::endl;
    std::cout << "Initial contact_p: (" 
              << legmodel.contact_p[0] << ", " 
              << legmodel.contact_p[1] << ")\n";

    // 2) 新的姿態 (要追蹤同一個輪緣上的點)
    double theta1 = M_PI * 130.0  / 180.0;
    double beta1  = M_PI * -90.0 / 180.0;

    Eigen::Vector2d P = pointOnRimByAlpha(legmodel, theta1, beta1, rim_id, alpha0);
    
    std::cout << "Tracked point at new pose: ("
              << P.x() << ", " << P.y() << ")\n";

    return 0;
}
