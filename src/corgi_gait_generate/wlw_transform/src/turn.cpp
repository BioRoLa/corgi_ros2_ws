#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <complex>
#include <cmath>


static inline double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}
static inline double rad2deg(double rad) {
    return rad * 180.0 / M_PI;
}

Eigen::Vector2d phi2tb(const Eigen::Vector2d &phi)
{
    Eigen::Vector2d tb;

    // r_cpx = exp(i*(phi0 + 17 deg))
    // l_cpx = exp(i*(phi1 - 17 deg))
    std::complex<double> r_cpx(0.0, phi[0] + deg2rad(17.0));
    std::complex<double> l_cpx(0.0, phi[1] - deg2rad(17.0));

    // drl = exp(i*((phi0+17deg)-(phi1-17deg)))
    std::complex<double> drl = std::exp(r_cpx) / std::exp(l_cpx);

    double theta = std::arg(drl);
    if (theta < 0)
        theta += 2.0 * M_PI;
    theta *= 0.5;

    // beta = arg( exp(i*(phi1-17deg)) ) + theta
    double beta = std::arg(std::exp(l_cpx)) + theta;

    tb << theta, beta;
    return tb;
}

int main()
{
    std::ifstream fin("turning_test_0216.csv");
    if (!fin.is_open()) {
        std::cerr << "無法開啟檔案: traj.csv" << std::endl;
        return 1;
    }

    // 輸出檔名可以自訂
    std::ofstream fout("0216_turn.csv");
    if (!fout.is_open()) {
        std::cerr << "無法開啟輸出檔案:re_traj.csv" << std::endl;
        return 1;
    }

    std::string line;
    // 為了讓「所有行、所有組」的 beta 單調遞減，用一個變數記錄上一個 beta
    // 設定為非常大的正值，確保第一筆 (或後續) 容易調整
    std::vector<double> lastBetaRad(4) ;
    lastBetaRad= {10,10,10,10};

    while (std::getline(fin, line))
    {
        if (line.empty()) {
            continue; // 跳過空行
        }

        // 用逗號分割
        std::stringstream ss(line);
        std::vector<double> vals;
        double temp;
        while (ss >> temp) {
            vals.push_back(temp);
            if (ss.peek() == ',') {
                ss.ignore();
            }
        }

        // 預期一行有 12 個欄位：前 8 個用來計算 phi，後 4 個保留
        if (vals.size() < 12) {
            // 數量不足，視情況處理
            continue;
        }

        // 前 8 個：4 組 (phi0, phi1)
        // 後 4 個保留
        std::vector<double> thetaDeg(4), betaDeg(4);

        for (int i = 0; i < 4; ++i) {
            Eigen::Vector2d phi(vals[2*i], vals[2*i + 1]);            
            // 取得 (theta, beta) in rad
            Eigen::Vector2d tb = phi2tb(phi);

            // 題主需求： beta 取負號
            tb[1] = -tb[1];

            while (tb[1] > lastBetaRad[i]) {
                tb[1] -= 2.0 * M_PI;
            }
            // 更新 lastBetaRad
            lastBetaRad [i]= tb[1];

            // 轉成度數
            thetaDeg[i] = (tb[0]);
            betaDeg[i]  = (tb[1]);
        }

        // 輸出到 re_traj.csv
        // (theta,beta)共 8 個欄位, 再加原行後 4 個
        for (int i = 0; i < 4; ++i) {
            if (i == 0 || i == 3){
                fout << thetaDeg[i] << "," << -betaDeg[i] << ",";
            }
            else{
            fout << thetaDeg[i] << "," << betaDeg[i] << ",";
            }
        }
        fout << vals[8] << "," << vals[9] << "," 
             << vals[10] << "," << vals[11] << "\n";
    }

    fin.close();
    fout.close();

    std::cout << "see re_traj.csv" << std::endl;
    return 0;
}
