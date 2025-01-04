#include "wlw.hpp"

int main(int argc, char **argv) {
    LegModel legmodel(true);
    double theta = M_PI * 130.0 / 180.0;
    double beta =  M_PI * 50.0 / 180.0;

    /* Forward kinematics */
    std::cout << "****************************************\n";
    std::cout << "****** Forward kinematics example ******\n";
    std::cout << "****************************************\n";
    auto start = std::chrono::high_resolution_clock::now();
    for (int i=0; i<1000000; i++){
        legmodel.forward(theta, beta);
    }
    auto end = std::chrono::high_resolution_clock::now();
    // 計算執行時間，並轉換成毫秒
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "time: " << duration.count() << " ms" << std::endl;
    std::cout << "Output G with single value input: (" << legmodel.G[0] << ", " << legmodel.G[1] << ")\n";

    // Note: The contact_map function and other advanced features are not fully implemented in this example.
    /* Contact map */
    legmodel.contact_map(theta, beta);
    std::cout << "Output rim with single value input: " << legmodel.rim << std::endl;
    std::cout << "Output alpha with single value input: " << legmodel.alpha << std::endl;
}
