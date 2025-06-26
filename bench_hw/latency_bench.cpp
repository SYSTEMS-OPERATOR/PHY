#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using clock = std::chrono::steady_clock;
    auto start = clock::now();
    double rms = 0.0;
    const int cycles = 1000;
    for (int i=0;i<cycles;i++) {
        auto c1 = clock::now();
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        auto c2 = clock::now();
        auto dt = std::chrono::duration_cast<std::chrono::microseconds>(c2-c1).count();
        double err = dt - 1000.0;
        rms += err * err;
    }
    rms = std::sqrt(rms / cycles);
    std::cout << "jitter_rms_us=" << rms << std::endl;
    return 0;
}
