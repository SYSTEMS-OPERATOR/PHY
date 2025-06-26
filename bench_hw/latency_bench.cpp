#include <chrono>
#include <iostream>
#include <thread>

int main() {
    using clock = std::chrono::steady_clock;
    const auto start = clock::now();
    auto next = start;
    double rms = 0.0;
    const int cycles = 1000;
    for (int i = 0; i < cycles; ++i) {
        next += std::chrono::microseconds(1000);
        std::this_thread::sleep_until(next);
        auto diff = std::chrono::duration<double, std::micro>(clock::now() - next).count();
        rms += diff * diff;
    }
    rms = std::sqrt(rms / cycles);
    std::cout << "jitter_us=" << rms << std::endl;
    return 0;
}
