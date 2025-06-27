#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

int main() {
    const int iterations = 1000;
    std::vector<double> samples;
    samples.reserve(iterations);
    auto next = std::chrono::steady_clock::now();
    for (int i = 0; i < iterations; ++i) {
        next += std::chrono::microseconds(1000);
        auto before = std::chrono::steady_clock::now();
        std::this_thread::sleep_until(next);
        auto after = std::chrono::steady_clock::now();
        samples.push_back(std::chrono::duration<double, std::micro>(after - next).count());
    }
    double sum = 0;
    for (double v : samples) sum += v * v;
    double rms = std::sqrt(sum / samples.size());
    std::cout << "rms_us=" << rms << std::endl;
    return 0;
}
