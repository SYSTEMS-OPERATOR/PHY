#include <chrono>
#include <cmath>
#include <thread>
#include <atomic>
#include <functional>
#include <vector>

// Simple 1 kHz real-time controller placeholder
class RTController {
public:
    using Callback = std::function<void(double)>;

    RTController() : running_(false), torque_limit_(1.0) {}

    void set_desired(double tau) { desired_ = tau; }

    void add_callback(const Callback& cb) { callbacks_.push_back(cb); }

    void start() {
        running_ = true;
        worker_ = std::thread([this]{ this->loop(); });
    }

    void stop() {
        running_ = false;
        if (worker_.joinable()) worker_.join();
    }

private:
    void loop() {
        auto period = std::chrono::microseconds(1000);
        while (running_) {
            auto start = std::chrono::steady_clock::now();
            double cmd = std::clamp(desired_, -torque_limit_, torque_limit_);
            for (auto& cb : callbacks_) cb(cmd);
            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
            if (elapsed.count() > 2000) {
                for (auto& cb : callbacks_) cb(0.0); // watchdog
            }
            if (elapsed < period)
                std::this_thread::sleep_for(period - elapsed);
        }
    }

    std::atomic<bool> running_;
    std::thread worker_;
    double desired_ = 0.0;
    double torque_limit_;
    std::vector<Callback> callbacks_;
};

#ifdef BUILD_MAIN
#include <iostream>
int main() {
    RTController rt;
    rt.add_callback([](double t){ std::cout << "tau=" << t << "\n"; });
    rt.set_desired(0.5);
    rt.start();
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    rt.stop();
    return 0;
}
#endif
