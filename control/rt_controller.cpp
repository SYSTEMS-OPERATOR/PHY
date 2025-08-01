#include <atomic>
#include <chrono>
#include <thread>
#include <vector>
#include <algorithm>

class RTController {
public:
    explicit RTController(double limit)
        : torque_limit(limit), running(false) {}

    void start() {
        running = true;
        worker = std::thread(&RTController::loop, this);
    }

    void stop() {
        running = false;
        if (worker.joinable())
            worker.join();
    }

    void set_desired(const std::vector<double>& t) {
        desired = t;
    }

    std::vector<double> get_command() const {
        return command;
    }

private:
    void loop() {
        using clock = std::chrono::steady_clock;
        auto next = clock::now();
        auto last_cmd = next;
        while (running) {
            next += std::chrono::microseconds(1000);
            command.resize(desired.size());
            for (size_t i = 0; i < desired.size(); ++i) {
                command[i] = std::clamp(desired[i], -torque_limit, torque_limit);
            }
            auto now = clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_cmd).count() > 2) {
                std::fill(command.begin(), command.end(), 0.0);
            }
            last_cmd = now;
            std::this_thread::sleep_until(next);
        }
    }

    double torque_limit;
    std::atomic<bool> running;
    std::vector<double> desired;
    std::vector<double> command;
    std::thread worker;
};
