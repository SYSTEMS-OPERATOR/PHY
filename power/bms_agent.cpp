#include <vector>
#include <chrono>
#include <thread>
#include <iostream>

class BMSAgent {
public:
    BMSAgent(double limit)
        : temp_limit(limit), contactor_open(false), last_time(std::chrono::high_resolution_clock::now()) {}

    void ingest(double temp) {
        auto now = std::chrono::high_resolution_clock::now();
        double dt = std::chrono::duration<double>(now - last_time).count();
        last_time = now;
        temps.push_back(temp);
        if (check_trip(dt)) {
            open_contactor();
        }
    }

    bool check_trip(double dt) const {
        if (temps.empty()) return false;
        double t = temps.back();
        double dtemp = temps.size() > 1 ? t - temps[temps.size() - 2] : 0.0;
        if (dt > 0 && dtemp / dt > 4.0)
            return true;
        return t > temp_limit;
    }

    void open_contactor() const {
        std::cout << "CONTACTOR OPEN" << std::endl;
    }

private:
    double temp_limit;
    mutable bool contactor_open;
    std::chrono::high_resolution_clock::time_point last_time;
    std::vector<double> temps;
};
