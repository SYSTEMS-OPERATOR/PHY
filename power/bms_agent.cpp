#include <vector>
#include <chrono>
#include <thread>
#include <iostream>

class BMSAgent {
public:
    BMSAgent(double limit) : temp_limit(limit), contactor_open(false) {}

    void ingest(double temp) {
        temps.push_back(temp);
        if (check_trip()) {
            open_contactor();
        }
    }

    bool check_trip() const {
        if (temps.empty()) return false;
        double t = temps.back();
        return t > temp_limit;
    }

    void open_contactor() const {
        std::cout << "CONTACTOR OPEN" << std::endl;
    }

private:
    double temp_limit;
    mutable bool contactor_open;
    std::vector<double> temps;
};
