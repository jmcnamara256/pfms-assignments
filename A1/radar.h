#ifndef RADAR_H
#define RADAR_H

#include <vector>
#include <random>
#include <chrono>
#include <string>
#include <sstream>

class Radar {
    public:
        Radar();
        std::vector<std::pair<double,double>> getData();
        std::string paramsToString();
        bool setRange(double max_range);
        bool setScanTime(int scan_time);
        unsigned int samples();
        double time_since_config();
        double time_since_query();
        double getScanTime();       

    private:
        // PRIVATE MEMBER FUNCTIONTS
        void generateTargets();
        void incrementTargets();
        // VARIABLE COUPLED PARAMETERS
        // Format MAXDIST,SCANTIME
        std::pair<double, int> current_params_ {80.0, 100};
        std::pair<double, int> alt_params_ {20.0, 200};
        // FIXED PARAMETERS
        const std::string MODEL = "RAD-XL";
        const std::string OUTPUT = "Targets(Range and Bearing Data)";
        const unsigned int MAX_NUM_TARGETS = 20;
        const double FOV = 20.0;
        const double MIN_RANGE = 0.5;
        // OTHER VARIABLES
        unsigned int samples_ = 0;
        std::chrono::steady_clock::time_point last_get_data_;
        std::chrono::steady_clock::time_point first_get_data_;
        std::chrono::steady_clock::time_point config_;
        std::vector<std::pair<double, double>> targets_;
        unsigned int num_targets_;
};

#endif