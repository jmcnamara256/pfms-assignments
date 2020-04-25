#include "radar.h"
#include <iostream>

Radar::Radar() {
    last_get_data_ = config_ = std::chrono::steady_clock::now();
    generateTargets();
}

/**
 * @brief Generates a random number of targets less than MAX_NUM_TARGETS. For each target, a random range and heading is created. 
 * The range and heading limits are determined by the current configurable parameters, scan_time/max_range.
 * 
 */
void Radar::generateTargets() {
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> distribution_num_targets(0,static_cast<double>(MAX_NUM_TARGETS));
    std::uniform_real_distribution<> distribution_range(MIN_RANGE, current_params_.first);
    std::uniform_real_distribution<> distribution_heading(-FOV/2, FOV/2);
    num_targets_ = distribution_num_targets(generator);
    for (int i = 0; i < num_targets_; i++){
        targets_.push_back(std::pair<double,double> (distribution_range(generator), distribution_heading(generator)));
    }
}

/**
 * @brief Returns the current range/heading of each target. <Range, Heading>
 * 
 * @return std::vector<std::pair<double,double>> 
 */
std::vector<std::pair<double,double>> Radar::getData() {
    incrementTargets();
    while ((std::chrono::steady_clock::now() - last_get_data_) < std::chrono::milliseconds(current_params_.second)){}
    last_get_data_ = std::chrono::steady_clock::now();
    return targets_;
}

/**
 * @brief Increments the targets to their new values. Target headings are not changed, only range. This method is called from getdata().
 * 
 */
void Radar::incrementTargets() {
    if (samples_ > 0) {
        for (auto &t : targets_) {
            t.first = ((t.first * 0.95) < 0.5) ? 0.5 : t.first * 0.95;
        } 
    } else {
        first_get_data_ = std::chrono::steady_clock::now();
    }
    samples_++;
}

/**
 * @brief Set the max range of the Radar. This will also change coupled variables, e.g. scan_time
 * 
 * @param max_range 
 * @return true 
 * @return false 
 */
bool Radar::setRange (double max_range) {
    if (max_range == current_params_.first) {
        return true;
    } else if (max_range == alt_params_.first) {
        current_params_.swap(alt_params_);
        config_ = std::chrono::steady_clock::now();
        return true;
    } else {
        return false;
    }
}
/**
 * @brief Sets the scan_time of the radar. Also changes the coupled variable max_range.
 * 
 * @param scan_time 
 * @return true 
 * @return false 
 */
bool Radar::setScanTime (int scan_time) {
    if (scan_time == current_params_.second) {
        return true;
    } else if (scan_time == alt_params_.second) {
        current_params_.swap(alt_params_);
        config_ = std::chrono::steady_clock::now();
        return true;
    } else {
        return false;
    }
}

/**
 * @brief Returns all of the fixed and variable parameters of the radar.
 * 
 * @return std::string 
 */
std::string Radar::paramsToString() {
    std::ostringstream str;
    str << "\nModel: " << MODEL 
        << "\nOutput: " << OUTPUT
        << "\nFOV: " << FOV << "deg"
        << "\nMin Range: " << MIN_RANGE << "m"
        << "\nMax Range: " << current_params_.first << "m"
        << "\nMax # Targets: " << MAX_NUM_TARGETS << std::endl;
    return str.str();
}

/**
 * @brief Returns the current sample number.
 * 
 * @return unsigned int 
 */
unsigned int Radar::samples() {
    return samples_;
}
/**
 * @brief Returns the time since the variable parameters were last changed. In seconds.
 * 
 * @return double 
 */
double Radar::time_since_config() {
    std::chrono::duration<double> t = std::chrono::steady_clock::now() - config_;
    return t.count();
}
/**
 * @brief Returns the time since the first getData() query was made. In seconds.
 * 
 * @return double 
 */
double Radar::time_since_query() {
    if (samples_ > 0) {
        std::chrono::duration<double> t = std::chrono::steady_clock::now() - first_get_data_;
        return t.count();
    }
    else {
        return 0;
    }
    
}

double Radar::getScanTime() {
    return static_cast<double>(current_params_.second)/1000;
}