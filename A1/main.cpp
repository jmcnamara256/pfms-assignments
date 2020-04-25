#include <iostream>
#include <iomanip>
#include "radar.h"

/**
 * @brief Prints the data from the radar. 
 * 
 * @param data 
 */
void printData(std::vector<std::pair<double, double>> data) {
    for(auto p : data) {
        std::cout << std::setw(10) << p.first << ", " << std::setw(10) << p.second << std::endl; 
    }
    std::cout << std::endl;
}

int main(){
    double range;
    std::cout << "Set radar range (20.0m/80.0m). Default (80.0) will be used if an invalid range is chosen: ";
    while(!(std::cin >> range)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Enter decimal value" << std::endl;
    }
    Radar radar;
    std::cout << radar.paramsToString() << std::endl;
    radar.setRange(range);
    while (radar.time_since_query() <= (5.0 - radar.getScanTime())) {
        std::cout << "Sample: " << radar.samples() << std::endl;
        printData(radar.getData());
    }
    std::cout << radar.time_since_query() << "seconds have elapsed." << std::endl;
    std::cout << "Set radar range (20.0m/80.0m). Default (80.0) will be used if an invalid range is chosen: ";
    while(!(std::cin >> range)) {
        std::cin.clear();
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        std::cout << "Enter decimal value" << std::endl;
    }
    radar.setRange(range);
    while (1) {
        std::cout << "Sample: " << radar.samples() << std::endl;
        printData(radar.getData());
    }
    return 0;
}