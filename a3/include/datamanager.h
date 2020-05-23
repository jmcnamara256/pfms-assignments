#ifndef DATAMANAGER_H
#define DATAMANAGER_H

#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>
#include "types.h"
#include "simulator.h"


// ####################################################
// ################## DATA STRUCTS ####################
// ####################################################
struct DopplerData
{
    std::deque<std::vector<VelocityStamped>> data;
    std::mutex mtx;
    std::condition_variable cv;
};

struct RadarData
{
    std::deque<std::vector<GlobalOrdStamped>> data;
    std::mutex mtx;
    std::condition_variable cv;
};
// ####################################################
// ####################################################


class DataManager
{
private:
    // Data

    // Simulator object to get data from
    Simulator* sim_ptr;

    std::thread base_sensor_thread;
    std::thread onboard_sensor_thread;

    RadarData radar_data;
    DopplerData base_data;


    // Functions
    void updateOnboardSensor(void);
    void updateBaseSensor(void);

    GlobalOrd radarToGlobal(double range, double bearing, Pose pose);
    double constrainToPi(double angle);

    const unsigned int MAX_VELOCITY_KEPT = 20;
    const unsigned int MAX_POSITION_KEPT = 10;

public:
    // Constructor+Descructor
    DataManager(Simulator* sim_);
    ~DataManager();

    // Data getters
    std::vector<GlobalOrdStamped> getNewestPosition();
    std::vector<VelocityStamped>  getNewestVelocity();

    std::deque<std::vector<GlobalOrdStamped>> getAllPosition();
    std::deque<std::vector<VelocityStamped>>  getAllVelocity();

    unsigned int getPosSampleNumber();
    unsigned int getVelSampleNumber();

};





#endif