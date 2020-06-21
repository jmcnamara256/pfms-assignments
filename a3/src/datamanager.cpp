#include "datamanager.h"

DataManager::DataManager(Simulator* sim_) : sim_ptr(sim_)
{
    // Begin data update threads
    onboard_sensor_thread   = std::thread(&DataManager::updateOnboardSensor, this);
    base_sensor_thread      = std::thread(&DataManager::updateBaseSensor, this);
}

DataManager::~DataManager()
{
    // Join the threads when finished
    onboard_sensor_thread.join();
    base_sensor_thread.join();
}

// ####################################################
// ################ Update Methods ####################
// ####################################################

// Get data from the simulator for the onboard radar
void DataManager::updateOnboardSensor(void)
{
    while(true)
    {
        // Container of global bogie positions with timestamps
        std::vector<GlobalOrdStamped> raw_global_data;

        // Get the raw 'rangebearing' data from the simulator.
        // This is blocking 10ms
        // Assign the bogie data to a temporary variable 'rawdata' so other things can use the actual data during this time
        std::vector<RangeBearingStamped> raw_data = sim_ptr->rangeBearingToBogiesFromFriendly();
    
        // Get the current friendly pose, in order to convert the readings to global coords
        Pose friendly = sim_ptr->getFriendlyPose();


        // For each bogie
        for(auto blip : raw_data)
        {
            // Convert to global coord and add to our container
            raw_global_data.push_back({radarToGlobal(blip.range, blip.bearing, friendly), blip.timestamp});
        }
        std::cout << std::endl;

        // Lock the data container
        std::lock_guard<std::mutex> lock(radar_data.mtx);
        
        // Add the new raw data to the front of the deque
        radar_data.data.push_front(raw_global_data);

        // Delete old data
        if(radar_data.data.size() > MAX_POSITION_KEPT)
        {
            radar_data.data.resize(MAX_POSITION_KEPT);
        }
        
    }
}

void DataManager::updateBaseSensor(void)
{
    while(true)
    {
        // Get the raw 'rangevelocity' data from the simulator.
        // This is blocking 100ms
        // Assign the bogie data to a temporary variable 'rawdata' so other things can use the actual data during this time
        std::vector<RangeVelocityStamped> raw_data = sim_ptr->rangeVelocityToBogiesFromBase();

        // Container of global bogie positions with timestamps
        std::vector<VelocityStamped> velocity_data;

        // For each bogie
        for(auto blip : raw_data)
        {
            // Extract data fields and add to vector
            velocity_data.push_back({blip.velocity, blip.timestamp});
        }

        // Lock the data container
        std::lock_guard<std::mutex> lock(base_data.mtx);
        
        // Add the new raw data to the front of the deque
        base_data.data.push_front(velocity_data);

        // Delete old data
        if(base_data.data.size() > MAX_VELOCITY_KEPT)
        {
            base_data.data.resize(MAX_VELOCITY_KEPT);
        }
    }
}

/**
 * @brief Transform data from radar format (range/bearing) to a global coordinate. The transformation is done relative to the given pose.
 * 
 * @param range 
 * @param bearing 
 * @param pose 
 * @return GlobalOrd 
 */
GlobalOrd DataManager::radarToGlobal(double range, double bearing, Pose pose)
{
    // Global angle of bogie from friendly. wrapped to -pi,pi
    double alpha = pose.orientation + bearing;

    double x = pose.position.x + range * cos(pose.orientation + bearing);
    double y = pose.position.y + range * sin(pose.orientation + bearing);

    std::cout << "X: " << x << "    Y: " << y << std::endl;

    return {x, y};
}


/**
 * @brief Wraps given angle to a domain of -Pi,Pi.
 * 
 * @param angle 
 * @return double 
 */
double DataManager::constrainToPi(double angle)
{
    angle = fmod((angle + M_PI), 2 * M_PI);

    // Account for a negative input angle
    if(angle < 0)
    {
        angle += (2 * M_PI);
    }

    // Output wrapped between -PI,PI
    return angle - M_PI;
}


std::vector<GlobalOrdStamped> DataManager::getNewestPosition()
{
    std::lock_guard<std::mutex> lock(radar_data.mtx);
    return radar_data.data.front();

}
std::vector<VelocityStamped> DataManager::getNewestVelocity()
{
    std::lock_guard<std::mutex> lock(base_data.mtx);
    return base_data.data.front();
}

std::deque<std::vector<GlobalOrdStamped>> DataManager::getAllPosition() 
{
    std::lock_guard<std::mutex> lock(radar_data.mtx);
    return radar_data.data;
}
std::deque<std::vector<VelocityStamped>> DataManager::getAllVelocity()
{
    std::lock_guard<std::mutex> lock(base_data.mtx);
    return base_data.data;
}

unsigned int DataManager::getPosSampleNumber()
{
    std::lock_guard<std::mutex> lock(radar_data.mtx);
    return radar_data.data.size();
}

unsigned int DataManager::getVelSampleNumber()
{
    std::lock_guard<std::mutex> lock(base_data.mtx);
    return base_data.data.size();
}





