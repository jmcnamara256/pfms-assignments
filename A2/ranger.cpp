#include "ranger.h"

/**
 * @brief Construct a new Ranger:: Ranger object
 * 
 */
Ranger::Ranger(){
    // create a normal distribution with mean 4.0 and stddev 5.0
    range_distribution = new std::normal_distribution<double>(DATA_MEAN, DATA_STDEV);

    // generate unique seed
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);
}


/**
 * @brief Destroy the Ranger:: Ranger object
 * 
 */
Ranger::~Ranger() {
    delete generator;
    delete range_distribution;
}


/**
 * @brief Returns the angular resolution of the sensor in degrees. For cone type sensors, this is equal to the Field of View.
 * For point type sensors, this is the angular seperation between readings.
 * 
 * @return unsigned int 
 */
unsigned int Ranger::getAngularResolution(void) {
    return angular_resolution_;
}

/**
 * @brief Returns the angular offset of this sensor from 90 degrees (vertical). Counterclockwise positive.
 * 
 * @return int 
 */
int Ranger::getOffset(void) {
    return orientation_offset_;
}


/**
 * @brief Returns the Field of View of this sensor in degrees.
 * 
 * @return unsigned int 
 */
unsigned int Ranger::getFieldOfView(void) {
    return fov_;
}

/**
 * @brief Maximum sensing range. Data above this is cropped to max range.
 * 
 * @return double 
 */
double Ranger::getMaxRange(void) {
    return max_range_;
}


/**
 * @brief Minimum sensing range. Data below this is cropped to min range.
 * 
 * @return double 
 */
double Ranger::getMinRange(void) {
    return min_range_;
}


/**
 * @brief Returns the number of times the sensor has generated data.
 * 
 * @return double 
 */
double Ranger::getSampleNumber(void) {
    return sample_number_;
}

/**
 * @brief Returns the sensing type of the sensor. Cone or Point.
 * 
 * @return SensingMethod 
 */
SensingMethod Ranger::getSensingMethod(void) {
    return sensor_type_;
}


/**
 * @brief Sets the angular offset of the sensor from 90 degrees (vertical). Counterclockwise positive.
 * 
 * @param offset 
 * @return true 
 * @return false 
 */
bool Ranger::setOffset(int offset) {
    orientation_offset_ = constrainAngle(offset);
    return true;
}

/**
 * @brief Sets the sensor Field of View.
 * 
 * @param fov 
 * @return true 
 * @return false 
 */
bool Ranger::setFieldOfView(unsigned int fov) {
    fov_ = fov;
}


/**
 * @brief Returns the fixed parameters of the sensor concatinated into a single string.
 * Fixed parameters can be retrieved individually via their respective get methods.
 * 
 * @return std::string 
 */
std::string Ranger::getFixedParameters() {
    std::ostringstream str;

    std::string sensing_type_str;
    if(sensor_type_ == CONE) {
        sensing_type_str = "CONE";
    }else if(sensor_type_ == POINT) {
        sensing_type_str = "POINT";
    }

    str << "\nModel: " << model_ 
        << "\nMin Range: " << min_range_ << "m"
        << "\nMax Range: " << max_range_ << "m"
        << "\nSensor Type: " << sensing_type_str << std::endl;
    return str.str();
}


/**
 * @brief Generate sensor data. Determines sensor type and generates data accordingly, return is a vector of ranges (doubles).
 * The cone type sensor (sonar), will only return a single value.
 * 
 * @return std::vector<double> 
 */
std::vector<double> Ranger::generateData() {
    // data vector
    std::vector<double> data;

    // Increment sample number
    sample_number_++;

    // for the sonar
    if(sensor_type_ == CONE) {
        // only 1 data point is needed
        data.push_back(constrainRange((*range_distribution)(*generator)));
        return data;
    }

    // for the laser
    else {
        // generate a data point for each angle in the fov
        // first angle is 0 deg
        for(int a = -fov_/2; a <= fov_/2; a += angular_resolution_) {
            data.push_back(constrainRange((*range_distribution)(*generator)));
        }
        return data;
    }
}


/**
 * @brief Constrain a value to be between the maximum and minimum range of this sensor.
 * If value < minimum, then value = minimum.
 * If value > maximum, then value = maximum.
 * 
 * @param v 
 * @return double 
 */
double Ranger::constrainRange(double v) {
    // if the data is out of range, crop to nearest edge
    return (v < min_range_) ? min_range_ : (max_range_ < v) ? max_range_ : v;
}


/**
 * @brief Returns the angle wrapped to a domain of (-180 -> 180) degrees.
 * 
 * @param v Angle to be wrapped.
 * @return double 
 */
double Ranger::constrainAngle(double v) {
    // Constrain an angle between -180 and 180
    v = fmod(v + 180,360);
    if(v < 0)
        v += 360;
    return v - 180;
}