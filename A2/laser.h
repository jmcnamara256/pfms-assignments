#ifndef LASER_H
#define LASER_H

#include "ranger.h"
#include <algorithm>

// default parameters
const unsigned int LASER_FOV = 180;
const double LASER_MAX_RANGE = 8.0;
const double LASER_MIN_RANGE = 0.2;
const unsigned int LASER_RES = 30;
const unsigned int LASER_OFFSET = 0;
const std::string LASER_MODEL = "SICK-XL";
const SensingMethod LASER_SENSOR_TYPE = POINT; 

/**
 * @brief A Laser sensor type.
 * 
 */
class Laser: public Ranger
{
    public:
        //Default constructor - should set all sensor attributes to a default value
        Laser();
        bool setAngularResolution(unsigned int res);
        bool setFieldOfView(unsigned int fov);
    private:

        // The allowable values for resolution and fov.
        // If more sensor configurations were required, simply add the values to these containers
        const std::vector<unsigned int> ALLOWED_RES = {10, 30};
        const std::vector<unsigned int> ALLOWED_FOV = {180};

   

};

#endif // LASER_H
