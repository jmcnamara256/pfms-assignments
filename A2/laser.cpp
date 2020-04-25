#include "laser.h"

Laser::Laser() {
    // Initialise defualt parameters for a Laser
    fov_ = LASER_FOV;
    max_range_ = LASER_MAX_RANGE;
    min_range_ = LASER_MIN_RANGE;
    angular_resolution_ = LASER_RES;
    model_ = LASER_MODEL;
    sensor_type_ = LASER_SENSOR_TYPE;
    orientation_offset_ = LASER_OFFSET;
}

bool Laser::setAngularResolution(unsigned int res) {
    
    // Check if the resolution is in the allowable list
    if(std::find(ALLOWED_RES.begin(), ALLOWED_RES.end(), res) != ALLOWED_RES.end()) {
        angular_resolution_ = res;
        return true;
    }else {

        // Use the default otherwise
        angular_resolution_ = LASER_RES;
        return false;
    }
}

bool Laser::setFieldOfView(unsigned int fov) {

    // Check if this fov is in the allowable list
    if(std::find(ALLOWED_FOV.begin(), ALLOWED_FOV.end(), fov) != ALLOWED_FOV.end()) {

        // Accept it
        fov_ = fov;
        return true;
    }else {

        // Use the default otherwise
        fov_ = LASER_FOV;
        return false;
    }
}