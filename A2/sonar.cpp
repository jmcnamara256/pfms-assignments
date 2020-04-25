#include "sonar.h"


/**
 * @brief Construct a new Sonar:: Sonar object
 * 
 */
Sonar::Sonar() {
    // set the default parameters
    fov_ = SONAR_FOV;
    max_range_ = SONAR_MAX_RANGE;
    min_range_ = SONAR_MIN_RANGE;
    angular_resolution_ = SONAR_RES;
    model_ = SONAR_MODEL;
    sensor_type_ = SONAR_SENSOR_TYPE;
    orientation_offset_ = SONAR_OFFSET;
}


/**
 * @brief Set the angular resolution of the sonar. This method also changes FOV, as they are equal for cone type sensors.
 * 
 * @param res The angular resolution requested.
 * @return true 
 * @return false 
 */
bool Sonar::setAngularResolution(unsigned int res) {
    // For cone sensor type, resolution and fov are equal
    // If one is set, change both

    // Check if resolution is in allowable list
    if(std::find(ALLOWED_RES.begin(), ALLOWED_RES.end(), res) != ALLOWED_RES.end()) {

        // Set it, and change fov
        angular_resolution_ = res;
        fov_ = res;
        return true;
    }else {

        // Use default otherwise
        angular_resolution_ = SONAR_RES;
        fov_ = SONAR_FOV;
        return false;
    }
    
}

/**
 * @brief Set the Field of View of the sonar. The angular resolution is also changed.
 * Checks input against allowable values for this sensor type. Returns false if value is not allowed, returns false.
 * 
 * @param fov Field of View requested.
 * @return true 
 * @return false 
 */
bool Sonar::setFieldOfView(unsigned int fov) {
    // For cone sensor type, resolution and fov are equal
    // If one is set, change both

    // Check if fov is in allowable list
    if(std::find(ALLOWED_FOV.begin(), ALLOWED_FOV.end(), fov) != ALLOWED_FOV.end()) {

        // Set it, and change resolution
        angular_resolution_ = fov;
        fov_ = fov;
        return true;
    }else {

        // Use default otherwise
        angular_resolution_ = SONAR_RES;
        fov_ = SONAR_FOV;
        return false;
    }
}