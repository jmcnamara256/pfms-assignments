#ifndef SONAR_H
#define SONAR_H

#include "ranger.h"
#include <algorithm>

// default parameters
const unsigned int SONAR_FOV = 20;
const double SONAR_MAX_RANGE = 16.0;
const double SONAR_MIN_RANGE = 0.2;
const unsigned int SONAR_RES = 20;
const unsigned int SONAR_OFFSET = 0;
const std::string SONAR_MODEL = "SN-001";
const SensingMethod SONAR_SENSOR_TYPE = CONE;

class Sonar: public Ranger
{
public:
    //Default constructor should set all sensor attributes to a default value
    Sonar();

    bool setAngularResolution(unsigned int res);
    bool setFieldOfView(unsigned int fov);
private:
   

    const std::vector<unsigned int> ALLOWED_RES = {20};
    const std::vector<unsigned int> ALLOWED_FOV = {20};
};

#endif // SONAR_H
