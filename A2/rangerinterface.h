#ifndef RANGERINTERFACE_H
#define RANGERINTERFACE_H

#include <vector>

typedef enum {
    CONE,
    POINT
} SensingMethod; /*!< Available data fusion methods - as per requirement C1*/

// The RangerInterface is a class which specifies the minimum
// required interface for your Ranger class

/**
 * @brief Specifies the minimum functionality for the Ranger Class.
 * 
 */
class RangerInterface
{
public:
    RangerInterface(){};

    //Generates raw data for sensor
    virtual std::vector<double> generateData() = 0;

    //Essential getters for obtaining internal private variables
    virtual unsigned int getAngularResolution(void) = 0;
    virtual int getOffset(void) = 0;
    virtual unsigned int getFieldOfView(void) = 0;
    virtual double getMaxRange(void) = 0;
    virtual double getMinRange(void) = 0;
    virtual SensingMethod getSensingMethod(void) = 0;

    //Essential setters for setting internal private variables
    virtual bool setAngularResolution(unsigned int) = 0;
    virtual bool setOffset(int) = 0;
    virtual bool setFieldOfView(unsigned int) = 0;

};

#endif // RANGERFUSIONINTERFACE_H
