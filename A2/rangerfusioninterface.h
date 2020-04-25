#ifndef RANGERFUSIONINTERFACE_H
#define RANGERFUSIONINTERFACE_H

#include <vector>
#include "rangerinterface.h"
#include "cell.h"

// The RangerFusionInterface is a class which specifies the minimum
// required interface for your RangerFusion class your ranger fusion
// class must inherit from it

/**
 * @brief Specifies the minimum functionality for the RangerFusion Class.
 * 
 */
class RangerFusionInterface
{
public:
    RangerFusionInterface(){}

    // Accepts container of rangers - as per requirement C1 
    virtual void setRangers(std::vector<RangerInterface*> rangers) = 0;

    // Accepts container of cells - as per requirement C2 
    virtual void setCells(std::vector<Cell*> cells) = 0;

    // Grab data and fuse - as per requirement C3
    virtual void grabAndFuseData() = 0;

    // Returns a container of raw data range readings - as per requirement C5 
    virtual std::vector<std::vector<double>> getRawRangeData() = 0;

};

#endif // RANGERFUSIONINTERFACE_H
