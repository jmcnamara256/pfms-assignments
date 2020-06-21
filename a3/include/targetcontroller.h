#ifndef TARGETCONTROLLER_H
#define TARGETCONTROLLER_H

#include <vector>
#include "types.h"
#include "simulator.h"
#include "datamanager.h"

const double COMPARISON_TOLERANCE = 5;

class TargetController
{
private:
    // Ptr to simulator object. This is needed to get radar data.
    Simulator* sim_ptr;
    DataManager* data_manager;


    // Member function
    double interpolate(double, double, double, double, double);
    double distance(GlobalOrd, GlobalOrd);
    double heading(GlobalOrd, GlobalOrd);
    GlobalOrd midPoint(GlobalOrd, GlobalOrd);
    bool equal(GlobalOrd, GlobalOrd);

    
    Pose select_Target();
    
public:
    // Constructor
    TargetController(Simulator*);
    ~TargetController();

    std::vector<Aircraft> estimate_aircraft();
    void testCraft();

};
#endif