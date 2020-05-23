#include "targetcontroller.h"
#include <thread>
#include <iostream>

TargetController::TargetController(Simulator* sim_) : sim_ptr(sim_)
{
    // Start the data handler
    data_manager = new DataManager(sim_ptr);
}

TargetController::~TargetController()
{
    delete data_manager;
}

// Intepolate the y-value given x and 2 points
double TargetController::interpolate(double x1, double y1, double x2, double y2, double xp)
{
    return (y2 - y1) / (x2 - x1) * (xp - x1) + y1;
}

std::vector<Aircraft> TargetController::estimate_aircraft()
{
    // we need 3 samples to do the calculation
    // Busy wait untill we have 3, this should only take 30ms
    // This should also only occur once upon startup of the sim
    while(data_manager->getPosSampleNumber() < 3){};

    std::vector<GlobalOrdStamped> current_pos, previous_pos, old_pos;
    
    auto pos_list = data_manager->getAllPosition();

    current_pos     = pos_list.front();
    previous_pos    = pos_list.at(1);
    old_pos         = pos_list.at(2);

    std::vector<Aircraft> bogies;

    // for each current bogie pos
    for(auto current : current_pos)
    {
        for(auto old : old_pos)
        {
            for(auto previous : previous_pos)
            {
                if(equal( previous.position , midPoint(current.position, old.position) ) &&
                    fabs(heading(current.position, previous.position) - heading(previous.position, old.position)) < M_PI / 6)
                {
                    double dt = double(current.timestamp - old.timestamp);
                    double dx = distance(current.position, old.position);
                    
                    Aircraft bogie;
                    bogie.pose = {current.position, heading(previous.position, current.position)};
                    bogie.linear_velocity = dx/dt / 1000.0;
                    bogie.previousGoalPose = {previous.position, heading(previous.position, current.position)};
                    bogies.push_back(bogie);

                    // std::cout << "X: " << current.position.x << "    Y: " << current.position.y << "    TS: " << current.timestamp << std::endl;
                    // std::cout << "X: " << previous.position.x << "    Y: " << previous.position.y << "    TS: " << previous.timestamp << std::endl;
                    // std::cout << "X: " << old.position.x << "    Y: " << old.position.y << "    TS: " << old.timestamp << std::endl;
                    //std::cout << "dt: " << dt  << "    DX: " << dx << "\n"<< std::endl;

                    goto next_bogie;    // to goto or not to goto
                }
            }
        }
        next_bogie:;
    } 

    std::cout << std::endl;
    return bogies;     
}


double TargetController::distance(GlobalOrd p1, GlobalOrd p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

/**
 * @brief Returns the global heading from p1 to p2
 * 
 * @param p1 
 * @param p2 
 * @return double 
 */
double TargetController::heading(GlobalOrd p1, GlobalOrd p2)
{
    return atan2(( p2.y - p1.y ) , ( p2.x - p1.x ));
}

GlobalOrd TargetController::midPoint(GlobalOrd p1, GlobalOrd p2)
{
    return { (p1.x + p2.x) / 2          ,  (p1.y + p2.y) / 2 };
}

bool TargetController::equal(GlobalOrd p1, GlobalOrd p2)
{
    return fabs(p2.x - p1.x) < COMPARISON_TOLERANCE && fabs(p2.y - p1.y) < COMPARISON_TOLERANCE;
}