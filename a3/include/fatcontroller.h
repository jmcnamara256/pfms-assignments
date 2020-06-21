#ifndef FATCONTROLLER_H
#define FATCONTROLLER_H

#include <thread>
#include "targetcontroller.h"
#include "purepursuit.h"
#include "simulator.h"

class FatController
{
    private:
        Simulator* sim_ptr;
        TargetController* target_control;
        PurePursuit* tracker;

        Aircraft friendly;

        std::thread sim_thread;
        std::thread control_thread;
        std::thread tracker_thread;


        // private functions
        Aircraft predictFuturePosition(Aircraft, double);
        double distance(GlobalOrd, GlobalOrd);

    public:
        FatController(Simulator* sim_);
        ~FatController();

        std::thread& begin();

        // Test data & Targeter
        void controlFriendly();
        void updateGoal();
};
#endif