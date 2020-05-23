#include "fatcontroller.h"
#include <chrono>
#include <iostream>

FatController::FatController(Simulator* sim_) : sim_ptr(sim_)
{
    // Give friendly default starting velocity
    friendly.angular_velocity = 0;
    friendly.linear_velocity  = 50;
}


FatController::~FatController()
{
    control_thread.join();
    tracker_thread.join();

    delete target_control;
    delete tracker;
}


std::thread& FatController::begin()
{
    // Start simulator
    sim_thread = sim_ptr->spawn();

    // Create a target controller and tracker
    target_control = new TargetController(sim_ptr);
    tracker        = new PurePursuit();

    tracker->setAngularThreshold(M_PI / 6);
    tracker->setVelocityLimits(900, 50, 2, -2);


    // Begin fat controller threads
    control_thread = std::thread(&FatController::controlFriendly, this);     // sends to sim
    tracker_thread = std::thread(&FatController::updateGoal, this);          // sets goal and runs pursuit

    return sim_thread;
}

// Send info to the simulator
// Use current stored friendly velocity
void FatController::controlFriendly()
{
    while(true)
    {
        // Send to sim
        sim_ptr->controlFriendly(friendly.linear_velocity, friendly.angular_velocity);


        // Sleep untill we need to send again
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}


// 1. Choose Target
// 2. Choose future prediction
// 3. Run purepursuit
// 4. Update friendly velocities required
void FatController::updateGoal()
{
    while(true)
    {
        // Update friendly pose
        friendly.pose = sim_ptr->getFriendlyPose();

        // Current bogie positions / velocity
        auto bogies = target_control->estimate_aircraft();


        Aircraft target;

        double min_dist_to_bogie = 1000000;

        if(bogies.size() != 0)
        {
            std::vector<Pose> poses;
            for(auto bogie : bogies)
            {
                double t_lookahead = distance(friendly.pose.position, bogie.pose.position) / 1500;

                double dist = distance(friendly.pose.position, bogie.pose.position);
                if(dist < min_dist_to_bogie)
                {
                    target = bogie;
                    min_dist_to_bogie = dist;
                }
                //poses.push_back(predictFuturePosition(bogie, t_lookahead).pose);
                poses.push_back(bogie.pose);
            }

            Velocity v = tracker->track(friendly.pose, target.pose);
            friendly.linear_velocity = v.linear;
            friendly.angular_velocity = v.angular;
            sim_ptr->testPose(poses);


        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}


Aircraft FatController::predictFuturePosition(Aircraft ac, double time)
{
    double dx = ac.linear_velocity * cos(ac.pose.orientation) * time;
    double dy = ac.linear_velocity * sin(ac.pose.orientation) * time;

    ac.pose.position.x += dx;
    ac.pose.position.y += dy;

    return ac;
}

double FatController::distance(GlobalOrd p1, GlobalOrd p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}