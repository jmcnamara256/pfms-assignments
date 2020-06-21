#ifndef PATHTRACKER_H
#define PATHTRACKER_H

#include "types.h"

/**
 * @brief Parent of  path tracking classes.
 * 
 */
class PathTracker
{
    public:
        // Track a pose
        virtual Velocity track(Pose start, Pose goal) = 0;

        void setVelocityLimits(double max_v_, double min_v_, double max_w_, double min_w_);

    protected:
        // Velocity limits, with default values.
        // These should be changed to suit the application

        // The linear velocity limits
        double max_v = 1;
        double min_v = 0;

        // Angular velocity limits
        double max_w = -1;
        double min_w =  1; 
        
};

#endif