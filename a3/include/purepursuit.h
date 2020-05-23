#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "types.h"
#include "pathtracker.h"

const double GRAVITY = 9.81;

class PurePursuit : public PathTracker
{
    public:
        PurePursuit();
        PurePursuit(double LOOK_AHEAD_DIST_, double MAX_G_, double ANGULAR_THRESHOLD_);
        Velocity track(Pose start, Pose goal);

        void setAngularThreshold(double angle);

    private:
        double distance(GlobalOrd p1, GlobalOrd p2);

        double constrainTo2Pi(double angle);
        double constrainToPi(double angle);
        double sign(double num);


        double LOOK_AHEAD_DIST;        // How far away can the target point along the path be?
        double MAX_G;
        double ANGULAR_THRESHOLD = M_PI;


        // Default values for plain constructor
        const double DEFAULT_LOOK_AHEAD         = 200;
        const double DEFAULT_MAX_G              = 6;
        const double DEFAULT_ANGULAR_THRESHOLD  = M_PI;
};

#endif