#ifndef PUREPURSUIT_H
#define PUREPURSUIT_H

#include "types.h"
#include "pathtracker.h"

class PurePursuit : public PathTracker
{
    public:
        PurePursuit(double LOOK_AHEAD_DIST_, double MAX_G_);
        Velocity track(Pose start, Pose goal);

    private:
        // Perpendicular distance
        double pDistance(GlobalOrd start, GlobalOrd end, GlobalOrd p);

        double distance(GlobalOrd p1, GlobalOrd p2);

        double piTo2pi(double angle);
        double pi2Topi(double angle);
        double sign(double num);


        const double LOOK_AHEAD_DIST;        // How far away can the target point along the path be?
        const double MAX_G;
};

#endif