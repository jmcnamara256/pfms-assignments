#include "purepursuit.h"
#include <cmath>
#include <iostream>

PurePursuit::PurePursuit(double LOOK_AHEAD_DIST_, double MAX_G_) : LOOK_AHEAD_DIST(LOOK_AHEAD_DIST_), MAX_G(MAX_G_) {}


double PurePursuit::pDistance(GlobalOrd start, GlobalOrd end, GlobalOrd p)
{
    // We want line in the form
    // Ax + By + C = 0

    // Slope - used to calc A/B/C
    double m = (end.y - start.y) / (end.x - start.x);

    // Calculate A, B, C
    // Derived this from standard 2-point-form of a line
    double A = -m;
    double B = 1;
    double C = m * start.x - start.y;

    // Perpendicular distance formular
    return std::fabs(p.x * A + p.y * B + C) / std::sqrt(A*A + B*B);
}

double PurePursuit::distance(GlobalOrd p1, GlobalOrd p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}

double PurePursuit::piTo2pi(double angle)
{
    if(angle < 0)
    {
        return angle + (2 * M_PI);
    }
    else
    {
        return angle;
    }
}

double PurePursuit::pi2Topi(double angle)
{
    if(angle > M_PI)
    {
        return angle - (2 * M_PI);
    }
    else
    {
        return angle;
    }  
}

double PurePursuit::sign(double num)
{
    if(num >= 0)
    {
        return 1;
    }
    else
    {
        return -1;
    }
}


Velocity PurePursuit::track(Pose start, Pose end)
{
    // Assume path is linear
    // Parameterise path as x=at+x1, y=bt+y1

    // Determine the parameter t, based on the lookahead distance
    double t = LOOK_AHEAD_DIST / distance(start.position, end.position);

    // Evaluate the goal x,y based on the parameter
    double x_goal = (end.position.x - start.position.x) * t + start.position.x;
    double y_goal = (end.position.y - start.position.y) * t + start.position.y;


    // delta between start and goal, in the global space 
    double dx = x_goal - start.position.x;
    double dy = y_goal - start.position.y;

    // Bearing of the goal, in the global space
    // Between -pi/pi. Work in this domain.
    double alpha = atan2(dy, dx);

    // Bearing from start to goal, relative to starting orientation.
    double phi = pi2Topi(start.orientation) - alpha - M_PI/2;

    // x coord of goal in relative space.
    double x = LOOK_AHEAD_DIST * cos(phi);

    std::cout << phi << std::endl;

    // Ratio of w/v
    double gamma = -2 * x / (pow(LOOK_AHEAD_DIST, 2));
    double w = sign(gamma) * sqrt(MAX_G * 9.81 * fabs(gamma));
    
    return {w/gamma, w};
}