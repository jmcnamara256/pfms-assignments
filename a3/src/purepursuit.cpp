#include "purepursuit.h"
#include <cmath>
#include <iostream>

PurePursuit::PurePursuit(double LOOK_AHEAD_DIST_, double MAX_G_, double ANGULAR_THRESHOLD_) : 
    LOOK_AHEAD_DIST(LOOK_AHEAD_DIST_), MAX_G(MAX_G_), ANGULAR_THRESHOLD(ANGULAR_THRESHOLD_) {}

PurePursuit::PurePursuit()
{
    LOOK_AHEAD_DIST     = DEFAULT_LOOK_AHEAD;
    MAX_G               = DEFAULT_MAX_G;
    ANGULAR_THRESHOLD   = DEFAULT_ANGULAR_THRESHOLD;
}



double PurePursuit::distance(GlobalOrd p1, GlobalOrd p2)
{
    return sqrt(pow(p2.x - p1.x, 2) + pow(p2.y - p1.y, 2));
}


double PurePursuit::constrainTo2Pi(double angle)
{
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0)
    {
        angle += 2 * M_PI;
    }
    return angle;
}

double PurePursuit::constrainToPi(double angle)
{
    angle = fmod((angle + M_PI), 2 * M_PI);

    // Account for a negative input angle
    if(angle < 0)
    {
        angle += (2 * M_PI);
    }

    // Output wrapped between -PI,PI
    return angle - M_PI; 
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
    // Between 0,2pi. Work in this domain.
    double alpha = constrainTo2Pi(atan2(dy, dx));

    // Bearing from start to goal, relative to starting orientation.
    double phi = constrainToPi(-start.orientation + alpha);

    if(fabs(phi) > ANGULAR_THRESHOLD)
    {
        // Dont do purepusuit yet, just turn quickly.
        
        // Ideal angular velocity to achieve maximum g's. Assumes minimum linear velocity.
        double w = sign(phi) * MAX_G * GRAVITY / min_v;

        // (min_v, w) will definitely result in an accelleration under 6 g. Also check w is within specified bounds.
        // This is not relevant to A3, but as a general case.
        if(w > max_w)
        {
            w = max_w;
        }
        else if (w < min_w)
        {
            w = min_w;
        }

        // Return the result
        return {min_v, w};
    }
    else
    {
        // Otherwise we are doing purepursuit

        // x coord of goal in relative space.
        double x = LOOK_AHEAD_DIST * sin(phi);

        // Ratio of w/v
        double gamma = 2 * x / (pow(LOOK_AHEAD_DIST, 2));

        // Find w from gamma, to attain maximum g's
        double w = sign(gamma) * sqrt(MAX_G * GRAVITY * fabs(gamma));
        
        // Return result.
        return {w/gamma, w};
    }
}


/**
 * @brief Set the maximum angle for purepursuit.
 * If angle between current and goal is >Threshold, then turn hard instead or pursuit.
 * 
 * @param angle 
 */
void PurePursuit::setAngularThreshold(double angle)
{
    ANGULAR_THRESHOLD = angle;
}