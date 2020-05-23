#include "pathtracker.h"

void PathTracker::setVelocityLimits(double max_v_, double min_v_, double max_w_, double min_w_)
{
    max_v = max_v_;
    min_v = min_v_;

    max_w = max_w_;
    min_w = min_w_;
}