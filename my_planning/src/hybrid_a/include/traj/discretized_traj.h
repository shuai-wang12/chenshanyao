#pragma once
#include <vector>
struct TrajectoryPoint
{
    double s=0;
    double x=0;
    double y=0;
    double theta=0;
    double kappa=0;
    double velocity=0;
};
bool readCoarsePath(std::vector<TrajectoryPoint>& coarse_path,
        std::vector<TrajectoryPoint>& middle_path,int& size);
