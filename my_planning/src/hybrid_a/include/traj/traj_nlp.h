#pragma once

#include <casadi/casadi.hpp>
#include <vector>
#include "traj/traj_planner_config.h"
#include "traj/discretized_traj.h"

using namespace casadi;

struct States{
    std::vector<double> x,y,theta,v,phi,a,omega,jerk;
    std::vector<double> xf, yf, xr, yr;
};

struct Constraints{
    double start_x,start_y,start_theta,start_v,start_phi,start_a,start_omega;
    std::vector<std::array<double, 4>> front_bound;
    std::vector<std::array<double, 4>> rear_bound;
};

class TrajectoryNLP{
public:
    explicit TrajectoryNLP(const TrajPlannerConfig& config);
    double solve(double inf_w,const Constraints& constraints,const States& guess,const std::vector<TrajectoryPoint>& reference,States& result);
private:
    void BuildNLP();
private:
    TrajPlannerConfig config_;
    Dict nlp_config_;
    Function solver_;
    Function infeasibility_evaluator_;
};
