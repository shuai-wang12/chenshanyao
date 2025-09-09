#pragma once

#include "traj/traj_nlp.h"
#include "traj/traj_optimizer.h"
#include "traj/traj_planner_config.h"
#include "traj/discretized_traj.h"
#include <vector>

class TrajPlanner{
public:
    struct StartState{
        double x,y,theta,v,phi,a,omega;
    };
    explicit TrajPlanner(const TrajPlannerConfig& config,const environment& _env)
    :opti_(config,_env),config_(config){
        
    }
    bool Plan(std::vector<TrajectoryPoint>& coarse_path,const StartState& state,std::vector<TrajectoryPoint>& result);
private:
    TrajOptimizer opti_;
    TrajPlannerConfig config_;
};