#pragma once 
#include "hybrid_a/hybrid_a.h"
#include "traj/traj_planner.h"
#include "cbs/planresult.hpp"

class single_planner
{
private:
    environment& env,&env2;
    carconfig &cfg;
    HybridAStarParam &hAParam;
    TrajPlannerConfig& config_;
public:
    explicit single_planner(environment &_env,environment &_env2, carconfig &_cfg,HybridAStarParam& _hAParam,TrajPlannerConfig &config);
    ~single_planner();
    bool plan(Vec3d& start,Vec3d& goal,PlanResult<Vec4d, int, double> &solution);

};

