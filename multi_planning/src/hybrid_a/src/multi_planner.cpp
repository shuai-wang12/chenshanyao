#include "multi_planner.h"

multi_planner::multi_planner(environment &_env, carconfig &_cfg,HybridAStarParam& _hAParam,TrajPlannerConfig &config)
    :env(_env), cfg(_cfg), hAParam(_hAParam),config_(config)
{

}

multi_planner::~multi_planner()
{
}

bool multi_planner::plan(std::vector<Vec3d> &initialStates,
                    std::vector<Vec3d> &goalStates,std::vector<PlanResult<Vec4d, int, double>> &solution){
    
}