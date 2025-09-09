#include "single_planner.h"
#include "hybrid_a/middlePark.h"
#include <cfloat>
#include "traj/traj_planner_config.h"
#include "visual_rviz/plot.h"

single_planner::single_planner(environment &_env,environment &_env2, carconfig &_cfg, HybridAStarParam &_hAParam,TrajPlannerConfig &config,int id)
    : env(_env),env2(_env2), cfg(_cfg), hAParam(_hAParam),config_(config)
{
    this->id=id;
}

single_planner::~single_planner()
{
}

bool single_planner::plan(Vec3d &start, Vec3d &goal,
                          PlanResult<Vec4d, int, double> &solution,bool isPark)
{
    int min_index = 0;
    std::vector<middlePoint> mp_vector;
    Vec3d middle=goal;
    HybridAStar hybridA(env2, cfg, hAParam);
    solution.states.clear();
    solution.actions.clear();
    solution.cost = 0;
    solution.fmin = 0;
 
if(isPark){
    // 求中间点
    calMiddlePoint(mp_vector, goal,
                   cfg.wheel_base_ / std::tan(cfg.steering_radian_));
    middle = Vec3d(mp_vector[0].pos);
    
    double min_theta = DBL_MAX;
    double distance_start2final = hypot(goal.x() - start.x(),
                                        goal.y() - start.y());
    double theta_start2final = atan2(goal.y() - start.y(),
                                     goal.x() - start.x());
    for (unsigned int i = 0; i < mp_vector.size(); i++)
    {
        double d_theta;
        d_theta = theta_start2final - mp_vector[i].pos.z();
        d_theta = hybridA.mod2pi(d_theta);
        d_theta = abs(d_theta);
        if (!this->env2.checkCollision(mp_vector[i].pos.x(), mp_vector[i].pos.y(),
                                      mp_vector[i].pos.z()))
            continue;

        if (d_theta < min_theta)
        {
            min_theta = d_theta;
            min_index = i;
            middle = Vec3d(mp_vector[i].pos);
            // std::cout<<start_yaw<<","<<mp_vector[i].pos.z()<<std::endl;
        }
    }
}else{
    middle=goal;
}

    if (hybridA.search(start, middle))
    {
        auto path=hybridA.getPath();
        solution.states.insert(solution.states.end(),path.begin(),path.end());
        for(int i=0;i<path.size();i++){
            solution.actions.push_back(path[i].w());
        }
if(isPark) {
        TypeVectorVecd<4> middlePath = getMiddlePath(mp_vector[min_index], goal, cfg.move_step_size_);
        // visual_rviz::PlotTraj(middlePath, 0.1, visual_rviz::Color::Green, 1,
        //                              "coarse_back trajectory");
        for(int i=0;i<middlePath.size();i++){
            middlePath[i].z()=hybridA.mod2pi(middlePath[i].z());
        }
        for (size_t i = 0; i < middlePath.size(); i++)
        {
            solution.states.push_back(middlePath[i]);
            solution.actions.push_back(-1);
        }
}
else{
        // visual_rviz::PlotTraj(solution.states, 0.1, visual_rviz::Color::Blue, 1,
        //                              "coarse trajectory");
}
        // smooth path
        std::vector<TrajectoryPoint> coarsepath_;
        std::vector<TrajectoryPoint> result;
        
        for (int i = 0; i < solution.states.size(); i++)
        {
            coarsepath_.push_back(
                {0, solution.states[i].x(), solution.states[i].y(), solution.states[i].z(), 0, (double)solution.actions[i]});
        }
        config_.nfe = coarsepath_.size();

        TrajPlanner::StartState state_{coarsepath_[0].x, coarsepath_[0].y, coarsepath_[0].theta, 1.0, 0.0, 0.0, 0.0};
        TrajPlanner traj_planner(config_);
        if (traj_planner.Plan(coarsepath_, state_, result))
        {
            visual_rviz::PlotTraj(result, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
        }
        solution.states.clear();
        solution.actions.clear();
        for(int i=0;i<result.size();i++){
            solution.states.push_back(Vec4d{result[i].x,result[i].y,result[i].theta,result[i].velocity});
            solution.actions.push_back(static_cast<int>(result[i].velocity));
        }
        visual_rviz::TriggerRemain();
        return true;
    }
    return false;
}

bool single_planner::plan2(Vec3d &start,Vec3d& park, Vec3d &goal,
                          PlanResult<Vec4d, int, double> &solution)
{
    PlanResult<Vec4d, int, double> solution1;
    PlanResult<Vec4d, int, double> solution2;
    solution.states.clear();
    solution.actions.clear();
    solution.cost = 0;
    solution.fmin = 0;
    if(!plan(start,park,solution1,true))
        return false;
    if(!plan(park,goal,solution2,false))
        return false;
    solution.states.insert(solution.states.end(),solution1.states.begin(),solution1.states.end());
    solution.states.insert(solution.states.end(),solution2.states.begin(),solution2.states.end());
    solution.actions.insert(solution.actions.end(),solution1.actions.begin(),solution1.actions.end());
    solution.actions.insert(solution.actions.end(),solution2.actions.begin(),solution2.actions.end());
    solution.cost=solution1.cost+solution2.cost;
    solution.fmin=solution1.fmin+solution2.fmin;
    
    return true;

}