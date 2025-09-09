#include "single_planner.h"
#include "hybrid_a/middlePark.h"
#include <cfloat>
#include "traj/traj_planner_config.h"
#include "visual_rviz/plot.h"
#include "csv/CSVWriter.h"
#include <boost/algorithm/string/replace.hpp>
//大部分和混合A*算法一致
// #define NO_MIDDLE 1
single_planner::single_planner(environment &_env,environment &_env2, carconfig &_cfg, HybridAStarParam &_hAParam,TrajPlannerConfig &config)
    : env(_env),env2(_env2), cfg(_cfg), hAParam(_hAParam),config_(config)
{
}

single_planner::~single_planner()
{
}

bool single_planner::plan(Vec3d &start, Vec3d &goal,
                          PlanResult<Vec4d, int, double> &solution)
{
    std::vector<middlePoint> mp_vector;
    Vec3d middle;
    HybridAStar hybridA(env2, cfg, hAParam);
    solution.states.clear();
    solution.actions.clear();
    solution.cost = 0;
    solution.fmin = 0;

//这个宏定义是为了选择倒车和前进的规划
#ifndef NO_MIDDLE    
    // 求中间点
    calMiddlePoint(mp_vector, goal,
                   cfg.wheel_base_ / std::tan(cfg.steering_radian_));
    middle = Vec3d(mp_vector[0].pos);
    int min_index = 0;
    double min_theta = DBL_MAX;
    //用于选择合适的中间点
    double distance_start2final = hypot(goal.x() - start.x(),
                                        goal.y() - start.y());
    double theta_start2final = atan2(goal.y() - start.y(),
                                     goal.x() - start.x());
    for (unsigned int i = 0; i < mp_vector.size(); i++)
    {
        /* 此处使用d_theta来判断选择哪个中间点，文章中使用DM图来选择哪个中间点 */
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
#else 
    middle=goal;
#endif

    if (hybridA.search(start, middle))
    {
        
        auto path=hybridA.getPath();

        solution.states.insert(solution.states.end(),path.begin(),path.end());
        for(int i=0;i<path.size();i++){
            solution.actions.push_back(path[i].w());
            // csv.newRow()<<solution.states[i].x()<<solution.states[i].y()<<solution.states[i].z()<<solution.states[i].w();
        }
        

        // solution.actions.insert(solution.states.end(),path.begin(),path.end());
#ifndef NO_MIDDLE   
        TypeVectorVecd<4> middlePath = getMiddlePath(mp_vector[min_index], goal, cfg.move_step_size_);
        for(int i=0;i<middlePath.size();i++){
            middlePath[i].z()=hybridA.mod2pi(middlePath[i].z());
        }
        visual_rviz::PlotTraj(middlePath, 0.1, visual_rviz::Color::Green, 1,
                                     "coarse_back trajectory");
        visual_rviz::PlotTraj(solution.states, 0.1, visual_rviz::Color::Blue, 1,
                                     "coarse trajectory");
        for (size_t i = 0; i < middlePath.size(); i++)
        {
            solution.states.push_back(Vec4d{middlePath[i].x(),middlePath[i].y(),middlePath[i].z(),middlePath[i].w()});
            // std::cout<<solution.states[i].z()<<std::endl;
            solution.actions.push_back(-1);
        }
#else
        visual_rviz::PlotTraj(solution.states, 0.1, visual_rviz::Color::Blue, 1,
                                     "coarse trajectory");
#endif

        std::string hya(__FILE__);
        boost::replace_all(hya, "src/single_planner.cpp", "record/hya.csv");
        CSVWriter csv;
        csv.newRow()<<"x"<<"y"<<"z"<<"w";
        for(int i=0;i<solution.states.size();i++){
            csv.newRow()<<solution.states[i].x()<<solution.states[i].y()<<solution.states[i].z()<<solution.states[i].w();
        }
        csv.writeToFile(hya);


        std::string preci(__FILE__);
        boost::replace_all(preci, "src/single_planner.cpp", "record/preci.csv");
        CSVWriter csv2;
        csv2.newRow()<<"x"<<"y"<<"z"<<"w";
        // smooth path
        std::vector<TrajectoryPoint> coarsepath_;
        std::vector<TrajectoryPoint> result;
        
        for (int i = 0; i < solution.states.size(); i++)
        {
            coarsepath_.push_back(
                {0, solution.states[i].x(), solution.states[i].y(), solution.states[i].z(), 0, (double)solution.actions[i]});
        }
        config_.nfe = coarsepath_.size();

        TrajPlanner::StartState state_{coarsepath_[0].x, coarsepath_[0].y, coarsepath_[0].theta, 0.0, 0.0, 0.0, 0.0};
        TrajPlanner traj_planner(config_,env);
        if (traj_planner.Plan(coarsepath_, state_, result))
        {
            visual_rviz::PlotTraj(result, 0.1, visual_rviz::Color::Red, 1, "refined trajectory");
            for (size_t i = 0; i < result.size(); i++)
            {
                csv2.newRow()<<result[i].x<<result[i].y<<result[i].theta<<result[i].velocity;
            }
        }
        csv2.writeToFile(preci);
        visual_rviz::Trigger();
        return true;
    }
    return false;
}