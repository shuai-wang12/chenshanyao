#include "planner.h"

#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/algorithm/string/replace.hpp>
#include <fstream>
#include "cbs/planresult.hpp"
#include "publish_test.h"
#include "traj/traj_planner_config.h"
#include "visual_rviz/plot.h"
#include "single_planner.h"

#include "cbs/cl_cbs.h"
#include "timer.h"
#define plan_once 1

Planner::Planner(environment &_env,environment &_env2, carconfig &_cfg, HybridAStarParam &_hAParam,TrajPlannerConfig& config)
    : env(_env),env2(_env2), cfg(_cfg), hAParam(_hAParam),config_(config)
{
    has_start = false;
    has_goal = false;
    planner_status=IDEL;
}

void Planner::setInitPose(Vec3d &start)
{
    if(planner_status!=PLANNING){
        starts.clear();
        starts.push_back(start);
        has_start = true;
        planner_status=SETTING;
        carVisualize(start,visual_rviz::Color::Yellow,1,"start_point");
        ROS_INFO("start:%.2f,%.2f,%.2f",start.x(),start.y(),start.z());
        // ROS_INFO("has_start");
    }
}
void Planner::setGoalPose(Vec3d &goal)
{
    if(planner_status!=PLANNING){
        goals.clear();
        goals.push_back(goal);
        has_goal = true;
        planner_status=SETTING;
        carVisualize(goal,visual_rviz::Color::Black,1,"goal_point");
        ROS_INFO("goal:%.2f,%.2f,%.2f",goal.x(),goal.y(),goal.z());
        // ROS_INFO("has_goal");
    }
}

void Planner::setInitGoalPoses(){
    if(planner_status!=PLANNING){
        planner_status=SETTING;
        goals.clear();
        starts.clear();
        starts.push_back(Vec3d(10,30,0));
        starts.push_back(Vec3d(30,10,M_PI_2));
        starts.push_back(Vec3d(44.1421356,44.1421356,-3*M_PI_4));
        goals.push_back(Vec3d(50,30,0));
        goals.push_back(Vec3d(30,50,M_PI_2));
        goals.push_back(Vec3d(15.85786437,15.85786437,-3*M_PI_4));
        has_goal = true;
        has_start=true;
        // planner_status=SETTING;
        for(int i=0;i<starts.size();i++){
            carVisualize(starts[i],visual_rviz::Color::Yellow,i,"start_point");
            carVisualize(goals[i],visual_rviz::Color::Black,i,"goal_point");
        }
        ROS_INFO("init_goals_starts");
    }
}

int Planner::Run()
{
    if(has_start==true&&has_goal==true&&planner_status==SETTING){
        planner_status=PLANNING;
        has_start=false;
        has_goal=false;
    }
    while (planner_status==PLANNING)
    {
        ROS_INFO("planning");

        // CL_CBS high_planner(env,cfg,hAParam,config_);
        // std::vector<PlanResult<Vec4d, int, double>> solution;
        // if(high_planner.search(starts,goals,solution)){
        //     carsVisualize(solution);
        //     planner_status=COMPLETE;
        // }
        // else{
        //     planner_status=ERROR;
        //     ROS_INFO("error");
        // }
        single_planner low_planner(env,env2,cfg,hAParam,config_);
        PlanResult<Vec4d,int,double> solutions;
        Timer search_used_time;
        if(low_planner.plan(starts[0],goals[0],solutions)){
            planner_status=COMPLETE;
            has_start = false, has_goal = false;
            ROS_INFO("complete");
        }
        else{
            planner_status=ERROR;
            ROS_INFO("error");
        }
        ROS_INFO(
            "\033[1;32m --> Time in Hybrid A star is %f ms\n",
            search_used_time.End());
    }
    return planner_status;
}

void Planner::carsVisualize(std::vector<PlanResult<Vec4d, int, double>>& solution){
    int max_time=0;
    for(int i=0;i<solution.size();i++){
        if(max_time<(solution[i].states.size()-1+solution[i].start_time)){
            max_time=(solution[i].states.size()-1+solution[i].start_time);
        }
    }
    for(int time=0;time<=max_time;time++){
        for(int i=0;i<solution.size();i++){
            if(time<solution[i].start_time){
                continue;
            }
            if(time-solution[i].start_time>=solution[i].states.size()){
                continue;
            }
            Vec3d pos=(solution[i].states[time-solution[i].start_time]).head(3);
            carVisualize(pos,visual_rviz::Color::Cyan,i,"moving_path");
        }
        ros::Duration(0.1).sleep();
    }
}

//坐标变换 显示车子
void Planner::carVisualize(Vec3d &pos,visual_rviz::Color color,int id, const std::string &ns){
    // 显示车子
    std::vector<double> dx;
    std::vector<double> dy;
    double theta = pos.z();
    Mat2d R;
    R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);
    MatXd transformed_vehicle_shape;
    transformed_vehicle_shape.resize(8, 1);
    for (unsigned int i = 0; i < 4u; ++i)
    {
        transformed_vehicle_shape.block<2, 1>(i * 2, 0) =
            R * cfg.vehicle_shape_.block<2, 1>(i * 2, 0) + Vec2d(pos.x(), pos.y());
        dx.push_back(transformed_vehicle_shape.block<2, 1>(i * 2, 0)[0]);
        dy.push_back(transformed_vehicle_shape.block<2, 1>(i * 2, 0)[1]);
    }
    visual_rviz::PlotPolygon(dx, dy, 0.1, color, id,
                                ns);
    visual_rviz::Trigger();
}

