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
#include "config.h"
#include "csv/CSVWriter.h"

Planner::Planner(environment &_env,environment &_env2, carconfig &_cfg, HybridAStarParam &_hAParam, TrajPlannerConfig &config)
    : env(_env),env2(_env2), cfg(_cfg), hAParam(_hAParam), config_(config)
{
    has_start = false;
    has_goal = false;
    planner_status = IDEL;
}

void Planner::setInitPose(Vec3d &start)
{
    if (planner_status != PLANNING)
    {
        starts.clear();
        starts.push_back(start);
        has_start = true;
        // planner_status=SETTING;
        carVisualize(start, visual_rviz::Color::Yellow, 1, "start_point");
        ROS_INFO("has_start");
    }
}
void Planner::setGoalPose(Vec3d &goal)
{
    if (planner_status != PLANNING)
    {
        goals.clear();
        goals.push_back(goal);
        has_goal = true;
        // planner_status=SETTING;
        carVisualize(goal, visual_rviz::Color::Black, 1, "goal_point");
        ROS_INFO("has_goal");
    }
}

void Planner::setInitGoalPoses()
{
    if (planner_status != PLANNING)
    {
        planner_status = SETTING;
        goals.clear();
        starts.clear();
#ifdef isProgress
        // starts.push_back(Vec3d(16.5, 50, -1.85292));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58, 4.04, 0.02));
        // starts.push_back(Vec3d(4.58,4.04,0.02));

        // parks.push_back(Vec3d(80.6345, 37.8569, -3.12844));
        // parks.push_back(Vec3d(19.73, 56.38, -0.95));
        // parks.push_back(Vec3d(52.03, 56.35, -1.55));
        // parks.push_back(Vec3d(11.22, 48.17, -0.75));
        // parks.push_back(Vec3d(28.99, 56.02, -1.57));
        // parks.push_back(Vec3d(46.13, 55.93, -1.54));
        // parks.push_back(Vec3d(39.85, 56.05, -1.55));
        // parks.push_back(Vec3d(56.10,51.65,-3.09));

        // goals.push_back(Vec3d(27, 50, 1.28867));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58, 4.04, 0.02));
        // goals.push_back(Vec3d(54.58,4.04,0.02));
#else
        starts.push_back(Vec3d(10, 30, 0));
        starts.push_back(Vec3d(30, 10, M_PI_2));
        starts.push_back(Vec3d(44.1421356, 44.1421356, -3 * M_PI_4));
        goals.push_back(Vec3d(50, 30, 0));
        goals.push_back(Vec3d(30, 50, M_PI_2));
        goals.push_back(Vec3d(15.85786437, 15.85786437, -3 * M_PI_4));
#endif
        /* 现在这样写，starts, parks, goals都没有数 */
        has_goal = true;
        has_start = true;
        // planner_status=SETTING;
        for (int i = 0; i < starts.size(); i++)
        {
            carVisualize(starts[i], visual_rviz::Color::Yellow, i, "start_point");
            carVisualize(goals[i], visual_rviz::Color::Black, i, "goal_point");
        }
        ROS_INFO("init_goals_starts");
    }
}

int Planner::Run()
{
    if (has_start == true && has_goal == true && planner_status == SETTING)
    {
        planner_status = PLANNING;
        has_start = false;
        has_goal = false;
    }
    while (planner_status == PLANNING)
    {
        ROS_INFO("planning");
        CL_CBS high_planner(env,env2, cfg, hAParam, config_);
        std::vector<PlanResult<Vec4d, int, double>> solution;
#ifdef isProgress
        //只有cbs
        if (high_planner.searchProgress(starts, parks, goals, solution))
        {
#else
        if (high_planner.search(starts, goals, solution))
        {
#endif

            // multi_opti(solution);//优化时间

            planCSVPrint(solution);
            carsVisualize(solution);
            planner_status = COMPLETE;
            ROS_INFO("last start time %d",solution.back().start_time);
            ROS_INFO("complete");
        }
        else
        {
            planner_status = ERROR;
            ROS_INFO("error");
        }
    }
    return planner_status;
}

//求车辆近似圆的圆心


void Planner::planCSVPrint(std::vector<PlanResult<Vec4d, int, double>> &solution)
{
    
    for (int i = 0; i < solution.size(); i++)
    {
        std::string inputFile(__FILE__);
        boost::replace_all(inputFile, "src/planner.cpp", "record/solutions");
        inputFile+=std::to_string(i)+".csv";
        CSVWriter csv;
        csv.newRow() << "x"
                     << "y"
                     << "theta"
                     << "t";
        for (int j = 0; j < solution[i].states.size(); j++)
        {
            csv.newRow() << solution[i].states[j].x() << solution[i].states[j].y() << solution[i].states[j].z() << solution[i].start_time+j;
        }
        csv.writeToFile(inputFile);
    }
}
 //这是只打印那些合法时间下的车辆点
void Planner::carsVisualize(std::vector<PlanResult<Vec4d, int, double>> &solution)
{
    //求最大时间，合法的应该小于这个时间
    int max_time = 0;
    for (int i = 0; i < solution.size(); i++)
    {
        if (max_time < (solution[i].states.size() - 1 + solution[i].start_time))
        {
            max_time = (solution[i].states.size() - 1 + solution[i].start_time);
        }
    }
    for (int time = 0; time <= max_time; time++)
    {
        for (int i = 0; i < solution.size(); i++)
        {
            if (time < solution[i].start_time)
            {
                continue;
            }
            if (time - solution[i].start_time >= solution[i].states.size())
            {
                continue;
            }
            Vec3d pos = (solution[i].states[time - solution[i].start_time]).head(3);
            carVisualize(pos, visual_rviz::Color::Cyan, i, "moving_path");
        }
        ros::Duration(0.025).sleep();
    }
}

void Planner::carVisualize(Vec3d &pos, visual_rviz::Color color, int id, const std::string &ns)
{
    // 显示车子,坐标变换
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
