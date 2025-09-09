#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#include <boost/algorithm/string/replace.hpp>

#include "carconfig.h"
#include "planner.h"
#include "publish_test.h"
#include "subscribers.h"
#include "filling.hpp"
#include "visual_rviz/plot.h"
#include "hybrid_a/hybrid_a_config.h"
#include "clipper2/clipper.h"
#include "csv/CSVWriter.h"
#include "math.h"

bool **binMap;
bool **binMap2;
int map_width;
int map_height;
bool has_start=false,has_goal=false;
double map_resolution;
Vec3d start, goal;
std::vector<IntPoint> boundary;

std::vector<int> bx;
std::vector<int> by;
// #define TEST_CODE 1

void setBinMapFalse(int i, int j)
{
    if (i < 0 || j < 0 || i >= map_width || j >= map_height)
        return;
    binMap[i][j] = false;
}

void setBinMap2False(int i, int j)
{
    if (i < 0 || j < 0 || i >= map_width || j >= map_height)
        return;
    binMap2[i][j] = false;
}

void initMap(ros::NodeHandle &nh, environment &env)
{
    // std::vector<int> pathOffset;
    nh.getParam("map_boundary_x",bx);
    nh.getParam("map_boundary_y",by);
    for (size_t i = 0; i < bx.size(); i++)
    {
        boundary.push_back({bx[i],by[i]});
        // pathOffset.push_back(bx[i]);
        // pathOffset.push_back(by[i]);
    }
    
    bx.push_back(bx[0]),by.push_back(by[0]);
    visual_rviz::PlotLine(bx,by,0.4,visual_rviz::Color::Grey, 1, "boundary");
    bx.pop_back(),by.pop_back();
    // visual_rviz::Trigger();

    binMap = new bool *[map_width];
    for (int x = 0; x < map_width; x++)
    {
        binMap[x] = new bool[map_height];
        for (int y = 0; y < map_height; y++)
        {
            // 全部设置为障碍物
            binMap[x][y] = true;
        }
    }
    // //内缩
    // Clipper2Lib::Paths64 subject;
    // subject.push_back(Clipper2Lib::MakePath(pathOffset));
    // Clipper2Lib::Paths64 solution;
    // Clipper2Lib::ClipperOffset offsetter;
    // offsetter.AddPaths(subject,Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
    // offsetter.Execute(-2,solution);
    // solution = Clipper2Lib::SimplifyPaths(solution,0.2);
    
    // std::vector<int>bbx,bby;
    // for(int i=0;i<solution[0].size();i++){
    //     bbx.push_back((int)solution[0][i].x);
    //     bby.push_back((int)solution[0][i].y);
    // }
    // bbx.push_back(bbx[0]),bby.push_back(bby[0]);
    // visual_rviz::PlotLine(bbx,bby,0.4,visual_rviz::Color::Grey, 2, "boundaryOffset");
    // visual_rviz::Trigger();
    // 填充，可通行区域是false
    filling(boundary, setBinMapFalse);
    for (int i = 0; i < map_width; i++)
    {
        binMap[i][0] = true, binMap[i][map_height - 1] = true;
    }
    for (int i = 0; i < map_height; i++)
    {
        binMap[0][i] = true, binMap[map_width - 1][i] = true;
    }
    env.setObstacles(binMap);
    
    // std::string inputFile(__FILE__);
    // boost::replace_all(inputFile, "main.cpp", "record/map1.csv");
    // CSVWriter csv;
    // csv.newRow()<<"i"<<"j";
    // for(int i=0;i<map_width;i++){
    //     for (int j = 0; j < map_height; j++)
    //     {
    //         if (binMap[i][j])
    //         {
    //             csv.newRow()<<i<<j;
    //         }
    //     }
    // }
    // csv.writeToFile(inputFile);
    //回收
    // for (int x = 0; x < map_width; x++)
    // {
    //     delete[] binMap[x];
    // }
    // delete[] binMap;
}


void initMap2(ros::NodeHandle &nh, environment &env)
{
    std::vector<int> pathOffset;
    // nh.getParam("map_boundary_x",bx);
    // nh.getParam("map_boundary_y",by);
    for (size_t i = 0; i < bx.size(); i++)
    {
        // boundary.push_back({bx[i],by[i]});
        pathOffset.push_back(bx[i]);
        pathOffset.push_back(by[i]);
    }
    
    // bx.push_back(bx[0]),by.push_back(by[0]);
    // visual_rviz::PlotLine(bx,by,0.4,visual_rviz::Color::Grey, 1, "boundary");
    // visual_rviz::Trigger();

    binMap2 = new bool *[map_width];
    for (int x = 0; x < map_width; x++)
    {
        binMap2[x] = new bool[map_height];
        for (int y = 0; y < map_height; y++)
        {
            // 全部设置为障碍物
            binMap2[x][y] = true;
        }
    }
    //内缩
    Clipper2Lib::Paths64 subject;
    subject.push_back(Clipper2Lib::MakePath(pathOffset));
    Clipper2Lib::Paths64 solution;
    Clipper2Lib::ClipperOffset offsetter;
    offsetter.AddPaths(subject,Clipper2Lib::JoinType::Round,Clipper2Lib::EndType::Polygon);
    offsetter.Execute(-2,solution);
    solution = Clipper2Lib::SimplifyPaths(solution,0.2);
    
    // std::string inEdge(__FILE__);
    // boost::replace_all(inEdge, "main.cpp", "record/inedge.csv");
    // CSVWriter csv2;
    // csv2.newRow()<<"x"<<"y";

    std::vector<int>bbx,bby;
    for(int i=0;i<solution[0].size();i++){
        bbx.push_back((int)solution[0][i].x);
        bby.push_back((int)solution[0][i].y);
        // csv2.newRow()<<(int)solution[0][i].x<<(int)solution[0][i].y;
    }
    // csv2.writeToFile(inEdge);
    bbx.push_back(bbx[0]),bby.push_back(bby[0]);
    visual_rviz::PlotLine(bbx,bby,0.4,visual_rviz::Color::Grey, 2, "boundaryOffset");
    visual_rviz::Trigger();
    // 填充，可通行区域是false
    filling(solution[0], setBinMap2False);
    for (int i = 0; i < map_width; i++)
    {
        binMap2[i][0] = true, binMap2[i][map_height - 1] = true;
    }
    for (int i = 0; i < map_height; i++)
    {
        binMap2[0][i] = true, binMap2[map_width - 1][i] = true;
    }
    env.setObstacles(binMap2);
    //回收
    // for (int x = 0; x < map_width; x++)
    // {
    //     delete[] binMap[x];
    // }
    // delete[] binMap;
    // std::string inputFile(__FILE__);
    // boost::replace_all(inputFile, "main.cpp", "record/map2.csv");
    // CSVWriter csv;
    // csv.newRow()<<"i"<<"j";
    // for(int i=0;i<map_width;i++){
    //     for (int j = 0; j < map_height; j++)
    //     {
    //         if (binMap[i][j])
    //         {
    //             csv.newRow()<<i<<j;
    //         }
    //     }
    // }
    // csv.writeToFile(inputFile);
}

void goalCallback(const geometry_msgs::PoseStampedPtr &msg)
{
    if(!has_goal){
        double theta = tf::getYaw(msg->pose.orientation);
        // goal = Vec3d(msg->pose.position.x,
        //          msg->pose.position.y, theta);
        // goal = Vec3d(40-15*cos(M_PI*5/6),28-15*sin(M_PI*5/6),M_PI*5/6);
        // goal = Vec3d(55,28,0);
        // goal = Vec3d(55,28,0);
        // goal = Vec3d(83,37.8569,3.14159);
        has_goal=true;
    }
   
}

void initCallback(const geometry_msgs::PoseWithCovarianceStampedPtr &msg)
{
    if(!has_start){
        double theta = tf::getYaw(msg->pose.pose.orientation);
        // start = Vec3d(msg->pose.pose.position.x,
        //           msg->pose.pose.position.y, theta);
        // start = Vec3d(40+15*cos(M_PI*5/6),28+15*sin(M_PI*5/6),M_PI*5/6);
        // start = Vec3d(25,28,0);
        // start = Vec3d(25,28,0);
        // start = Vec3d(16.5,50,1.310193935+M_PI);
        has_start=true;
    }
    
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hybrid_a");
    ros::NodeHandle nh("~");
    visual_rviz::Init(nh, "world", "planner_markers");

    //设置终点
    double gx = nh.param("goal/x",1.0);
    double gy = nh.param("goal/y",1.0);
    double gtheta = nh.param("goal/theta",1.0);

    goal= Vec3d(gx,gy,gtheta);

//设置起点
    double sx = nh.param("start/x",1.0);
    double sy = nh.param("start/y",1.0);
    double stheta = nh.param("start/theta",1.0);

    start= Vec3d(sx,sy,stheta);

    //参数参考混合A*算法
    carconfig cfg;
    double steering_angle = nh.param("planner/steering_angle", 10.0);
    cfg.steering_radian_ = steering_angle * M_PI / 180.0;
    cfg.length_ = nh.param("planner/length", 4.7);
    cfg.width_ = nh.param("planner/width", 2.0);
    cfg.rear_ = nh.param("planner/rear", 1.3);
    cfg.segment_length_discrete_num_ =
        nh.param("planner/segment_length_discrete_num", 8);
    cfg.segment_length_ = nh.param("planner/segment_length", 1.6);
    cfg.wheel_base_ = nh.param("planner/wheel_base", 2.0);
    cfg.steering_discrete_num_ = nh.param("planner/steering_angle_discrete_num", 1);
    cfg.move_step_size_ = cfg.segment_length_ / (double)cfg.segment_length_discrete_num_;
    cfg.steering_radian_step_size_ = cfg.steering_radian_ / (double)cfg.steering_discrete_num_;
    cfg.setVehicleShape();

    double hybrid_a_resolution = nh.param("planner/resolution", 0.2);
    HybridAStarParam HAParam;
    TrajPlannerConfig traj_cfg;

    HAParam.steering_penalty_ = nh.param("planner/steering_penalty", 1.05);
    HAParam.steering_change_penalty_ =
        nh.param("planner/steering_change_penalty", 1.5);
    HAParam.reversing_penalty_ = nh.param("planner/reversing_penalty", 2.0);
    HAParam.shot_distance_ = nh.param("planner/shot_distance", 5.0);
    HAParam.tie_breaker_ = 1.0 + 1e-3;

    traj_cfg.vehicle.max_velocity = nh.param("traj_opti/vehicle/max_velocity", 5.0);
    traj_cfg.vehicle.min_acceleration = nh.param("traj_opti/vehicle/min_acceleration", -5.0);
    traj_cfg.vehicle.max_acceleration = nh.param("traj_opti/vehicle/max_acceleration", 5.0);
    traj_cfg.vehicle.jerk_max = nh.param("traj_opti/vehicle/jerk_max", 100.0);
    traj_cfg.vehicle.phi_max = nh.param("traj_opti/vehicle/phi_max", 0.26);
    traj_cfg.vehicle.omega_max = nh.param("traj_opti/vehicle/max_velocity", 1.0);

    map_width = nh.param("map_width", 60);
    map_height = nh.param("map_height", 60);
    map_resolution = nh.param("map_resolution", 1.0);
    

    // 初始化地图并显示
    environment env(0.0, map_width * map_resolution, 0.0, map_height * map_resolution, map_resolution, hybrid_a_resolution, 72, cfg);
    environment env2(0.0, map_width * map_resolution, 0.0, map_height * map_resolution, map_resolution, hybrid_a_resolution, 72, cfg);
    initMap(nh, env);
    initMap2(nh, env2);

    ros::Subscriber goal_pose_sub = nh.subscribe("/move_base_simple/goal", 1, &goalCallback);
    ros::Subscriber init_pose_sub = nh.subscribe("/initialpose", 1, &initCallback);

    /* 两个env，不是env，env2吗？ */
    Planner planner(env,env, cfg, HAParam,traj_cfg);

    ros::Rate rate(5);
    while (ros::ok())
    {
        // setplanner
#ifndef TEST_CODE
        if(has_start^has_goal){
            if(planner.planner_status==Planner::COMPLETE){
                visual_rviz::Clear();
                visual_rviz::PlotLine(bx,by,0.4,visual_rviz::Color::Grey, 1, "boundary");
                visual_rviz::Trigger();
                planner.planner_status=Planner::SETTING;
            }
            if(has_start){
                planner.setInitPose(start);
                has_start=false;
            }
            if(has_goal){
                planner.setGoalPose(goal);
                has_goal=false;
            }
        }
#else
//状态机罢了
        if(has_start^has_goal){
            planner.setInitGoalPoses();
            has_start=false;
            has_goal=false;
        }
#endif
        if(planner.Run()==Planner::COMPLETE){
            visual_rviz::Trigger();
            has_goal=false,has_start=false;
        }

        // publish_env(env_pub,planner.env.get_map_data(),planner.env.map_grid_sizeX,planner.env.map_grid_sizeY);
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}

