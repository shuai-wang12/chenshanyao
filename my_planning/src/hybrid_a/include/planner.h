#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>

#include <memory>
#include "hybrid_a/hybrid_a.h"
#include "subscribers.h"
#include "voronoi/dynamicvoronoi.h"
#include "voronoi/voronoiedge.h"
#include "hybrid_a/middlePark.h"
#include "hybrid_a/hybrid_a_config.h"
#include "carconfig.h"
#include "environments.h"
#include "traj/traj_planner.h"
#include "visual_rviz/plot.h"
#include "cbs/planresult.hpp"

class Planner
{
public:
    enum Status{
        IDEL    =0,
        SETTING ,
        PLANNING ,
        COMPLETE,
        ERROR
    };
public:
    Planner() = delete;
    explicit Planner(environment &_env,environment &_env2, carconfig &_cfg,HybridAStarParam& _hAParam,TrajPlannerConfig& config);
    void setInitPose(Vec3d &start);
    void setGoalPose(Vec3d &goal);
    void setInitGoalPoses();
    int Run();

private:
    void carsVisualize(std::vector<PlanResult<Vec4d, int, double>>& solution);
    void carVisualize(Vec3d &pos,visual_rviz::Color color,int id, const std::string &ns);

private:
    std::shared_ptr<HybridAStar> kinodynamic_astar_searcher_ptr_;

    std::vector<middlePoint> mp_vector;

    ros::Time timestamp_;

    std::shared_ptr<TrajPlanner> traj_planner;

    //多车的起点终点
    std::vector<Vec3d> starts;
    std::vector<Vec3d> goals;

public:
    environment &env , &env2;
    carconfig &cfg;
    HybridAStarParam &hAParam;
    TrajPlannerConfig& config_;
    // HybridAStar hybridA;
    Status planner_status;
    bool has_map_{};
    bool has_start{}, has_goal{};
};