#include "traj/traj_planner.h"
#include <ros/ros.h>

bool TrajPlanner::Plan(std::vector<TrajectoryPoint>& coarse_path,const StartState& state,std::vector<TrajectoryPoint>& result){
    //coarse_trajectory 每个采样点的时间都是一样的，因此每个点的距离应该差不多吧
    
    //起点边界约束
    Constraints opti_constraints;
    opti_constraints.start_x=state.x;
    opti_constraints.start_y=state.y;
    opti_constraints.start_v=state.v;
    opti_constraints.start_a=state.a;
    opti_constraints.start_omega=state.omega;
    opti_constraints.start_theta=state.theta;
    opti_constraints.start_phi=state.phi;

    States optimized;
    if(!opti_.Optimize(coarse_path,opti_constraints,optimized)){
        ROS_ERROR("Optimization failed");
        return false;
    }

    std::vector<double> opti_x,opti_y,opti_v;
    result.clear();
    double incremental_s=0.0;
    for(int i=0;i<config_.nfe;i++){
        TrajectoryPoint tp;
        incremental_s += i > 0 ? hypot(optimized.x[i] - optimized.x[i-1], optimized.y[i] - optimized.y[i-1]) : 0.0;
        tp.s = incremental_s;

        tp.x = optimized.x[i];
        tp.y = optimized.y[i];
        tp.theta = optimized.theta[i];
        tp.velocity = optimized.v[i];
        tp.kappa = tan(optimized.phi[i]) / config_.vehicle.wheel_base;

        result.push_back(tp);
    }
    return true; 
}