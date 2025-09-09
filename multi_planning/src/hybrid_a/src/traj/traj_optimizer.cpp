#include "traj/traj_optimizer.h"

#include <ros/ros.h>

TrajOptimizer::TrajOptimizer(const TrajPlannerConfig& config)
    : config_(config), nlp_(config) {
  vehicle_ = config.vehicle;
}

bool TrajOptimizer::Optimize(std::vector<TrajectoryPoint>& coarse,
                             Constraints& constraints, States& result) {
  States guess;
  for (auto& pt : coarse) {
    guess.x.push_back(pt.x);
    guess.y.push_back(pt.y);
    guess.theta.push_back(pt.theta);
    guess.v.push_back(pt.velocity);
  }
  InitialGuess(guess,coarse);

  int iter = 0;
  double w_penalty = config_.opti_w_penality0;

  Constraints iterative_constraints = constraints;
  while (iter < config_.opti_iter_max) {
    double cur_infeasibility =
        nlp_.solve(w_penalty, iterative_constraints, guess,coarse, guess);
    ROS_INFO("iter = %d, cur_infeasibility = %f, w_penalty = %f", iter,
             cur_infeasibility, w_penalty);
    if (cur_infeasibility < config_.opti_varepsilon_tol) {
      result = guess;
      return true;
    } else {
      w_penalty *= config_.opti_alpha;
      iter++;
    }
  }
  return false;
}

/**
 * 粗糙解转换为优化问题的初值
 */
void TrajOptimizer::InitialGuess(States& states,const std::vector<TrajectoryPoint>& coarse) {
  // states.v.resize(config_.nfe, 0.0);
  states.phi.resize(config_.nfe, 0.0);

  double hi = config_.tf / (config_.nfe - 1);  //时间等间隔采样
  //求速度，转向角
  for (size_t i = 1; i < states.x.size(); i++) {
    double velocity =
        hypot(states.y[i] - states.y[i - 1], states.x[i] - states.x[i - 1]) /
        hi;
    double phi = atan((states.theta[i] - states.theta[i - 1]) *
                      vehicle_.wheel_base / (states.v[i] * hi));

    if(coarse[i].velocity<0){
      states.v[i] = std::max(-vehicle_.max_velocity, -velocity);
      // phi=-phi;
    }
    else{
      states.v[i] = std::min(vehicle_.max_velocity, velocity);
    }
    states.phi[i] =
        std::min(vehicle_.phi_max, std::max(-vehicle_.phi_max, phi));
  }

  //求加速度，角速度
  states.a.resize(config_.nfe, 0.0);
  states.omega.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.a[i] = std::min(vehicle_.max_acceleration,
                           std::max(vehicle_.min_acceleration,
                                    (states.v[i] - states.v[i - 1]) / hi));
    states.omega[i] = std::min(
        vehicle_.omega_max, std::max(-vehicle_.omega_max,
                                     (states.phi[i] - states.phi[i - 1]) / hi));
  }

  //求加加速度
  states.jerk.resize(config_.nfe, 0.0);
  for (size_t i = 1; i < states.x.size(); i++) {
    states.jerk[i] = std::min(
        vehicle_.jerk_max,
        std::max(-vehicle_.jerk_max, (states.a[i] - states.a[i - 1] / hi)));
  }
}
