#pragma once

#include <glog/logging.h>

#include <map>
#include <memory>

#include "rs_path.h"
#include "state_node.h"
#include "environments.h"
#include "dubins.h"
#include "hybrid_a_config.h"

class HybridAStar
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    explicit HybridAStar(const environment &_env, const carconfig &_cfg,
                         const HybridAStarParam &_hAParam)
        : env(_env), cfg(_cfg), hAParam(_hAParam)
    {
        rs_path_ptr_ =
            std::make_shared<RSPath>(_cfg.wheel_base_ / std::tan(_cfg.steering_radian_));
    }

    ~HybridAStar();
    bool AnalyticExpansions(const StateNode::ptr &current_node_ptr,
                            const StateNode::ptr &goal_node_ptr, double &length);
    inline double ComputeG(const StateNode::ptr &current_node_ptr,
                           const StateNode::ptr &neighbor_node_ptr) const;
    inline double ComputeH(const StateNode::ptr &current_node_ptr,
                           const StateNode::ptr &terminal_node_ptr);
    void GetNeighborNodes(const StateNode::ptr &curr_node_ptr,
                          std::vector<StateNode::ptr> &neighbor_nodes);
    bool search(const Vec3d &start, const Vec3d &goal);
    void DynamicModel(const double &step_size, const double &phi,
                      double &x, double &y, double &theta);
    VectorVec4d getPath() const;
    double mod2pi(const double &x);

private:
    // 终点指针，地图指针
    StateNode::ptr terminal_node_ptr_ = nullptr, start_node_ptr = nullptr, goal_node_ptr = nullptr;
    std::multimap<double, StateNode::ptr> openset_;

    double path_length_ = 0.0;

    std::shared_ptr<RSPath> rs_path_ptr_;
    const environment &env;
    const carconfig &cfg;
    const HybridAStarParam &hAParam;
    // debug
    double check_collision_use_time = 0.0;
    int num_check_collision = 0;
    int visited_node_number_ = 0;
};
