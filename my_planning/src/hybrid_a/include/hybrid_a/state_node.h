#pragma once

#include <Eigen/Dense>
#include <map>
#include "type.h"

struct StateNode {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum NODE_STATUS { NOT_VISITED = 0, IN_OPENSET, IN_CLOSESET };

  enum DIRECTION { FORWARD = 0, BACKWARD = 1, NO = 3 };

  StateNode() = delete;  //没有默认构造函数
  explicit StateNode(const Vec3i& grid_index) {
    node_status_ = NOT_VISITED;
    grid_index_ = grid_index;
    parent_node_ = nullptr;
  }

  void Reset() {
      node_status_ = NOT_VISITED;
      parent_node_ = nullptr;
  }

  NODE_STATUS node_status_; 
  DIRECTION direction_{};

  int time; //引入时域
  Vec3d state_;
  Vec3i grid_index_{};

  double g_cost_{}, f_cost{};
  int steering_grade_{};

  StateNode* parent_node_;
  
  typedef StateNode* ptr;
  std::multimap<double,StateNode::ptr>::iterator stateIt;

  VectorVec4d intermediate_states_;   //for rs_path
};
