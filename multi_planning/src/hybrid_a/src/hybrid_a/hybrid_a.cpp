#include "hybrid_a/hybrid_a.h"
#include "timer.h"
#include <ros/ros.h>

HybridAStar::~HybridAStar(){
    for(int i=0;i<env.state_sizeX;i++){

        for(int j=0;j<env.state_sizeY;j++){
            for(int k=0;k<env.state_sizePHI;k++){
                if(env.state_node_map[i][j][k]!=nullptr){
                    delete env.state_node_map[i][j][k];
                    env.state_node_map[i][j][k]=nullptr;
                }
            }
        }
    }
}

bool HybridAStar::search(const Vec3d& start,const Vec3d& goal){
    double neighbor_time = 0.0, compute_h_time = 0.0, compute_g_time = 0.0;

    const Vec3i start_index=env.state2index(start);
    const Vec3i goal_index=env.state2index(goal);

    goal_node_ptr=new StateNode(goal_index);
    goal_node_ptr->state_ = goal;
    goal_node_ptr->direction_=StateNode::NO;
    goal_node_ptr->steering_grade_=0;

    start_node_ptr=new StateNode(start_index);
    start_node_ptr->state_=start;
    start_node_ptr->steering_grade_=0;
    start_node_ptr->direction_=StateNode::NO;
    start_node_ptr->node_status_=StateNode::IN_OPENSET;
    start_node_ptr->intermediate_states_.emplace_back(Vec4d(start.x(),start.y(),start.z(),1));
    start_node_ptr->g_cost_=0;
    start_node_ptr->f_cost=ComputeH(start_node_ptr,goal_node_ptr);
    ROS_INFO("start_node_init");
    env.state_node_map[start_index.x()][start_index.y()]
                    [start_index.z()]=start_node_ptr;
    env.state_node_map[goal_index.x()][goal_index.y()]
                    [goal_index.z()]=goal_node_ptr;
    ROS_INFO("state_node_map_init");
    openset_.clear();
    start_node_ptr->stateIt=openset_.insert(std::make_pair(0,start_node_ptr));

    std::vector<StateNode::ptr> neighbor_nodes_ptr;
    StateNode::ptr current_node_ptr;
    StateNode::ptr neighbor_node_ptr;

    int count=0;
    while(!openset_.empty()){
        current_node_ptr=openset_.begin()->second;
        current_node_ptr->node_status_=StateNode::IN_CLOSESET;
        openset_.erase(openset_.begin());
        if((current_node_ptr->state_.head(2)-goal_node_ptr->state_.head(2))
            .norm()<hAParam.shot_distance_){
            double rs_length=0.f;
            //找到答案，直接返回terminal_node_ptr
            if(AnalyticExpansions(current_node_ptr,goal_node_ptr,rs_length)){
                terminal_node_ptr_=goal_node_ptr;
                //它的目的是计算和记录长度
                StateNode::ptr grid_node_ptr=terminal_node_ptr_->parent_node_;
                while(grid_node_ptr!=nullptr){
                    grid_node_ptr=grid_node_ptr->parent_node_;
                    path_length_+=cfg.segment_length_;
                }
                path_length_=path_length_-cfg.segment_length_+rs_length;
                ROS_INFO("answer");
                return true;
            }
            // ROS_INFO("AnalyticExpansions_no");
        }
        GetNeighborNodes(current_node_ptr,neighbor_nodes_ptr);
        // ROS_INFO("getneighbors_ok");
        for(unsigned int i=0;i<neighbor_nodes_ptr.size();i++){
            neighbor_node_ptr=neighbor_nodes_ptr[i];
            double neighbor_edge_cost=ComputeG(current_node_ptr,neighbor_node_ptr);
            // ROS_INFO("computeG");
            double current_h=ComputeH(neighbor_node_ptr,goal_node_ptr)*hAParam.tie_breaker_;
            // ROS_INFO("computeH");
            const Vec3i& index=neighbor_node_ptr->grid_index_;
            //不在open和close中
            if(env.state_node_map[index.x()][index.y()][index.z()]==nullptr){
                neighbor_node_ptr->parent_node_=current_node_ptr;
                neighbor_node_ptr->g_cost_=current_node_ptr->g_cost_+neighbor_edge_cost;
                neighbor_node_ptr->f_cost=neighbor_node_ptr->g_cost_+current_h;
                neighbor_node_ptr->node_status_=StateNode::IN_OPENSET;
                neighbor_node_ptr->stateIt=openset_.insert(std::make_pair(neighbor_node_ptr->f_cost,neighbor_node_ptr));
                env.state_node_map[index.x()][index.y()][index.z()]=neighbor_node_ptr;
                // ROS_INFO("add_node_in nullptr");
                continue;
            }
            else if(env.state_node_map[index.x()][index.y()][index.z()]->node_status_==StateNode::IN_OPENSET){
                double new_g_cost=current_node_ptr->g_cost_+neighbor_edge_cost;
                if(env.state_node_map[index.x()][index.y()][index.z()]->g_cost_>new_g_cost){
                    neighbor_node_ptr->g_cost_ = new_g_cost;
                    neighbor_node_ptr->f_cost=new_g_cost+current_h;
                    neighbor_node_ptr->node_status_=StateNode::IN_OPENSET;
                    neighbor_node_ptr->parent_node_=current_node_ptr;
                    openset_.erase(env.state_node_map[index.x()][index.y()][index.z()]->stateIt);
                    // ROS_INFO("delete_node");
                    delete env.state_node_map[index.x()][index.y()][index.z()];
                    // ROS_INFO("delete_ok");
                    neighbor_node_ptr->stateIt=openset_.insert(std::make_pair(neighbor_node_ptr->f_cost,neighbor_node_ptr));
                    env.state_node_map[index.x()][index.y()][index.z()]=neighbor_node_ptr;
                }
                else{
                    delete neighbor_node_ptr;
                }
                continue;
            }
            else if(env.state_node_map[index.x()][index.y()][index.z()]->node_status_==StateNode::IN_CLOSESET){
                delete neighbor_node_ptr;
                continue;
            }
        }
    }
    return false;
}

double HybridAStar::ComputeH(const StateNode::ptr& current_node_ptr,
                          const StateNode::ptr& terminal_node_ptr){
    double h;
    //启发式函数不需要用A* 我的地图没有障碍物
    h=(current_node_ptr->state_.head(2)-terminal_node_ptr->state_.head(2))
        .lpNorm<1>();
    
    if(h<3.0*hAParam.shot_distance_){
        h=rs_path_ptr_->Distance(
            current_node_ptr->state_.x(),current_node_ptr->state_.y(),
            current_node_ptr->state_.z(),terminal_node_ptr->state_.x(),
            terminal_node_ptr->state_.y(),terminal_node_ptr->state_.z()
        );
    }
    return h;
}

double HybridAStar::ComputeG(const StateNode::ptr &current_node_ptr,
                            const StateNode::ptr &neighbor_node_ptr) const{
    double g;
    //前进后退的cost不同
  if (neighbor_node_ptr->direction_ == StateNode::FORWARD) {
    if (neighbor_node_ptr->steering_grade_ !=
        current_node_ptr->steering_grade_) {
      if (neighbor_node_ptr->steering_grade_ == 0) {
        g = cfg.segment_length_ * hAParam.steering_change_penalty_;
      } else {
        g = cfg.segment_length_ * hAParam.steering_change_penalty_ * hAParam.steering_penalty_;
      }
    } else {
      if (neighbor_node_ptr->steering_grade_ == 0) {
        g = cfg.segment_length_;
      } else {
        g = cfg.segment_length_ * hAParam.steering_penalty_;
      }
    }
  } else {
    if (neighbor_node_ptr->steering_grade_ !=
        current_node_ptr->steering_grade_) {
      if (neighbor_node_ptr->steering_grade_ == 0) {
        g = cfg.segment_length_ * hAParam.steering_change_penalty_ * hAParam.reversing_penalty_;
      } else {
        g = cfg.segment_length_ * hAParam.steering_change_penalty_ * hAParam.steering_penalty_ *
            hAParam.reversing_penalty_;
      }
    } else {
      if (neighbor_node_ptr->steering_grade_ == 0) {
        g = cfg.segment_length_ * hAParam.reversing_penalty_;
      } else {
        g = cfg.segment_length_ * hAParam.steering_penalty_ * hAParam.reversing_penalty_;
      }
    }
  }

  return g;
}

bool HybridAStar::AnalyticExpansions(const StateNode::ptr& current_node_ptr,
                                    const StateNode::ptr& goal_node_ptr,double& length){
    
    VectorVec4d rs_path_poses = rs_path_ptr_->GetRSPath(
      current_node_ptr->state_, goal_node_ptr->state_, cfg.move_step_size_, length);
    
    //检查rs_pose是否合理
    for(const auto& pose:rs_path_poses){
        if(env.outBoundary(pose.head(2))||
            !env.checkCollision(pose.x(),pose.y(),pose.z())){
            return false;
        }            
    }
    goal_node_ptr->intermediate_states_=rs_path_poses;
    goal_node_ptr->parent_node_=current_node_ptr;

    auto begin=goal_node_ptr->intermediate_states_.begin();
    goal_node_ptr->intermediate_states_.erase(begin);

    return true;
}

void HybridAStar::GetNeighborNodes(const StateNode::ptr &curr_node_ptr,
                        std::vector<StateNode::ptr> &neighbor_nodes){
    neighbor_nodes.clear();

    for(int i=-cfg.steering_discrete_num_;i<=cfg.steering_discrete_num_;i++){
        VectorVec4d intermediate_states;
        bool has_obstacle=false;

        double x=curr_node_ptr->state_.x();
        double y=curr_node_ptr->state_.y();
        double theta=curr_node_ptr->state_.z();

        const double phi=i*cfg.steering_radian_step_size_;

        //forward 前向扩展
        for(int j=1;j<=cfg.segment_length_discrete_num_;j++){
            DynamicModel(cfg.move_step_size_, phi, x, y, theta);
            intermediate_states.emplace_back(Vec4d(x, y, theta,1.0));

            if (!env.checkCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }
        Vec3i grid_index = env.state2index(Vec3d(intermediate_states.back().x(),
                                                  intermediate_states.back().y(),
                                                  intermediate_states.back().z()));
        if (!env.outBoundary(intermediate_states.back().head(2)) && !has_obstacle) {
            auto neighbor_forward_node_ptr = new StateNode(grid_index);
            neighbor_forward_node_ptr->intermediate_states_ = intermediate_states;
            neighbor_forward_node_ptr->state_ = Vec3d(intermediate_states.back().x(),
                                                  intermediate_states.back().y(),
                                                  intermediate_states.back().z());
            neighbor_forward_node_ptr->steering_grade_ = i;
            neighbor_forward_node_ptr->direction_ = StateNode::FORWARD;
            neighbor_nodes.push_back(neighbor_forward_node_ptr);
        }

        //backward
        has_obstacle=false;
        intermediate_states.clear();
        x = curr_node_ptr->state_.x();
        y = curr_node_ptr->state_.y();
        theta = curr_node_ptr->state_.z();
        for (int j = 1; j <= cfg.segment_length_discrete_num_; j++) {
            DynamicModel(-cfg.move_step_size_, phi, x, y, theta);
            intermediate_states.emplace_back(Vec4d(x, y, theta,-1.0));

            if (!env.checkCollision(x, y, theta)) {
                has_obstacle = true;
                break;
            }
        }
        if (!env.outBoundary(intermediate_states.back().head(2)) && !has_obstacle) {
            grid_index = env.state2index(Vec3d(intermediate_states.back().x(),
                                                  intermediate_states.back().y(),
                                                  intermediate_states.back().z()));
            auto neighbor_backward_node_ptr = new StateNode(grid_index);
            neighbor_backward_node_ptr->intermediate_states_ = intermediate_states;
            neighbor_backward_node_ptr->state_ = Vec3d(intermediate_states.back().x(),
                                                  intermediate_states.back().y(),
                                                  intermediate_states.back().z());
            neighbor_backward_node_ptr->steering_grade_ = i;
            neighbor_backward_node_ptr->direction_ = StateNode::BACKWARD;
            neighbor_nodes.push_back(neighbor_backward_node_ptr);
        }
    }
}

void HybridAStar::DynamicModel(const double &step_size, const double &phi,
                               double &x, double &y, double &theta) {
  x = x + step_size * std::cos(theta);
  y = y + step_size * std::sin(theta);
  theta = mod2pi(theta + step_size / cfg.wheel_base_ * std::tan(phi));
}

double HybridAStar::mod2pi(const double &x){
    double v= fmod(x,2*M_PI);
    if (v < -M_PI) {
        v += 2.0 * M_PI;
    } else if (v > M_PI) {
        v -= 2.0 * M_PI;
    }

    return v;
}

VectorVec4d HybridAStar::getPath() const{
    VectorVec4d path;
    std::vector<StateNode::ptr> temp_nodes;
    StateNode::ptr state_grid_node_ptr = terminal_node_ptr_;
    while(state_grid_node_ptr!=nullptr){
        temp_nodes.emplace_back(state_grid_node_ptr);
        state_grid_node_ptr=state_grid_node_ptr->parent_node_;
    }
    std::reverse(temp_nodes.begin(),temp_nodes.end());
    for(const auto& node:temp_nodes){
        path.insert(path.end(),node->intermediate_states_.begin(),
                node->intermediate_states_.end());
    }
    return path;
}

