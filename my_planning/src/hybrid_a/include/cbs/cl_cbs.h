#pragma once

#define USE_FIBONACCI_HEAP
#include <boost/heap/d_ary_heap.hpp>
#include <boost/heap/fibonacci_heap.hpp>
// #include <boost/program_options.hpp>
#include <boost/functional/hash.hpp>
#include <unordered_set>

#include "environments.h"
#include "cbs/planresult.hpp"
#include "hybrid_a/hybrid_a.h"
#include "traj/traj_planner_config.h"


bool agentCollision(const Vec4d& state,const Vec4d& other,double lf,double width);

// //Constraint是cbs特有的
struct Conflict {
  int time;
  size_t agent1;
  size_t agent2;

  Vec4d s1;
  Vec4d s2;

};

struct Constraint{
    Constraint(int time,Vec4d s,size_t agentid)
        : time(time), s(s), agentid(agentid) {}
    Constraint() = default;
    int time;
    Vec4d s;
    size_t agentid;
    
    bool operator<(const Constraint& other) const {
        return std::tie(time, s.x(), s.y(), s.z(), agentid) <
            std::tie(other.time, other.s.x(), other.s.y(), other.s.z(),
                        other.agentid);
    }

    bool operator==(const Constraint& other) const {
        return std::tie(time, s.x(), s.y(), s.z(), agentid) ==
            std::tie(other.time, other.s.x(), other.s.y(), other.s.z(),
                        other.agentid);
    }

    bool satisfyConstraint(const Vec4d& state,const int time,const carconfig& cfg) const {
        if (time < this->time ||
            time > this->time + 2)//需要设置等待时间
            return true;
        return !agentCollision(this->s,state,cfg.length_-cfg.rear_,cfg.width_);
    }

};

namespace std {
template <>
struct hash<Constraint> {
  size_t operator()(const Constraint& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, s.time);
    boost::hash_combine(seed, s.s.x());
    boost::hash_combine(seed, s.s.y());
    boost::hash_combine(seed, s.s.z());
    boost::hash_combine(seed, s.agentid);
    return seed;
  }
};
} 

namespace CBS{
struct Constraints {
  std::unordered_set<Constraint> constraints;

  void add(const Constraints& other) {
    constraints.insert(other.constraints.begin(), other.constraints.end());
  }

  bool overlap(const Constraints& other) {
    for (const auto& c : constraints) {
      if (other.constraints.count(c) > 0) return true;
    }
    return false;
  }
};
}

class CL_CBS
{
public:
    CL_CBS(environment &_env, carconfig &_cfg, HybridAStarParam &_hAParam, TrajPlannerConfig &config);
    CL_CBS() = default;
    ~CL_CBS() = default;
    // 第二项是时间
    bool search(std::vector<Vec3d> &initialStates,
                std::vector<Vec3d> &goalStates,
                std::vector<PlanResult<Vec4d, int, double>> &solution);
private:
    void createConstraintsFromConflict(const Conflict &conflict, std::map<size_t, CBS::Constraints> &constraints);
    bool getFirstConflict(const std::vector<PlanResult<Vec4d,int,double>>&solution,Conflict &result);
private:
    environment &env;
    carconfig &cfg;
    HybridAStarParam &hAParam;
    TrajPlannerConfig &config_; 

private:

    struct HighLevelNode
    {
        std::vector<PlanResult<Vec4d, int, double>> solution;
        std::vector<CBS::Constraints> constraints;
        double cost;
        int id;

        typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>::handle_type
            handle;

        bool operator<(const HighLevelNode& n) const {
        // if (cost != n.cost)
        return cost > n.cost;
        // return id > n.id;
    }
    };
    
    Vec4d getState(size_t agentIdx,
            const std::vector<PlanResult<Vec4d, int, double>> &solution,
            size_t t);
    //       HighLevelNode start;
    //       start.solution.resize(initialStates.size());
    //       start.constraints.resize(initialStates.size());    //i个车的约束，每个约束都是一系列约束，即i与其他车的约束，是vector
    //       start.cost = 0;
    //       start.id =0;

    //       for(int i=0;i<initialStates.size();i++){

    //       }
    //    }

    //  private:
    //   struct HighLevelNode {
    //     std::vector<PlanResult<State, Action, Cost>> solution;
    //     std::vector<Constraints> constraints;

    //     Cost cost;
    //     int id;

    //     //用于记录节点在openset中的位置
    //     typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    //                                      boost::heap::mutable_<true>>::handle_type
    //         handle;
    //     //最小堆
    //     bool operator<(const HighLevelNode& n) const { return cost > n.cost; }
    //   };
    //   struct LowLevelEnvironment{
    //    LowLevelEnvironment(Environment& env,size_t agentIdx, const Constraints& constraints):m_env(env){

    //    }
    //    Enviornment& m_env;
    //   };
    
};
