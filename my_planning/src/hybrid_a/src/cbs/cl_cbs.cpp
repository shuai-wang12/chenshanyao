#include "cbs/cl_cbs.h"
#include "single_planner.h"
#include "ros/ros.h"

CL_CBS::CL_CBS(environment &_env, carconfig &_cfg, HybridAStarParam &_hAParam, TrajPlannerConfig &config)
    : env(_env), cfg(_cfg), hAParam(_hAParam), config_(config)
{
}

bool CL_CBS::search(std::vector<Vec3d> &initialStates,
                    std::vector<Vec3d> &goalStates,
                    std::vector<PlanResult<Vec4d, int, double>> &solution)
{
    // HighLevelNode start;
    // start.solution.resize(initialStates.size());
    // start.constraints.resize(initialStates.size());
    // start.cost = 0;
    // start.id = 0;
    // for (int i = 0; i < initialStates.size(); i++)
    // {
    //     single_planner low_planner(env, cfg, hAParam, config_);
    //     if (!low_planner.plan(initialStates[i], goalStates[i], start.solution[i]))
    //     {
    //         return false;
    //     }
    //     start.cost += start.solution[i].cost;
    // }
    // typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
    //                                  boost::heap::mutable_<true>>
    //     open;

    // auto handle = open.push(start);
    // (*handle).handle = handle;
    // int iter = 0;
    // solution.clear();
    // int id = 1;
    // while (!open.empty())
    // {
    //     iter++;
    //     if (iter > 10000)
    //     {
    //         open.clear();
    //         ROS_INFO("error_IterOut");
    //         return false;
    //     }
    //     HighLevelNode P = open.top();
    //     open.pop();
    //     Conflict conflict;
    //     if (!getFirstConflict(P.solution, conflict))
    //     {
    //         solution = P.solution;
    //         open.clear();
    //         return true;
    //     }

    //     HighLevelNode newNode = P;
    //     int agentNeedAddIdx=std::max(conflict.agent1,conflict.agent2);
    //     newNode.solution[agentNeedAddIdx].start_time++;
    //     newNode.id=id;
    //     newNode.cost = 0;
    //     auto handle = open.push(newNode);
    //       (*handle).handle = handle;
    //     // if(conflict)
    //     // std::map<size_t, CBS::Constraints> constraints;
    //     // createConstraintsFromConflict(conflict, constraints);
    //     // for (const auto &c : constraints)
    //     // {
    //     //     size_t i = c.first;
    //     //     HighLevelNode newNode = P;
    //     //     newNode.id = id;
            
    //     //     // newNode.constraints[i].add(c.second);
    //     //     // newNode.cost -= newNode.solution[i].cost;
    //     //     // todo添加约束后，如何挪动时间
    //     //     ++id;
    //     // }
    //     ++id;
    // }
    return false;
}

void CL_CBS::createConstraintsFromConflict(
    const Conflict &conflict, std::map<size_t, CBS::Constraints> &constraints)
{
    CBS::Constraints c1;
    c1.constraints.emplace(
        Constraint(conflict.time, conflict.s2, conflict.agent2));
    constraints[conflict.agent1] = c1;
    CBS::Constraints c2;
    c2.constraints.emplace(
        Constraint(conflict.time, conflict.s1, conflict.agent1));
    constraints[conflict.agent2] = c2;
    
}

bool CL_CBS::getFirstConflict(const std::vector<PlanResult<Vec4d, int, double>> &solution, Conflict &result)
{
    int max_t = 0;
    for (const auto &sol : solution)
    {
        max_t = std::max<int>(max_t, sol.states.size() - 1 + sol.start_time);
    }
    for (int t = 0; t < max_t; ++t)
    {
        for (size_t i = 0; i < solution.size(); i++)
        {
            Vec4d state1 = getState(i, solution, t);
            for (size_t j = i + 1; j < solution.size(); j++)
            {
                Vec4d state2 = getState(j, solution, t);
                if (agentCollision(state1, state2, cfg.length_ - cfg.rear_, cfg.width_))
                {
                    result.time = t;
                    result.agent1 = i;
                    result.agent2 = j;
                    result.s1 = state1;
                    result.s2 = state2;
                    return true;
                }
            }
        }
    }
    return false;
}

Vec4d CL_CBS::getState(size_t agentIdx,
                       const std::vector<PlanResult<Vec4d, int, double>> &solution,
                       size_t t)
{
    assert(agentIdx < solution.size());
    t = t - solution[agentIdx].start_time;
    if (t < 0)
    {
        return solution[agentIdx].states[0];
    }
    if (t < solution[agentIdx].states.size())
    {
        return solution[agentIdx].states[t];
    }
    assert(!solution[agentIdx].states.empty());
    return solution[agentIdx].states.back();
}

bool agentCollision(const Vec4d &state, const Vec4d &other, double lf, double width)
{
    if (pow(state.x() - other.x(), 2) + pow(state.y() - other.y(), 2) <
        pow(2 * (lf), 2) + pow(width, 2))
        return true;
    return false;
}