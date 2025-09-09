#include "cbs/cl_cbs.h"
#include "single_planner.h"
#include "ros/ros.h"
#include "config.h"
#include "multi_opti.h"
#include "csv/csv.h"
#include <boost/algorithm/string/replace.hpp>
#include "visual_rviz/plot.h"
#include "timer.h"

//这部分的内容大部分都是cl-cbs论文里的东西，就是改了cbs产生不同节点那部分

CL_CBS::CL_CBS(environment &_env,environment &_env2, carconfig &_cfg, HybridAStarParam &_hAParam, TrajPlannerConfig &config)
    : env(_env),env2(_env2), cfg(_cfg), hAParam(_hAParam), config_(config)
{
}

bool CL_CBS::search(std::vector<Vec3d> &initialStates,
                    std::vector<Vec3d> &goalStates,
                    std::vector<PlanResult<Vec4d, int, double>> &solution)
{
    HighLevelNode start;
    start.solution.resize(initialStates.size());
    start.constraints.resize(initialStates.size());
    start.cost = 0;
    start.id = 0;
    for (int i = 0; i < initialStates.size(); i++)
    {
        single_planner low_planner(env,env2, cfg, hAParam, config_,i);
        if (!low_planner.plan(initialStates[i], goalStates[i], start.solution[i],false))
        {
            return false;
        }
        start.cost += start.solution[i].cost;
    }
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;
    int iter = 0;
    solution.clear();
    int id = 1;
    while (!open.empty())
    {
        iter++;
        if (iter > 10000)
        {
            open.clear();
            ROS_INFO("error_IterOut");
            return false;
        }
        HighLevelNode P = open.top();
        open.pop();
        Conflict conflict;
        if (!getFirstConflict(P.solution, conflict))
        {
            solution = P.solution;
            open.clear();
            return true;
        }

        HighLevelNode newNode = P;
        int agentNeedAddIdx=std::max(conflict.agent1,conflict.agent2);
        newNode.solution[agentNeedAddIdx].start_time++;
        newNode.id=id;
        newNode.cost = 0;
        auto handle = open.push(newNode);
          (*handle).handle = handle;

        ++id;
    }
    return false;
}

bool CL_CBS::searchProgress(std::vector<Vec3d> &initialStates,
                std::vector<Vec3d> &parkStates,
                std::vector<Vec3d> &goalStates,
                std::vector<PlanResult<Vec4d, int, double>> &solution){
    Timer search_used_time;
    HighLevelNode start;
    start.solution.resize(6);
    start.constraints.resize(6);
    start.cost = 0;
    start.id = 0;

    //输入单车序列文件
    std::string inputFile(__FILE__);
    boost::replace_all(inputFile, "src/cbs/cl_cbs.cpp", "pathes/");
    std::string tempfile = inputFile+"r11_park2.csv";
    io::CSVReader<3> in1(tempfile);
    in1.read_header(io::ignore_extra_column,"x","y","theta");
    double x,y,theta;
    while(in1.read_row(x,y,theta)){
        start.solution[0].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[0].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[0].start_time=0;

    tempfile = inputFile+"r12_park4.csv";
    io::CSVReader<3> in2(tempfile);
    in2.read_header(io::ignore_extra_column,"x","y","theta");
    while(in2.read_row(x,y,theta)){
        start.solution[1].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[1].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[1].start_time=0;

    tempfile = inputFile+"park9.csv";
    io::CSVReader<3> in3(tempfile);
    in3.read_header(io::ignore_extra_column,"x","y","theta");
    while(in3.read_row(x,y,theta)){
        start.solution[2].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[2].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[2].start_time=0;

    tempfile = inputFile+"newout1.csv";
    io::CSVReader<3> in4(tempfile);
    in4.read_header(io::ignore_extra_column,"x","y","theta");
    while(in4.read_row(x,y,theta)){
        start.solution[3].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[3].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[3].start_time=0;

    tempfile = inputFile+"out7.csv";
    io::CSVReader<3> in5(tempfile);
    in5.read_header(io::ignore_extra_column,"x","y","theta");
    while(in5.read_row(x,y,theta)){
        start.solution[4].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[4].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[4].start_time=0;

    tempfile = inputFile+"out10.csv";
    io::CSVReader<3> in6(tempfile);
    in6.read_header(io::ignore_extra_column,"x","y","theta");
    while(in6.read_row(x,y,theta)){
        start.solution[5].states.push_back(Vec4d(x,y,theta,0));
        visual_rviz::PlotTraj(start.solution[5].states, 0.1, visual_rviz::Color::Red, -1, "refined trajectory");
    }
    start.solution[5].start_time=0;
    visual_rviz::TriggerRemain();
    // for (int i = 0; i < initialStates.size(); i++)
    // {
        // single_planner low_planner(env,env2, cfg, hAParam, config_,i);
        // if (!low_planner.plan2(initialStates[i],parkStates[i], goalStates[i], start.solution[i]))
        // {
        //     return false;
        // }
    //     start.solution[i].start_time=i*2;
    //     start.cost += start.solution[i].cost;
    // }

    toCircle(start.solution);//转换到圆心，因为把车子当做一个圆做检测
    //这个和cl-cbs算法的框架一致，看人家的论文好一些
    typename boost::heap::d_ary_heap<HighLevelNode, boost::heap::arity<2>,
                                     boost::heap::mutable_<true>>
        open;

    auto handle = open.push(start);
    (*handle).handle = handle;
    int iter = 0;
    solution.clear();
    int id = 1;
    while (!open.empty())
    {
        iter++;
        if (iter > 100000000)
        {
            open.clear();
            ROS_INFO("error_IterOut");
            return false;
        }
        HighLevelNode P = open.top();
        open.pop();
        Conflict conflict;
        if (!getFirstConflict(P.solution, conflict))
        {
            solution = P.solution;
            circleBack(solution);
            open.clear();
            ROS_INFO(
                "\033[1;32m --> Time in Hybrid A star is %f ms\n",
                search_used_time.End());
            return true;
        }

        HighLevelNode newNode = P;

        //这里是修改时间的部分
        /* cost都置为0，没有重新规划 */
        int agentHighIdx=std::max(conflict.agent1,conflict.agent2);
        int agentLowIdx=std::min(conflict.agent1,conflict.agent2);
        // if(newNode.solution[agentHighIdx].start_time<newNode.solution[agentLowIdx].start_time)
        // {
        //     newNode.solution[agentHighIdx].start_time=newNode.solution[agentLowIdx].start_time+20;
        // }
        int dt = getStartDeltaTime(newNode.solution[agentLowIdx],newNode.solution[agentHighIdx]);
        newNode.solution[agentHighIdx].start_time=dt+newNode.solution[agentLowIdx].start_time;
        newNode.id=id;
        newNode.cost = 0;
        auto handle = open.push(newNode);
          (*handle).handle = handle;
        // std::cout<<agentNeedAddIdx<<" "<<newNode.solution[agentNeedAddIdx].start_time<<std::endl;
        ++id;
        ROS_INFO("CBS:%d",iter);
    }
    
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
            if(solution[i].states.size()+solution[i].start_time<=t||t<solution[i].start_time){
                continue;
            }
            Vec4d state1 = getState(i, solution, t);
            for (size_t j = i + 1; j < solution.size(); j++)
            {
                if(solution[j].states.size()+solution[j].start_time<=t||t<solution[j].start_time){
                    continue;
                }
                Vec4d state2 = getState(j, solution, t);
                if (agentCollision(state1, state2, cfg.length_ , cfg.width_))
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
    assert(!solution[agentIdx].states.empty());
    t = t - solution[agentIdx].start_time;
    if (t < 0)
    {
        return solution[agentIdx].states[0];
    }
    if (t < solution[agentIdx].states.size())
    {
        return solution[agentIdx].states[t];
    }
    return solution[agentIdx].states.back();
}

void toCircle(std::vector<PlanResult<Vec4d, int, double>>& solution){
    for (int i = 0; i < solution.size(); i++){
        for (int j = 0; j < solution[i].states.size(); j++){
            solution[i].states[j].x()+=1.05*std::cos(solution[i].states[j].z());
            solution[i].states[j].y()+=1.05*std::sin(solution[i].states[j].z());
        }
    }
}

void circleBack(std::vector<PlanResult<Vec4d, int, double>>& solution){
    for (int i = 0; i < solution.size(); i++){
        for (int j = 0; j < solution[i].states.size(); j++){
            solution[i].states[j].x()-=1.05*std::cos(solution[i].states[j].z());
            solution[i].states[j].y()-=1.05*std::sin(solution[i].states[j].z());
        }
    }
}


bool agentCollision(const Vec4d &state, const Vec4d &other, double lf, double width)
{
    if ((pow(state.x() - other.x(), 2) + pow(state.y() - other.y(), 2)) <
        (pow(lf, 2) + pow(width, 2)))
        return true;
    return false;
}