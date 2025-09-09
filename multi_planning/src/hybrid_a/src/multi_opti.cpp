#include "multi_opti.h"
#include "ros/ros.h"
void multi_opti(std::vector<PlanResult<Vec4d, int, double>>& solution){
    if(solution.size()<2)
        return ;
    for (size_t i = 1; i < solution.size(); i++)
    {
        int dt=getStartDeltaTime(solution[i-1],solution[i]);
        solution[i].start_time=dt+solution[i-1].start_time;
    }
}

//这个函数就是计算最合适的时空轨迹平移的时间
int getStartDeltaTime(PlanResult<Vec4d, int, double>& solution1,PlanResult<Vec4d, int, double>& solution2){
    int delta_time = solution2.start_time-solution1.start_time;
    std::vector<double> delta_dis(solution1.states.size(),0);
    std::vector<double> delta_d_dis(solution1.states.size(),0);
    std::vector<double> velocity_x(solution1.states.size(),1);
    std::vector<double> velocity_y(solution1.states.size(),1);
    //根据我自己论文里第四章的  局部冲突优化做的 就是求变化率，求距离，根据距离求最好的平移时间
    for (size_t i = 0; i < solution1.states.size(); i++)
    {
        if (i<delta_time||i-delta_time>=solution2.states.size())
        {
            delta_dis[i]=std::numeric_limits<double>::infinity();
        }
        else{
            delta_dis[i]=(solution1.states[i].x()-solution2.states[i-delta_time].x())*(solution1.states[i].x()-solution2.states[i-delta_time].x())+
                (solution1.states[i].y()-solution2.states[i-delta_time].y())*(solution1.states[i].y()-solution2.states[i-delta_time].y());
        }
        if(i!=0&&i<solution2.states.size()){
            velocity_x[i]=solution2.states[i].x()-solution2.states[i-1].x();
            velocity_y[i]=solution2.states[i].y()-solution2.states[i-1].y();
        }
    }
    double min_dis = std::numeric_limits<double>::infinity();
    ROS_INFO("dt:%d",delta_time);
    int start_time = delta_time;
    int iter = 0;
    while(iter<100){
        // iter++;
        start_time++;
        min_dis = std::numeric_limits<double>::infinity();
        for (size_t i = 0; i < solution1.states.size(); i++)
        {
            if (i<start_time||i-start_time>=solution2.states.size())
            {
                delta_dis[i]=std::numeric_limits<double>::infinity();
            }
            else{
                delta_d_dis[i]=2*(solution1.states[i].x()-solution2.states[i-start_time].x())*velocity_x[i-start_time]+
                    2*(solution1.states[i].y()-solution2.states[i-start_time].y())*velocity_y[i-start_time];
                if(std::isinf(delta_dis[i])){
                    delta_dis[i]=(solution1.states[i].x()-solution2.states[i-start_time].x())*(solution1.states[i].x()-solution2.states[i-start_time].x())+
                    (solution1.states[i].y()-solution2.states[i-start_time].y())*(solution1.states[i].y()-solution2.states[i-start_time].y());
                }
                else{
                    delta_dis[i]=delta_dis[i]+delta_d_dis[i];
                }
                if (min_dis>delta_dis[i])
                {
                    ROS_INFO("dd:%f,%d",delta_dis[i],start_time);
                    min_dis=delta_dis[i];
                }                
            }
        }
        //26.09是圆心之间的距离的  平方？ QAQ
        if(min_dis>26.09){
            return start_time;
        }
    }
    return start_time;
}