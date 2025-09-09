#pragma once

#include <vector>
#include <math.h>
#include "type.h"

#include "voronoi/dynamicvoronoi.h"
#include "voronoi/voronoiedge.h"
#include "state_node.h"
typedef struct middlePoint{

    int steering;                       /*左右转还是前进*/
    double R,turningtheta;              /*泊车半径*/
    double cost;
    Vec3d pos;                   /*位置信息*/

    middlePoint(/* args */){};
    ~middlePoint(){};
    bool operator<(const middlePoint& other) const {
      // Sort order
      // 1. lowest fScore
      // 2. highest gScore
      return cost > other.cost;
    }
}middlePoint;

// struct compare_middleParkPoint
// {
//     bool operator()(const middleParkPoint& n1, const middleParkPoint& n2) const
//     {
//         return n1.cost > n2.cost;
//     }
// };

/**************************************************************
*  @brief : 计算中间点 以后再简化
*  @param : 输入泊车终点，输出采样的倒车点
*  @return : void
*  @note : 
***************************************************************/
void calMiddlePoint(std::vector<middlePoint>& mp,const Vec3d& goal,const double radius);
TypeVectorVecd<4> getMiddlePath(const middlePoint& p,const Vec3d& goal_state,
                                const double step_size);
double normalizeHeadingRad(double t);
// double calVoronoiField(DynamicVoronoi& diagram,voronoiedge& edge);