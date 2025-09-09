#include "hybrid_a/middlePark.h"
#include "hybrid_a/state_node.h"
#include "hybrid_a/hybrid_a.h"
#include <iostream>
/**************************************************************
 *  @brief : 计算中间点 以后再简化
 *  @param : 输入泊车终点，输出采样的倒车点
 *  @return : void
 *  @note :
 ***************************************************************/
void calMiddlePoint(std::vector<middlePoint> &mp, const Vec3d &goal, const double radius)
{
    middlePoint p;
    double x, y, theta;
    const double dyaw = M_PI / 18;
    x = radius;
    y = 0;
    theta = 0;
    p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
    p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
    p.pos[2] = goal[2] + theta;
    p.R = radius;
    p.turningtheta = 0;
    p.steering = 0;
    mp.push_back(p);

    x = radius * M_PI_2;
    y = 0;
    theta = 0;
    p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
    p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
    p.pos[2] = goal[2] + theta;
    p.R = radius * M_PI_2;
    p.turningtheta = 0;
    p.steering = 0;
    mp.push_back(p);

    size_t num = 10;
    for (size_t i = 1; i < num; i++)
    {
        theta = dyaw * i;

        x = radius;
        p.R = radius / sin(theta);
        y = (i == num - 1) ? p.R : p.R - x / tan(theta);
        p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
        p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
        p.pos[2] = goal[2] + theta;
        p.turningtheta = theta;
        p.steering = 1;
        mp.push_back(p);

        x = radius;
        p.R = radius / sin(theta);
        y = (i == num - 1) ? -p.R : -p.R + x / tan(theta);
        p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
        p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
        p.pos[2] = goal[2] - theta;
        p.turningtheta = theta;
        p.steering = -1;
        mp.push_back(p);

        p.R = radius * M_PI_2 / theta;
        x = p.R * sin(theta);
        y = p.R * (1 - cos(theta));
        p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
        p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
        p.pos[2] = goal[2] + theta;
        p.turningtheta = theta;
        p.steering = 1;
        mp.push_back(p);

        y = -y;
        p.pos[0] = goal[0] + x * cos(goal[2]) - y * sin(goal[2]);
        p.pos[1] = goal[1] + x * sin(goal[2]) + y * cos(goal[2]);
        p.pos[2] = goal[2] - theta;
        p.turningtheta = theta;
        p.steering = -1;
        mp.push_back(p);
    }
}

TypeVectorVecd<4> getMiddlePath(const middlePoint& p,const Vec3d& goal_state,
                                const double step_size){
  double length=p.R;
  const auto interpolation_number=static_cast<unsigned int>(length/step_size);

  double xSucc,ySucc,yawSucc,dx,dy,dyaw,ratio;
  TypeVectorVecd<4> path_poses;
  path_poses.emplace_back(Vec4d(p.pos.x(),p.pos.y(),p.pos.z(),1));
  if(p.steering==0){
    for(size_t i=0;i<(size_t)(interpolation_number);i++){
      Vec4d s=path_poses.back();
      xSucc=s.x()-step_size*cos(s.z());
      ySucc=s.y()-step_size*sin(s.z());
      yawSucc=normalizeHeadingRad(s.z());
      path_poses.emplace_back(Vec4d(xSucc,ySucc,yawSucc,-1));
    }
    ratio=(length-static_cast<int>(length/step_size)*step_size)/step_size;
    dyaw=0;
    dx=ratio*-step_size;
    dy=0;
  }
  else{
    double newdYaw=step_size/length;
    std::cout<<p.turningtheta<<std::endl;
    std::cout<<newdYaw<<std::endl;
    for(size_t i=0;i<(size_t)(p.turningtheta/newdYaw);i++){
      Vec4d s=path_poses.back();
      xSucc=s.x()-length*sin(newdYaw)*cos(s.z())-
          p.steering*length*(1-cos(newdYaw))*sin(s.z());
      ySucc=s.y()-length*sin(newdYaw)*sin(s.z())+
          p.steering*length*(1-cos(newdYaw))*cos(s.z());
      yawSucc=normalizeHeadingRad(s.z()-p.steering*newdYaw);
      path_poses.emplace_back(
        Vec4d(xSucc,ySucc,yawSucc,-1)
      );
    }
    ratio=(p.turningtheta-static_cast<int>(p.turningtheta/newdYaw)*newdYaw)/newdYaw;
    dyaw=ratio*p.steering*newdYaw;
    dx=length*sin(dyaw);
    dy=-length*(1-cos(dyaw));
    if(p.steering>0) dx=-dx,dy=-dy; 
  }
  Vec4d s=path_poses.back();
  xSucc=s.x()+dx*cos(s.z())-dy*sin(s.z());
  ySucc=s.y()+dx*sin(s.z())+dy*cos(s.z());
  yawSucc=normalizeHeadingRad(s.z()-dyaw);
  path_poses.emplace_back(Vec4d(xSucc,ySucc,yawSucc,-1));
  return path_poses;
}

double normalizeHeadingRad(double t)
{
    if (t < 0)
    {
        t = t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
        return 2.f * M_PI + t;
    }
    return t - 2.f * M_PI * static_cast<int>(t / (2.f * M_PI));
}

double calVoronoiField(int x, int y, DynamicVoronoi &diagram, voronoiedge &edge)
{
    double alpha = 0.1;
    double d_max = 30;
    double d_obs = diagram.getDistance(x, y);
    double d_edge = edge.getVEdgeDist(x, y);
    if (d_obs > d_max)
        return 0;
    double temp = (d_obs - d_max / d_max);
    double result = (alpha / (alpha + d_obs)) * (d_edge / (d_obs + d_edge)) * temp * temp;
    return result;
}
