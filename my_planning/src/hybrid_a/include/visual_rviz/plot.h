#pragma once

#include <mutex>
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "visual_rviz/color.h"
#include "type.h"
#include "traj/discretized_traj.h"
#include "math/polygon2d.h"
namespace visual_rviz
{
    using math::Polygon2d;
    void Init(ros::NodeHandle &node, const std::string &frame, const std::string &topic);

    void Plot(const std::vector<double> &xs, const std::vector<double> &ys,
              double width, Color color, int id, const std::string &ns);

    void PlotLine(const std::vector<double> &xs, const std::vector<double> &ys,
                  double width, Color color, int id, const std::string &ns);
    void PlotLine(const std::vector<int> &xs, const std::vector<int> &ys,
                  double width, Color color, int id, const std::string &ns);
    void PlotPolygon(const std::vector<double> &xs, const std::vector<double> &ys, double width, Color color, int id, const std::string &ns);

    void PlotPolygon(const Polygon2d &polygon, double width, Color color, int id,
                 const std::string &ns);
    void Trigger();

    std::string& getFrame();
    std::mutex& getMutex();
    visualization_msgs::MarkerArray& getArr();

    void Clear();

    void PlotTraj(const std::vector<Vec4d> &ps,
            double width, Color color, int id, const std::string &ns);
    void PlotTraj(const TypeVectorVecd<4> &ps,
            double width, Color color, int id, const std::string &ns);
    void PlotTraj(const std::vector<TrajectoryPoint> &ps,
            double width, Color color, int id, const std::string &ns);
} // namespace visual_rviz
