#include "visual_rviz/plot.h"

namespace visual_rviz
{
    // 无命名空间说明只在本文件使用
    namespace
    {
        std::string frame_ = "world";
        std::mutex mutex_;

        ros::Publisher publisher_;
        visualization_msgs::MarkerArray arr_;
    } // namespace

    std::string& getFrame(){
        return frame_;
    }
    std::mutex& getMutex(){
        return mutex_;
    }
    visualization_msgs::MarkerArray& getArr(){
        return arr_;
    }

    void Init(ros::NodeHandle &node, const std::string &frame,
              const std::string &topic)
    {
        frame_ = frame;
        publisher_ = node.advertise<visualization_msgs::MarkerArray>(topic, 10, true);
    }

    // 显示曲线
    void Plot(const std::vector<double> &xs, const std::vector<double> &ys,
              double width, Color color, int id, const std::string &ns)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = frame_;
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = (id >= 0) ? id : arr_.markers.size();
        msg.scale.x = 0.5;
        msg.scale.y = 0.5;
        msg.scale.z = 0.5;
        msg.color = color.toColorRGBA();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;

        for (size_t i = 0; i < xs.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = xs[i];
            pt.y = ys[i];
            msg.points.push_back(pt);
        }
        mutex_.lock();
        arr_.markers.push_back(msg);
        mutex_.unlock();
    }

    void PlotLine(const std::vector<double> &xs, const std::vector<double> &ys,
                  double width, Color color, int id, const std::string &ns)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = frame_;
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = id >= 0 ? id : arr_.markers.size();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::LINE_STRIP;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;
        msg.color = color.toColorRGBA();

        for (size_t i = 0; i < xs.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = xs[i];
            pt.y = ys[i];
            pt.z = 0.1 * id;
            msg.points.push_back(pt);
        }

        mutex_.lock();
        arr_.markers.push_back(msg);
        mutex_.unlock();
    }

    void PlotLine(const std::vector<int> &xs, const std::vector<int> &ys,
                  double width, Color color, int id, const std::string &ns)
    {
        visualization_msgs::Marker msg;
        msg.header.frame_id = frame_;
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = id >= 0 ? id : arr_.markers.size();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::LINE_STRIP;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;
        msg.color = color.toColorRGBA();

        for (size_t i = 0; i < xs.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = xs[i];
            pt.y = ys[i];
            pt.z = 0.1 * id;
            msg.points.push_back(pt);
        }

        mutex_.lock();
        arr_.markers.push_back(msg);
        mutex_.unlock();
    }
    
    void PlotPolygon(const std::vector<double> &xs, const std::vector<double> &ys,
                     double width, Color color, int id, const std::string &ns)
    {
        auto xss = xs;
        auto yss = ys;
        xss.push_back(xs[0]);
        yss.push_back(ys[0]);
        PlotLine(xss, yss, width, color, id, ns);
    }

    void PlotTraj(const std::vector<Vec4d> &ps,
            double width, Color color, int id, const std::string &ns){
        visualization_msgs::Marker msg;
        msg.header.frame_id = getFrame();
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = (id >= 0) ? id : getArr().markers.size();
        msg.scale.x = 0.5;
        msg.scale.y = 0.5;
        msg.scale.z = 0.5;
        msg.color = color.toColorRGBA();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;

        for (size_t i = 0; i < ps.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = ps[i].x();
            pt.y = ps[i].y();
            msg.points.push_back(pt);
        }
        getMutex().lock();
        getArr().markers.push_back(msg);
        getMutex().unlock();
    }

    void PlotTraj(const TypeVectorVecd<4> &ps,
            double width, Color color, int id, const std::string &ns){
        visualization_msgs::Marker msg;
        msg.header.frame_id = getFrame();
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = (id >= 0) ? id : getArr().markers.size();
        msg.scale.x = 0.5;
        msg.scale.y = 0.5;
        msg.scale.z = 0.5;
        msg.color = color.toColorRGBA();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;

        for (size_t i = 0; i < ps.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = ps[i].x();
            pt.y = ps[i].y();
            msg.points.push_back(pt);
        }
        getMutex().lock();
        getArr().markers.push_back(msg);
        getMutex().unlock();
    }

    void PlotTraj(const std::vector<TrajectoryPoint> &ps,
            double width, Color color, int id, const std::string &ns){
        visualization_msgs::Marker msg;
        msg.header.frame_id = getFrame();
        msg.header.stamp = ros::Time();
        msg.ns = ns;
        msg.id = (id >= 0) ? id : getArr().markers.size();
        msg.scale.x = 0.5;
        msg.scale.y = 0.5;
        msg.scale.z = 0.5;
        msg.color = color.toColorRGBA();

        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.pose.orientation.w = 1.0;
        msg.scale.x = width;

        for (size_t i = 0; i < ps.size(); i++)
        {
            geometry_msgs::Point pt;
            pt.x = ps[i].x;
            pt.y = ps[i].y;
            msg.points.push_back(pt);
        }
        getMutex().lock();
        getArr().markers.push_back(msg);
        getMutex().unlock();
    }

    void TriggerRemain()
    {
        mutex_.lock();
        publisher_.publish(arr_);
        // arr_.markers.clear();
        mutex_.unlock();
    }

    void Trigger()
    {
        mutex_.lock();
        publisher_.publish(arr_);
        arr_.markers.clear();
        mutex_.unlock();
    }

    void Clear()
    {
        mutex_.lock();
        arr_.markers.clear();

        visualization_msgs::MarkerArray arr;
        visualization_msgs::Marker msg;
        msg.header.frame_id = frame_;
        msg.ns = "Markers";

        msg.action = visualization_msgs::Marker::DELETEALL;
        arr.markers.push_back(msg);
        publisher_.publish(arr);
        mutex_.unlock();
    }
} // namespace visual_rviz