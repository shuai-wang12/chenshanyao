#pragma once

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <nav_msgs/Path.h>
#include "hybrid_a/middlePark.h"
#include "type.h"
#include <std_msgs/Bool.h>
/**
 * 本文件定义publsh的函数
 * publisher声明在外部
*/

void publish_middle_points(ros::Publisher& mp_pub,std::vector<middlePoint>& mp_vector);
void publish_env(ros::Publisher& pub,uint8_t* map_data,int sizeX,int sizeY);
void publish_path(ros::Publisher& pub,const VectorVec3d& path);
void publish_flag(ros::Publisher& pub);