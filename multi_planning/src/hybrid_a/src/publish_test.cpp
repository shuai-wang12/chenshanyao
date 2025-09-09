#include "publish_test.h"

void publish_middle_points(ros::Publisher& mp_pub,
                            std::vector<middlePoint>& mp_vector) {
    visualization_msgs::Marker mp;
    geometry_msgs::Point temp;
    mp.header.frame_id="world";
    mp.header.stamp=ros::Time::now();
    mp.action=visualization_msgs::Marker::ADD;
    mp.type=visualization_msgs::Marker::POINTS;
    mp.id=1;
    mp.scale.x=1;
    mp.scale.y=1;
    mp.scale.z=0.1;

    mp.color.a = 1;
    mp.color.r = 1.0;
    mp.color.b = 0.0;
    mp.color.g = 0.0;

    mp.pose.orientation.w=1.0f;
    for(int i=0;i<mp_vector.size();i++){
        temp.x=mp_vector[i].pos.x();
        temp.y=mp_vector[i].pos.y();
        mp.points.emplace_back(temp);
    }
    mp_pub.publish(mp);
}

void publish_env(ros::Publisher& pub,uint8_t* map_data,int sizeX,int sizeY){
    visualization_msgs::Marker p_array;
    geometry_msgs::Point temp;
    p_array.header.frame_id="test";
    p_array.header.stamp=ros::Time::now();
    p_array.action=visualization_msgs::Marker::ADD;
    p_array.type=visualization_msgs::Marker::POINTS;
    p_array.id=0;
    p_array.pose.orientation.w=1.0;
    p_array.scale.x=1;
    p_array.scale.y=1;
    p_array.scale.z=1;

    p_array.color.a = 1;
    p_array.color.r = 1.0;
    p_array.color.b = 0.0;
    p_array.color.g = 0.0;
    for(int i=0;i<sizeX;i++){
        for(int j=0;j<sizeY;j++){
            if(map_data[i+j*sizeX]==1){
                temp.x=i;
                temp.y=j;
                p_array.points.push_back(temp);
            }
        }
    }
    pub.publish(p_array);
}

void publish_path(ros::Publisher& pub,const VectorVec3d& path){
  nav_msgs::Path nav_path;
  
  geometry_msgs::PoseStamped pose_stamped;
  for (const auto &pose : path) {
    pose_stamped.header.frame_id = "world";
    pose_stamped.pose.position.x = pose.x();
    pose_stamped.pose.position.y = pose.y();
    pose_stamped.pose.position.z = 0.0;
    pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(pose.z());

    nav_path.poses.emplace_back(pose_stamped);
  }

  nav_path.header.frame_id = "world";
  nav_path.header.stamp = ros::Time::now();

  pub.publish(nav_path);
}

void publish_flag(ros::Publisher& pub){
    std_msgs::Bool msg;
    msg.data=true;
    pub.publish(msg);
}