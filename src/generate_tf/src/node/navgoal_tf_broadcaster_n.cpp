#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "navgoal_tf_broadcaster");
  ros::NodeHandle n;
  tf::Quaternion q;
  tf::Vector3 pos;

  ros::Rate r(50);

  tf::TransformBroadcaster broadcaster;
  while(n.ok()){
    
    q.setRPY(0.220, -0.540, -1.530);
    pos.setValue(1.006, 0.214, 0.000);
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform( q, pos ),
        ros::Time::now(), "map", "base_footprint"));
    
    q.setRPY(0.0, -0.0, -1.0);
    pos.setValue(1.0,2 , 0.000);
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform( q, pos ),
        ros::Time::now(), "tool0_controller_base", "tool0_controller"));

    q.setRPY(0.0, -0.0, -1.0);
    pos.setValue(1.0,2 , 0.000);
    broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform( q, pos ),
        ros::Time::now(),"ar_marker0", "ur_base"));

  ros::spinOnce();
  r.sleep();
  }
}
