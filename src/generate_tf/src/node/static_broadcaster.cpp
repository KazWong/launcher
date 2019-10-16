#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace ros;

Time t;
string priname = "static_tf";

geometry_msgs::TransformStamped SetTF(string frame_id, string child_frame_id, tf2::Vector3 pos, tf2::Quaternion quat) {
  geometry_msgs::TransformStamped static_transformStamped;
  
  static_transformStamped.header.stamp = t;
  static_transformStamped.header.frame_id = frame_id;
  static_transformStamped.child_frame_id = child_frame_id;
  
  static_transformStamped.transform.translation.x = pos.getX();
  static_transformStamped.transform.translation.y = pos.getY();
  static_transformStamped.transform.translation.z = pos.getZ();

  static_transformStamped.transform.rotation.x = quat.x();
  static_transformStamped.transform.rotation.y = quat.y();
  static_transformStamped.transform.rotation.z = quat.z();
  static_transformStamped.transform.rotation.w = quat.w();
  
  return static_transformStamped;
}

int main(int argc, char** argv){
  init(argc, argv, "static_broadcastor_node");
  NodeHandle n;
  
  tf2::Quaternion q;
  tf2::Vector3 pos;
  t = Time::now();
  vector<geometry_msgs::TransformStamped> static_tf;
  tf2_ros::StaticTransformBroadcaster static_broadcaster;
  
  YAML::Node index = YAML::LoadFile("/home/sae/gen3/te_lab/src/generate_tf/param/static_tf_index.yaml"); 
  int tf_num = index.size();
  tf_num++;
  cout << "tf_num is:" <<tf_num <<endl;
  string child_tf="";
  string parent_tf="";
  string child_tf_index="";
  string parent_tf_index="";
  string transform_tf_index="";
  vector<double> transform_;
  double pos_[6];
  for(int i=1; i < tf_num; i++) {
        string child_tf="";
        string parent_tf="";
        string child_tf_index="";
        string parent_tf_index="";
        string transform_tf_index="";
        vector<double> transform_;
        double pos_[6];
        child_tf_index = priname + "/index" + to_string(i) ;
        cout << "child_tf_index is:" <<child_tf_index<<endl;
        n.getParam(child_tf_index,child_tf);
        cout << "child_tf is:" <<child_tf<<endl;

        parent_tf_index = priname+"/"+child_tf+"/parent_frame";
        cout << "parent_tf_index is:" <<parent_tf_index<<endl;
        n.getParam(parent_tf_index,parent_tf);
        cout << "parent_tf is:" <<parent_tf<<endl;

        transform_tf_index = priname+"/"+child_tf+"/transform";
        cout << "transform_tf_index is:" <<transform_tf_index<<endl;

        n.getParam(transform_tf_index,transform_);  
        for(int k=0; k<6;k++) {
            pos_[k] = transform_[k];
            cout << "pos_[k] is:"<<pos_[k]<<endl;
        }
        q.setRPY(pos_[3], pos_[4], pos_[5]);
        pos.setValue(pos_[0], pos_[1], pos_[2]);
        static_tf.push_back(SetTF(parent_tf, child_tf, pos, q));
  }
        static_broadcaster.sendTransform(static_tf);
        spin();
}





