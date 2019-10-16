#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sstream>
#include <iostream>
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

using namespace std;
string priname = "tf_broadcastor";

int main(int argc, char** argv){
  ros::init(argc, argv, "tf_broadcaster_node");
  ros::NodeHandle n;
  tf::Quaternion q;
  tf::Vector3 pos;

  ros::Rate r(50);
//  cout <<"load yaml"<<endl;
  YAML::Node index = YAML::LoadFile("/home/sae/gen3/te_lab/src/generate_tf/param/tf_broadcastor_index.yaml");
 // cout <<"load yaml11"<<endl;
  tf::TransformBroadcaster broadcaster;
  while(n.ok()){
    
    int tf_num = index.size();
    tf_num++;
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
          n.getParam(child_tf_index,child_tf);

          parent_tf_index = priname+"/"+child_tf+"/parent_frame";
          n.getParam(parent_tf_index,parent_tf);

          transform_tf_index = priname+"/"+child_tf+"/transform";

          n.getParam(transform_tf_index,transform_);
          for(int k=0; k<6;k++) {
              pos_[k] = transform_[k];
          }
          q.setRPY(pos_[3], pos_[4], pos_[5]);
          pos.setValue(pos_[0], pos_[1], pos_[2]);
          broadcaster.sendTransform(
            tf::StampedTransform(
              tf::Transform( q, pos ),
              ros::Time::now(), parent_tf, child_tf));
    }

  ros::spinOnce();
  r.sleep();
  }
}

