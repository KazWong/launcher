#ifndef _GEN_TF_H_
#define _GEN_TF_H_

#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <sstream>
#include <iostream>  
#include <fstream> 
#include <string>
#include <vector>
#include "yaml-cpp/yaml.h"

using namespace std;
using namespace ros;

namespace gen_tf {


class Gentf{

public:
     double decode_x;
     double decode_y;
     double decode_z;
     double encode_x;
     double encode_y;
     double encode_z;

private:
     NodeHandle n;
      
public:
     Gentf();
     ~Gentf();

     void write_to_yaml(string parent_f, string child_f, double x,double y,double z, double rx, double ry, double rz, bool is_static_tf);

   
};

};//namespace

#endif
