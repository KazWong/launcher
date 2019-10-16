#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_listener.h"
#include <iostream>
#include "generate_tf/gen_tf.h"


class echoListener
{
public:

  tf::TransformListener tf;
  gen_tf::Gentf tf_;
  echoListener() {

  };

  ~echoListener() {

  };

private:

};

int main(int argc, char ** argv)
{
  ros::init(argc, argv, "write_tf_yaml_node", ros::init_options::AnonymousName);

  if (argc < 4)
  {
    printf("Not enough arguments provided.\n\n");
    printf("1. tf type, choose from 'nav','park','grab'\n");
    printf("2. tf cordinate frame, try 'map','ar_marker_0','tool0_controller'\n");
    printf("3. tf target frame, try 'nav_goal_CS','ar_parked_HT' \n");
    return -1;
  }

  ros::NodeHandle nh;

  //Instantiate a local listener
  echoListener echoListener;
  std::string reference_frameid;
  std::string tf_type = std::string(argv[1]);
  std::string cordinate_frameid = std::string(argv[2]);
  std::string target_frameid = std::string(argv[3]);
  bool is_static_tf = false;

  if (tf_type.compare("nav") == 0) {
     reference_frameid = "base_footprint";
     is_static_tf = true;
  }  else if (tf_type.compare("park")==0) {
     reference_frameid = "ur_base";
  }  else if (tf_type.compare("grab")==0) {
     reference_frameid = "tool0_controller_base";
  }  else {
     ROS_ERROR("undefined tf type");
     return -1;
  }

  // Wait for up to two second for the first transforms to become avaiable. 
  echoListener.tf.waitForTransform(cordinate_frameid, reference_frameid, ros::Time(), ros::Duration(2.0));

  //Nothing needs to be done except wait for a while
  //The callbacks withing the listener class
  //will take care of everything
 
  try {
        tf::StampedTransform echo_transform;
        echoListener.tf.lookupTransform(cordinate_frameid, reference_frameid, ros::Time(), echo_transform);
        std::cout.precision(3);
        std::cout.setf(std::ios::fixed,std::ios::floatfield);
        std::cout << "At time " << echo_transform.stamp_.toSec() << std::endl;
        double yaw, pitch, roll;
        echo_transform.getBasis().getRPY(roll, pitch, yaw);
        tf::Quaternion q = echo_transform.getRotation();
        tf::Vector3 v = echo_transform.getOrigin();
        std::cout << "- Translation: [" << v.getX() << ", " << v.getY() << ", " << v.getZ() << "]" << std::endl;
        std::cout << "- Rotation: in Quaternion [" << q.getX() << ", " << q.getY() << ", " 
                  << q.getZ() << ", " << q.getW() << "]" << std::endl
                  << "            in RPY (radian) [" <<  roll << ", " << pitch << ", " << yaw << "]" << std::endl
                  << "            in RPY (degree) [" <<  roll*180.0/M_PI << ", " << pitch*180.0/M_PI << ", " << yaw*180.0/M_PI << "]" << std::endl;
        if (is_static_tf)
            echoListener.tf_.write_to_yaml(cordinate_frameid,target_frameid, v.getX(), v.getY(),v.getZ(),roll,pitch,yaw,true);
        else
            echoListener.tf_.write_to_yaml(cordinate_frameid,target_frameid, v.getX(), v.getY(),v.getZ(),roll,pitch,yaw,false);
      }
      catch(tf::TransformException& ex)
      {
        std::cout << "Failure at "<< ros::Time::now() << std::endl;
        std::cout << "Exception thrown:" << ex.what()<< std::endl;
        std::cout << "The current list of frames is:" <<std::endl;
        std::cout << echoListener.tf.allFramesAsString()<<std::endl;   
        return -1;    
      }

  return 0;
};

