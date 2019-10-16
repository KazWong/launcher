#include <iostream>
#include <cmath>
#include <string>
#include <ros/ros.h>
#include <tf/tf.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>
#include <memory>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Twist.h>
#include <ghost/StringCmd.h>
#include <std_srvs/Empty.h>

using namespace std;
using namespace ros;


/********************
Goal : ROS navigation endpoint of the global planner
Destination : Final endpoint of the task
SimpleLoopGoal : Loop until reach Goal
GoToPoint : command try to go to a point
*********************/

class MB {
private:
  typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
  
  ros::NodeHandle nh;
  MoveBaseClient ac;
  ros::ServiceClient srv_clearmap;
  ros::ServiceClient park_client;
  ros::Publisher cmd_pos_pub;
  ros::Publisher maxvel_pub;
  tf::TransformListener listener;  
  std::unique_ptr<tf::StampedTransform> final_destination; 
  int fd;
  string path;
  string named_pipe;
  int mapreset, mapreset_counter; 

public:
  MB(string _path = "") : nh(), ac("move_base", true), listener(ros::Duration(1.5)), 
                  mapreset(-1), mapreset_counter(0), path(_path) {
    while( !ac.waitForServer(ros::Duration(2.0)) && ros::ok() )
      ROS_INFO("Waiting for the move_base action server to come up");  
    
    srv_clearmap = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
  if (path != "")
    Open(path);
    
    sleep(2);
  }

  ~MB() {
    if (path != "")
      Close();
  }
  
  
  void ClearCostMap() {
    std_srvs::Empty srv;
    srv_clearmap.call(srv);
  }
 
  void MapResetCount(int max) {
    mapreset = max;
  }
  
  void CancelGoal() {
    ac.cancelGoal();
  }
  
  bool Destination(string s) {
    tf::StampedTransform destination;
    
    try {
      listener.waitForTransform( "map", s, ros::Time(0), ros::Duration(0.1) );
      listener.lookupTransform( "map", s, ros::Time(0), destination);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    final_destination.reset( new tf::StampedTransform(destination) );
  }
  
  bool Destination(double *f_x, double *f_y) {
    if (final_destination != nullptr) {
      bool reached = false;
      tf::StampedTransform robot_pose;
      double des_vec_x, des_vec_y;
      
      try {
        listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(0.1) );
        listener.lookupTransform( "map", "base_footprint", ros::Time(0), robot_pose);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
      }
      
      des_vec_x = (*final_destination).getOrigin().getX() - robot_pose.getOrigin().getX();
      des_vec_y = (*final_destination).getOrigin().getY() - robot_pose.getOrigin().getY();
      
      double des_th = atan2(des_vec_y, des_vec_x);
      double f_th = atan2(*f_y, *f_x);
      double des_m = sqrt(des_vec_x * des_vec_x + des_vec_y * des_vec_y);
      double f_m = sqrt(*f_x * (*f_x) + *f_y * (*f_y));
      double yaw = tf::getYaw(robot_pose.getRotation());

//ROS_INFO("%f\t%f\t%f\t%f\t%f", des_th, f_th, des_m, f_m, fabs(f_th - des_th));             
      if (des_m > f_m /* && fabs(f_th - des_th) <= 1.570796 /*0.392699*/ && f_m > 0.01) {
        ROS_INFO("Follow something");  
        if ( goToPoint(*f_x * cos(yaw) - *f_y * sin(yaw), *f_x * sin(yaw) + *f_y * cos(yaw), 0.0) ) {
          *f_x = 0.0;
          *f_y = 0.0;
        }
      } else {
        ROS_INFO("Follow nothing");  
        reached = goToPoint(des_vec_x, des_vec_y, 0.0);
        if (reached)
          final_destination.reset();
        return reached;
      }
      
      return false;
    }
  }
  
  void SimpleLoopGoal(string s, bool clear) {
    while ( !goToPoint(s, false) && ros::ok() ) {
      ROS_INFO("%s", s.c_str());
      sleep(1);
    }
    if (clear) {
      ClearCostMap();
      sleep(2);
    }
  }
  
  void SimpleLoopGoal(float x, float y, float z, float q_x, float q_y, float q_z, float q_w, bool clear) {
    while ( !goToPoint(x, y, z, q_x, q_y, q_z, q_w) && ros::ok() ) {
      sleep(1);
    }
    if (clear) {
      ClearCostMap();
      sleep(2);
    }
  }
  
  bool goToPoint(string const nav_goal, bool align) {    
    tf::StampedTransform transform_nav_goal_pose; 
    tf::StampedTransform robo_pose; 
    
    std::cout << nav_goal << std::endl;

    if ( !listener.frameExists(nav_goal) ) {
      std::cout << "frame does not exist" << std::endl;
      return false;
    }

    try {
      listener.waitForTransform( "map", nav_goal, ros::Time(0), ros::Duration(0.1) );
      listener.lookupTransform( "map", nav_goal, ros::Time(0), transform_nav_goal_pose);
      listener.waitForTransform( "map", "base_footprint", ros::Time(0), ros::Duration(0.1) );
      listener.lookupTransform( "map", "base_footprint", ros::Time(0), robo_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    if (align)
      return goToPoint( transform_nav_goal_pose.getOrigin(), transform_nav_goal_pose.getRotation() );
    else
      return goToPoint( transform_nav_goal_pose.getOrigin(), robo_pose.getRotation() );
  }
  
  bool goToPoint(float x, float y, float z) {  
    tf::StampedTransform robo_pose; 
  	
    try {
      listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform( "map", "base_footprint", ros::Time(0), robo_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    return goToPoint( tf::Vector3( robo_pose.getOrigin().getX() + x, robo_pose.getOrigin().getY() + y, robo_pose.getOrigin().getZ() + z ), robo_pose.getRotation() );
  }
  
  bool goToPoint(float x, float y, float z, float q_x, float q_y, float q_z, float q_w) {  
    tf::StampedTransform robo_pose; 
  	
    try {
      listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1));
      listener.lookupTransform( "map", "base_footprint", ros::Time(0), robo_pose);
    } catch (tf::TransformException ex) {
      ROS_ERROR("%s",ex.what());
      return false;
    }
    
    return goToPoint( tf::Vector3( robo_pose.getOrigin().getX() + x, robo_pose.getOrigin().getY() + y, robo_pose.getOrigin().getZ() + z ), 
                      tf::Quaternion( robo_pose.getRotation().x() + q_x, robo_pose.getRotation().y() + q_y, robo_pose.getRotation().z() + q_z, robo_pose.getRotation().w() + q_w ) );
  }

  
  bool goToPoint(tf::Vector3 origin, tf::Quaternion rotation) {     
    move_base_msgs::MoveBaseGoal goal;
    actionlib::SimpleClientGoalState state = actionlib::SimpleClientGoalState::LOST;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    
    goal.target_pose.pose.position.x = origin.getX();
    goal.target_pose.pose.position.y = origin.getY();
    goal.target_pose.pose.position.z = origin.getZ();
    goal.target_pose.pose.orientation.x = rotation.x(); 
    goal.target_pose.pose.orientation.y = rotation.y(); 
    goal.target_pose.pose.orientation.z = rotation.z(); 
    goal.target_pose.pose.orientation.w = rotation.w(); 
    ac.sendGoal(goal);
    
    ac.waitForResult(ros::Duration(0.6));
    
    state = ac.getState();
    ROS_INFO_STREAM(state.toString());
    
    if(state == actionlib::SimpleClientGoalState::SUCCEEDED) {
      tf::StampedTransform robo_goal_pose; 
      
      try {
        listener.waitForTransform("map", "base_footprint", ros::Time(0), ros::Duration(0.1));
        listener.lookupTransform( "map", "base_footprint", ros::Time(0), robo_goal_pose);
      } catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        return false;
      }
      
      //double x2 = robo_goal_pose.getOrigin().getX() - origin.getX();
      //double y2 = robo_goal_pose.getOrigin().getY() - origin.getY();

      //std::cout << robo_goal_pose.getOrigin().getX() << origin.getX() << std::endl;
      //std::cout << robo_goal_pose.getOrigin().getY() << origin.getY() << std::endl;

      //if ( sqrt(x2*x2 + y2*y2) < 0.1 ) {
        mapreset_counter = 0;
        ACK(true);
        return true;
      //}
    }
    
    if (mapreset_counter > mapreset && mapreset > 0) {
      ac.cancelGoal();
      ClearCostMap();
      ROS_INFO("map reset");
      //sleep(1);
      ac.sendGoal(goal);
      mapreset_counter = 0;
    }
    
    if(state == actionlib::SimpleClientGoalState::ABORTED) {
      mapreset_counter++;
      return false;
    } else
      mapreset_counter = 0;
      
    return false;
  }
  
//////////////////////////////////////////////////
/* named pipe */
//////////////////////////////////////////////////

  void ACK(bool x) {
    char ack[10];
    (x)? sprintf(ack, "1\r\n") : sprintf(ack, "0\r\n");

    write(fd, ack, sizeof(ack));
    std::cout << "ACK\t" << ack << std::endl;
  }
  
  void Open(string _path) {
    char *path = new char [_path.size()+1];
    strcpy( path, _path.c_str() );
    
    //mkfifo(path, 0666);
    fd = open(path, O_WRONLY);
    delete path;
  }
  
  void Close() {
    close(fd);
    //unlink("/tmp/ack");
  }
};
