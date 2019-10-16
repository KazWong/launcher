#include <string>
#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Vector3.h>
#include "movebase_client.h"
#include <thread>
#include <mutex> 
#include <fcntl.h>
#include <stdio.h>
#include <chrono>

using namespace std;
using namespace ros;

mutex mtx;
int route = 0;
int option = 2;
bool alive = true;
double x, y, z, q_x, q_y, q_z, q_w;

void Route(const std_msgs::Int16::ConstPtr& msg) {
  if (msg->data >= 0)
    route = msg->data;
}

void Goal(const geometry_msgs::Transform::ConstPtr& msg) {
  mtx.lock();
  x = msg->translation.x;
  y = msg->translation.y;
  z = msg->translation.z;
  q_x = msg->rotation.x;
  q_y = msg->rotation.y;
  q_z = msg->rotation.z;
  q_w = msg->rotation.w;
  mtx.unlock();
  
  route = -1;
}

void Follow(const geometry_msgs::Vector3::ConstPtr& msg) {
  float tmp_x = 65535.0, tmp_y = 65535.0;
     
  mtx.lock();
  if (msg->x != -x || msg->y != -y)
    route = 1;
  x = -msg->x;
  y = -msg->y;
  mtx.unlock();
}

void NamedPipeCallback() {
  int fd = -1;
  char pipe[] = "/tmp/followerXY";
  char buf[512];
  
  //while (fd = open(pipe, O_RDONLY) < 0 && alive) 
    //std::this_thread::sleep_for(chrono::milliseconds(1000));
  fd = open(pipe, O_RDONLY);

  ROS_INFO("Receive thread inited");
  while (alive) {
    float tmp_x = 65535.0, tmp_y = 65535.0;
    if (read(fd, buf, 512) > 0) {
      sscanf(buf, "%g %g", &tmp_x, &tmp_y);
      
      mtx.lock();
      if (tmp_x != x || tmp_y != y) {
        route = 1;
        x = (double)tmp_x;
        y = (double)tmp_y;
        ROS_INFO("Received: %.5f, %.5f\n", x, y);
      }
      mtx.unlock();
    }

    //std::this_thread::sleep_for(chrono::milliseconds(100));
  }
  close(fd);

  //unlink(pipe);
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "wandering");
  NodeHandle nh;
  string gate;
  int counter;
  double lx, ly;
        
  counter = 0;
  lx = ly = 0.0;
  x = y = z = q_x = q_y = q_z = q_w = 0.0;

  thread pipe(NamedPipeCallback);

  MB move("/tmp/ack");
  Subscriber route_sub = nh.subscribe<std_msgs::Int16>("wandering_route", 1, Route);
  Subscriber goal_sub = nh.subscribe<geometry_msgs::Transform>("goal", 1, Goal);
  Subscriber follow_sub = nh.subscribe<geometry_msgs::Vector3>("follow", 1, Follow);
  
  move.MapResetCount(5);
  gate.clear();
  
  while (ros::ok()) {
    bool exit_route = false;

    ros::spinOnce();
    ros::Duration(0.1).sleep();

    switch(route) {
      case -1 : // -1 : loop to point
        move.SimpleLoopGoal(x, y, z, q_x, q_y, q_z, q_w, true);
        route = 0;
        break;
      case 0 : // 0 : hold
        sleep(1);
        break;
      case 1 : // 1-2 : gate - follow people any time
          mtx.lock();
          if ( gate.empty() && (x != 65535.0 && y == 65535.0) ) {
            char c[50] = "";
            sprintf(c, "%i", (int)x);

            ROS_INFO("Set Gate %s", c);
            gate = string(c);
            counter = 0;
            move.Destination(gate);
          } else if (x != 65535.0 && y != 65535.0) {
            double tmp_x, tmp_y;
 
            tmp_x = y * 0.8;
            tmp_y = -x * 0.8;
            
            if (tmp_x != lx || tmp_y != ly)
              counter = 0;
            lx = tmp_x;
            ly = tmp_y;
          }
          mtx.unlock();

          ROS_INFO("local xy: %.5f, %.5f\n", lx, ly);
          route = option;
          break;
      case 2 : // 2 : gate - select people 
        if (counter >= 2) {
          lx = ly = 0.0;
        }
        exit_route = move.Destination(&lx, &ly);
        counter++;

        if (exit_route) {
          route = 0;
          lx = ly = 0.0;
          gate.clear();
        } else
          route = 1;
        break;
      case 3 : // 3 : multi set point loop
        move.SimpleLoopGoal("point5", true);
        move.SimpleLoopGoal("point4", true);
        move.SimpleLoopGoal("point3", true);
        move.SimpleLoopGoal("point2", true);
        move.SimpleLoopGoal("point1", true);
        move.SimpleLoopGoal("point0", true);
        //route = 0;
        break;
      case 4 : // 4 : loop between 23 and 45
        move.SimpleLoopGoal("23", false);
        move.SimpleLoopGoal("45", false);
        break;
    }
  }
  
  //kill thread
  alive = false;
  if ( pipe.joinable() ) 
    pipe.join();

  return 0;
}
