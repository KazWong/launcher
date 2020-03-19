#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#include "math.h"
#include "serial.h"
#include <thread>
#include <mutex> 

class Airport {

public:
	Named_Pipe encoder;
	//STM32_NUCLEO encoder;
	STM32_NUCLEO control;

public:
	Airport(const char *cmd_port="/dev/ttyACM0", const char *en_port="/tmp/encoderXYZ") : encoder(en_port), control(cmd_port) {}
   
	~Airport() {}
	
	void ReadData(float *vel, float *odom) {
		char msg1[512], msg2[512];
		char cmd1[6], cmd2[6];
		float d1[3], d2[3], *a, *b;
		char* pEnd;
		
		encoder.Read(msg1, 512);
		encoder.Read(msg2, 512);

		sscanf(msg1, "%s %g %g %g", cmd1, &d1[0], &d1[1], &d1[2]);
		sscanf(msg2, "%s %g %g %g", cmd2, &d2[0], &d2[1], &d2[2]);

		if ( strcmp(cmd1, "VEL") == 0 && strcmp(cmd2, "COD") == 0 ) {
			a = d1;
			b = d2;
		} else if ( strcmp(cmd1, "COD") == 0 && strcmp(cmd2, "VEL") == 0 ) {
			a = d2;
			b = d1;
		} else {
			std::cout << "NO DATA" << std::endl;
			return;
		}
		
		for (int i=0;i<3;i++)
			vel[i] = a[i];
		for (int i=0;i<3;i++)
			odom[i] = b[i];
	}
	
	// vx, vy unit: meter per second
	// STM32 receive  millimeter per second for x, y
	// and unit: rad per second for z
	void Move(float vx=0, float vy=0, float vrz=0) {
		char msg[50];
		
		sprintf(msg, "%g,%g,%g\n", -vy * 1000, vx * 1000, vrz);
		control.Write(msg);
	}

	void Unlock(int r) {
		char msg[50];
		
		sprintf(msg, "%d,%d,%d\n", 65535, 65535, r);
		control.Write(msg);
	}
   
	void NVICReset() {
		char msg[] = "65535,65535,65535\n";
		
		control.Write(msg);
	}

};

Airport ser;
bool alive = true;
double _tgt_vel[3];
double cmd_vel[3];
std::mutex mtx;
std::mutex cmd_mtx;

void ControlLoopCallback() {
	while(alive) {
		cmd_mtx.lock();
		ser.Move(cmd_vel[0], cmd_vel[1], cmd_vel[2]);
		ROS_INFO("MOV\t[%.4f, %.4f, %.4f]", cmd_vel[0], cmd_vel[1], cmd_vel[2]);
		cmd_mtx.unlock();
		ros::Duration(0.1).sleep();
	}
}

void CmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_vel) {
	ROS_INFO("CMD\t[%.4f, %.4f, %.4f]", cmd_vel->linear.x, cmd_vel->linear.y, cmd_vel->angular.z);
	
	mtx.lock();
        _tgt_vel[0] = cmd_vel->linear.x;
        _tgt_vel[1] = cmd_vel->linear.y;
        _tgt_vel[2] = cmd_vel->angular.z;
	mtx.unlock();
}

int main(int argc, char** argv) {
	float odom[3];
	double period = 0.0125;

	cmd_vel[0] = cmd_vel[1] = cmd_vel[2] = 0.0;
        _tgt_vel[0] = _tgt_vel[1] = _tgt_vel[2] = 0.0;
	odom[0] = odom[1] = odom[2] = 0.0;
	ser.NVICReset();

	ros::init(argc, argv, "Drive");
	ros::NodeHandle hn;
	ros::Subscriber sub = hn.subscribe("/cmd_vel", 1, CmdVelCallback);
	ros::Publisher odom_pub = hn.advertise<nav_msgs::Odometry>("/odom", 1);
	tf::TransformBroadcaster tf_pub;
	ros::Time current_time, last_time;
	std::thread control(ControlLoopCallback);
	
	while ( ros::ok() ) {
		float vel[3];
		double acc[3];
		//float odom_fake[3];
		current_time = ros::Time::now();
		double dt = (current_time - last_time).toSec();
		nav_msgs::Odometry odom_msg;
		tf::Transform odom_tf;
	
		vel[0] = vel[1] = vel[2] = 0.0;
		
		ser.ReadData(vel, odom);	
	
		//odom[0] += ( vel[0] * cos(odom[2]) - vel[1] * sin(odom[2]) ) * dt;
		//odom[1] += ( vel[0] * sin(odom[2]) + vel[1] * cos(odom[2]) ) * dt;
		//odom[2] += vel[2] * dt;
		
		ROS_INFO("VEL\t[%.4f, %.4f, %.4f]", vel[0], vel[1], vel[2]);
		ROS_INFO("COD\t[%.4f, %.4f, %.4f]", odom[0], odom[1], odom[2]);
		//ROS_INFO("SCO\t[%.4f, %.4f, %.4f]", odom_fake[0], odom_fake[1], odom_fake[2]);

		odom_msg.header.stamp = current_time;
		odom_msg.header.frame_id = "odom";
		odom_msg.child_frame_id = "base_footprint";
		odom_msg.pose.pose.position.x = odom[0];
		odom_msg.pose.pose.position.y = odom[1];
		odom_msg.pose.pose.position.z = 0.0;
		odom_msg.pose.pose.orientation = tf::createQuaternionMsgFromYaw(odom[2]);
		odom_msg.twist.twist.linear.x  = vel[0];
		odom_msg.twist.twist.linear.y  = vel[1];
		odom_msg.twist.twist.angular.z = vel[2];

		odom_tf.setOrigin( tf::Vector3(odom[0], odom[1], 0.0) );
		odom_tf.setRotation(tf::createQuaternionFromYaw(odom[2]));
		
		tf_pub.sendTransform( tf::StampedTransform(odom_tf, current_time, "odom", "base_footprint") );
		odom_pub.publish(odom_msg);

		double tgt_vel[3];
		mtx.lock();
		for (int i=0;i<3;i++)
			tgt_vel[i] = _tgt_vel[i];
		mtx.unlock();
		
		(fabs(tgt_vel[0] - vel[0]) > 25.)? acc[0] = 25.:acc[0] = (tgt_vel[0] - vel[0]);
		(fabs(tgt_vel[1] - vel[1]) > 25.)? acc[1] = 25.:acc[1] = (tgt_vel[1] - vel[1]);
		(fabs(tgt_vel[2] - vel[2]) > 32.)? acc[2] = 32.:acc[2] = (tgt_vel[2] - vel[2]);
		
		cmd_mtx.lock();
		cmd_vel[0] = acc[0] * 0.1 + vel[0];
		cmd_vel[1] = acc[1] * 0.1 + vel[1];
		cmd_vel[2] = acc[2] * 0.1 + vel[2];
		cmd_mtx.unlock();
		
		last_time = current_time;
		
		ros::spinOnce();
		ros::Duration(period).sleep();
		ROS_INFO(" ");
	}
	
	ros::spin();
	ser.Move(0.0, 0.0, 0.0);

	//kill thread
	alive = false;
	if ( control.joinable() ) 
		control.join();

	return 0;
}
