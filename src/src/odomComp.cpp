#include <math.h>
#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "nav_msgs/Odometry.h"
#include "project/paramsToGivenPose.h"
#include <dynamic_reconfigure/server.h>
#include <project/parametersConfig.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>

using namespace std;

enum integr_enum {EULER, RUNGE_KUTTA};

class OdomComputer{
public:
	OdomComputer(){
		integration_sub = n.subscribe("/cmd_vel", 1000, &OdomComputer::callback, this);
		pub_odom = n.advertise<nav_msgs::Odometry>("/odom", 1000);

		//dyn reconfigure configuration
		dynamic_reconfigure::Server<project::parametersConfig>::CallbackType parameters_handler;
		parameters_handler = boost::bind(&OdomComputer::set_parameters, this, _1, _2);
		this->parameters_server.setCallback(parameters_handler);
		//default integration method
		this->integr_method = RUNGE_KUTTA;

		this->n.getParam("/initialPose_x", this->x_old_position);
		this->n.getParam("/initialPose_y", this->y_old_position);
		this->n.getParam("/initialPose_theta", this->old_theta);

		//service configuration
		this->params_to_given_pose_service = n.advertiseService("params_to_given_pose", &OdomComputer::params_to_given_pose, this);
		
	}

	void callback (const geometry_msgs::TwistStamped::ConstPtr& msg){
		double vx, vy;
		double dt;
		double tmp;
		
		vx = msg->twist.linear.x;
		vy = msg->twist.linear.y;
		w = msg->twist.angular.z;
		old_header.seq = msg->header.seq;

		tmp = (msg->header.stamp.nsec - old_n_second);
		dt = (msg->header.stamp.sec - old_second) + (tmp / 1000000000);
		if (integr_method == EULER){				 
			x_new_position = x_old_position + (vx * cos(old_theta) - vy * sin(old_theta)) * dt;
			y_new_position = y_old_position + (vx * sin(old_theta) + vy * cos(old_theta)) * dt;
		}
		else if (integr_method == RUNGE_KUTTA){
			x_new_position = x_old_position + (vx * cos(old_theta + (w * dt * 0.5)) - vy * sin(old_theta + (w * dt * 0.5))) * dt;
			y_new_position = y_old_position + (vx * sin(old_theta + (w * dt * 0.5)) + vy * cos(old_theta + (w * dt * 0.5))) * dt;
		}
		new_theta = old_theta + w * dt;


		// publication of the Odometry
		nav_msgs::Odometry msg_odometry;

		msg_odometry.header.seq = msg->header.seq;
		msg_odometry.header.stamp = msg->header.stamp;
		msg_odometry.header.frame_id = "odom";

		msg_odometry.child_frame_id = "base_link";

		msg_odometry.pose.pose.position.x = x_new_position;
		msg_odometry.pose.pose.position.y = y_new_position;
		msg_odometry.pose.pose.position.z = 0.0; 
		msg_odometry.pose.pose.orientation = tf::createQuaternionMsgFromYaw(new_theta);

		/*for (int i = 0; i < 16; i++) {
			msg_odometry.pose.covariance[i] = 0.0;
		}*/

		msg_odometry.twist.twist = msg->twist;

		/*for (int i = 0; i < 16; i++) {
			msg_odometry.twist.covariance[i] = 0.0;
		}*/

		this->pub_odom.publish(msg_odometry);

		// saving of the values needed for the next integration
		this->x_old_position = x_new_position;
		this->y_old_position = y_new_position;
		this->old_theta = new_theta;
		old_header = msg->header;
		old_second = msg->header.stamp.sec;
		old_n_second = msg->header.stamp.nsec;
	}

		void set_parameters(project::parametersConfig &config, uint32_t level) {
		/* 
		*	callback of the dynamic reconfigure server: depending on the parameter received,
		*	sets the integration method to Euler or Runge-Kutta
		*/
			switch (config.integr_method) {
				case 0: this->integr_method = EULER; cout << "changed in EULER" << endl; break;
				case 1: this->integr_method = RUNGE_KUTTA; cout << "changed in RUNGE_KUTTA" << endl; break;
			}
		}


		bool params_to_given_pose(project::paramsToGivenPose::Request  &req,
	        project::paramsToGivenPose::Response &res) {
		/*
		*	 callback of the service paramsToGivenPose: forces the robot pose to 
		* 	(x,y,theta) chosen by the caller of the service
		*/	
			this->x_old_position = req.x;
			this->y_old_position = req.y;
			this->old_theta = req.theta; 

			return true;
		}

private:
	ros::NodeHandle n;
	ros::Subscriber integration_sub;
	ros::Publisher pub_odom;
	std_msgs::Header old_header;

	double v;
	double w;
	double new_theta, old_theta;
	double x_new_position, x_old_position;
	double y_new_position, y_old_position;
	double old_second,old_n_second;

	dynamic_reconfigure::Server<project::parametersConfig> parameters_server; 

	integr_enum integr_method; // the integration method used to compute the odometry
	ros::ServiceServer params_to_given_pose_service;
};

int main (int argc, char **argv){
	ros::init(argc, argv, "odometry_computer");
	OdomComputer oc;
	ros::spin();
	return 0;
}