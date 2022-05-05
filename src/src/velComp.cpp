#include <math.h>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/calibration.h"

#define _USE_MATH_DEFINES
#define T 5

class VelComputer{
public:	
	VelComputer(){
		odom_sub= nh.subscribe("/wheel_states", 1000, &VelComputer::subCallback, this);
		odom_pub = nh.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1000);

		//service configuration
		calibration = nh.advertiseService("robot_calibration", &VelComputer::calibrationCallback, this);
	}

	void subCallback(const sensor_msgs::JointState::ConstPtr& msg){
		geometry_msgs::TwistStamped odom_to_publish;
		double dticks[4];
		double dt;
		double x; // temp variable used to compute dt
		double ticks_per_second[4]; // dticks/dt
		double old_second = old_header.stamp.sec; 
		double old_n_second = old_header.stamp.nsec; 
		double calibrated_w[4]; //in rad/s
		double sum_w = 0;
		double vx, vy, w;
	
		if (first_time == 0){
			old_header = msg->header;
			odom_to_publish.header = msg->header;
			for(int i = 0; i < 4; i++){
				old_position[i] = msg->position[i];
				old_velocity[i] = msg->velocity[i];
			}
			dt = msg->header.stamp.nsec / 1000000000;
			old_second = msg->header.stamp.sec;
			old_n_second = msg->header.stamp.nsec;
			first_time = 1;
		}

		if (old_header.seq != msg->header.seq){
			for(int i = 0; i < 4; i++){
				dticks[i] = msg->position[i] - old_position[i];
			}

			//compute dt
			x = (msg->header.stamp.nsec - old_n_second);
			dt = (msg->header.stamp.sec - old_second) + (x / 1000000000);

			//compute w in ticks/s
			for(int i = 0; i < 4; i++){
				ticks_per_second[i] = dticks[i] / dt;
				calibrated_w[i] = (ticks_per_second[i]/(T*n)) * 2 * M_PI;
			}
			
			// below calculation of Vx, Vy, Wz

			//Vx
			for(int i = 0; i < 4; i++){
				sum_w = sum_w + calibrated_w[i];
			}

			vx = (sum_w * r) / 4;
			sum_w = 0;

			//Vy
			for(int i = 0; i < 4; i++){
				if (i == 0 or i == 3){
					sum_w = sum_w - calibrated_w[i];
				}
				else {
					sum_w = sum_w + calibrated_w[i];
				}
			}
			vy = (sum_w * r) / 4;
			sum_w = 0;

			//Wz
			for(int i = 0; i < 4; i++){
				if (i == 0 or i == 2){
					sum_w = sum_w - calibrated_w[i];
				}
				else {
					sum_w = sum_w + calibrated_w[i];
				}
			}
			w = (sum_w * r) / (4 * (lx + ly));

			//Publishing Vx, Vy and Wz
			odom_to_publish.twist.linear.x = vx;
			odom_to_publish.twist.linear.y = vy;
			odom_to_publish.twist.angular.z = w;
			odom_to_publish.header = old_header;
			odom_pub.publish(odom_to_publish);

			//Saving the current message in old_message
			for(int i = 0; i < 4; i++){
				old_position[i] = msg->position[i];
				old_velocity[i] = msg->velocity[i];
			}
			odom_to_publish.header = msg->header;
			old_header = msg->header;
			old_second = msg->header.stamp.sec;
			old_n_second = msg->header.stamp.nsec;
		}
	}

	bool calibrationCallback(project::calibration::Request  &req,
	        project::calibration::Response &res) {

			// callback of the calibration service
			this->r = req.r;
			this->lx = req.lx;
			this->ly = req.ly;
			this->n = req.encoder; 

			return true;
		}

private:
	ros::NodeHandle nh;
	ros::Subscriber odom_sub;
	ros::Publisher odom_pub;

	ros::ServiceServer calibration;

	std_msgs::Header old_header;
	double old_position[4];
	double old_velocity[4];
	int first_time = 0;
	//Robot Parameters in order: radius, wheel distance from center along x, wheel distance from center along y	
	double r = 0.078, lx = 0.2, ly = 0.171;
	int n = 42;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "velocity_computer");
	VelComputer vc;
	ros::spin();
	return 0;
}
