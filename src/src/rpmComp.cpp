#include "ros/ros.h"
#include "geometry_msgs/TwistStamped.h"
#include "project/custom.h"

class rpmComputer{
public:
	rpmComputer(){
		rpm_sub = n.subscribe("/cmd_vel", 1000, &rpmComputer::Callback, this);
		rpm_pub = n.advertise<project::custom>("/wheels_rpm", 1000);
	}
	void Callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
		double w_rpm [4];
		double tmp;

		w_rpm[0] = ((msg->twist.linear.x - msg->twist.linear.y) - ((lx + ly) * msg->twist.angular.z)) / r;
		w_rpm[1] = ((msg->twist.linear.x + msg->twist.linear.y) + ((lx + ly) * msg->twist.angular.z)) / r;
		w_rpm[2] =  ((msg->twist.linear.x + msg->twist.linear.y) - ((lx + ly) * msg->twist.angular.z)) / r;
		w_rpm[3] = ((msg->twist.linear.x - msg->twist.linear.y) + ((lx + ly) * msg->twist.angular.z)) / r;

		for(int i = 0; i < 4; i++){
			w_rpm[i] = w_rpm[i] * 60 * T;
		}
		
		//wheel speed in RPM
		wheels_msg.header = msg->header;
		wheels_msg.rpm_fl = w_rpm[0];
		wheels_msg.rpm_fr = w_rpm[1];
		wheels_msg.rpm_rl = w_rpm[2];
		wheels_msg.rpm_rr = w_rpm[3];

		rpm_pub.publish(wheels_msg);
	}

private: 
	ros::NodeHandle n;
	ros::Subscriber rpm_sub;
	ros::Publisher rpm_pub;
	project::custom wheels_msg;
	double r = 0.078, lx = 0.2, ly = 0.171, T = 5;
};

int main(int argc, char **argv){
	ros::init(argc, argv, "rpm_computer");
	rpmComputer rc;
	ros::spin();
	return 0;
}