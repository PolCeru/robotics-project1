#include "ros/ros.h"
#include "turtlesim/Pose.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>	
#include "nav_msgs/Odometry.h"

class Broadcaster {
public:
	Broadcaster(){
		sub = n.subscribe("/odom", 1000, &Broadcaster::callback, this);
	}
	
	void callback(const nav_msgs::Odometry::ConstPtr& msg){
		transformStamped.header.stamp = msg->header.stamp;
		transformStamped.header.frame_id = "odom";
		transformStamped.child_frame_id = "base_link";
		transformStamped.transform.translation.x = msg->pose.pose.position.x;
		transformStamped.transform.translation.y = msg->pose.pose.position.y;
		transformStamped.transform.translation.z = 0.0;

		tf2::Quaternion q;
		q.setRPY(0, 0, msg->pose.pose.orientation.z);
		transformStamped.transform.rotation.x = q.x();
		transformStamped.transform.rotation.y = q.y();
		transformStamped.transform.rotation.z = q.z();
		transformStamped.transform.rotation.w = q.w();
		br.sendTransform(transformStamped);
	}

private:
	ros::NodeHandle n;
	tf2_ros::TransformBroadcaster br;
	geometry_msgs::TransformStamped transformStamped;
	ros::Subscriber sub;
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "tf");
	Broadcaster broadcaster;
	ros::spin();
	return 0;
}