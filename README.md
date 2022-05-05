# Robotics: Project 1
## Group components:
- 10686087 Bruno Andrea
- 10723902 Cerutti Paolo

### Files description
- **parameters.cfg**: File containing the creation of the enumeration for dynamic parameter reconfiguration;
- **project.launch**: Sets the inital parameters and the statics transformations and it launches all nodes, rviz, rqt_service_caller, rqt_plot and rqt_reconfigure;
- **custom.msg**: contains the message structure required.
- **srv folder**
	- **calibration.srv**: defines the request structure in order to define the calibration service;
	- **paramsToGivenPose.srv**: defines the request structure in order to define the service that resets the odometry to any given pose.
- **src folder**
	- **velComp.cpp**: the node that reads from /wheel_states, computes velocity (with the formula adapted for encoder ticks) and publishes on /cmd_vel. It defines a service to calibrate the robot parameters (N, r, l, w);
	- **odomComp.cpp**: the node that reads from /cmd_vel, computes robot odometry and publishes on /odom. It defines a service to reset the odometry to any given pose. Adds the posibility to set dynamically the integration method (via **parameters.cfg**);
	- **tfBroadcaster.cpp**: the node that does a TF transformation: odom->base_link;
	- **rpmComp.cpp**: the node that reads from /cmd_vel, computes the wheels velocity in RPM and publishes on /wheels_rpm using a custom message defined in **custom.msg**.
- **additional_files folder**
	- **cfg.rviz**: configuration file to set rviz automatically to show the /robot/pose and /odom topics (with TF transformations).

### ROS Parameters description
- **parameters.cfg**: enum that contains 2 values: 
	- _EULER_: has index 0, used to choose Euler integration method;
	- _RUNGE_KUTTA_: has index 1, used to choose Runge-Kutta integration method.
- **calibration.srv**: 
	- _float64 r_: the wheels radius; 
	- _float64 lx_: wheel distance from center along x; 
	- _float64 ly_: wheel distance from center along y; 
	- _int 32_: encoder_: encoder resolution.
- **paramsToGivenPose.srv**:
	- _float64 x_: x initial coordinate;
	- _float64 y_: y initial coordinate;
	- _float64 theta_: theta initial coordinate.

### TF Tree structure
- world
	- odom
		- base_link
			- FrontLeftW
			- FrontRightW
			- RearLeftW
			- RearRightW

### Custom messages description
- **custom.msg**:
	- _Header header_: the header of the message;
	- _float64 rpm_fl_: contains the front left wheel velocity in rpm;
	- _float64 rpm_fr_: contains the front right wheel velocity in rpm;
	- _float64 rpm_rl_: contains the rear left wheel velocity in rpm;
	- _float64 rpm_rr_: contains the rear right wheel velocity in rpm.

### How to start / use nodes
Just run the project.launch file, this will launch all nodes and open the GUI to set parameters and call services. Additionally it will open rviz and the graph showing the difference from the /robot/pose and /odom topics. 

### Additional info
- In order for the graph to correctly show both topics it is necessary to start the bag at the same time as the launch file.
- To quickly and correctly display the elements in rviz it is possible to load cfg.rviz