#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "second_assignment/Speed_srv.h"
#include "ros/ros.h"

//definition of the variation for acceleration or deceleration
#define SPEEDVAR 0.5

//ros variable for resetting the position of the robot
std_srvs::Empty reset;

//initial speed of the robot
float speed = 0;

//function that change the speed of the robot according to the input
bool changeSpeed(second_assignment::Speed_srv::Request &req, second_assignment::Speed_srv::Response &res)
{
	switch(req.input)
	{
		//increase the speed of the robot
		case 'a':
		case 'A':
			speed += SPEEDVAR;
			break;
		
		//decrease the speed of the robot
		case 'd':
		case 'D':
			speed -= SPEEDVAR;
			break;
			
		//reset the position of the robot and its speed
		case 'r':
		case 'R':
			ros::service::call("/reset_positions", reset);
			speed = 0;
			break;
			
		//close the server node
		case 'q':
		case 'Q':
			ros::shutdown();
			break;
			
		//invalid input by keyboard
		default:
			std::cout << "Invalid input. Please choose one of the alternatives proposed";
			break;
	}
	
	//gives to the output of the server the value obtained by keyboard
	res.output = speed;
	
	return true;
}

//main function
int main(int argc, char ** argv)
{
	//initialization of the node
	ros::init(argc, argv, "server");
	
	//initialization of the node handle
	ros::NodeHandle nh;
	
	//advertise the service and call the function
	ros::ServiceServer service = nh.advertiseService("/Speed_srv", changeSpeed);
	
	//using this to spin the program
	ros::spin();
	
	return 0;
}
