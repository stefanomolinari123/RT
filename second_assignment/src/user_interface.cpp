#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Speed_srv.h"
#include "second_assignment/Speed.h"
#include "stdio.h"
#include "sensor_msgs/LaserScan.h"

//definition of the publisher
ros::Publisher pub;

//definition of the client
ros::ServiceClient client;


//setting speeds (linear and angular) to make the robot goes straight
float min_linear_vel = 0;
float max_linear_vel = 4;

//function that read the input insert by keyboard
void readInput(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//defining a variable srv
	second_assignment::Speed_srv srv;

	//defining a variable speed
	second_assignment::Speed speed;	
	
	char inputUsr;
	
	//show the menu
	std::cout << "\nMENU.";
	std::cout << "\n\nPress:\n-a: to accelerate;\n-d: to decelerate;\n-r: to reset the position;\n-q: to stop the program execution.\n";
	
	//getting the input by keyboard
	std::cin >> inputUsr;
	
	//if the input is correct procede with the creation of the server 
	if(inputUsr == 'a' || inputUsr == 'A' || inputUsr == 'd' || inputUsr == 'D' || inputUsr == 'r' || inputUsr == 'R' || inputUsr == 'q' || inputUsr == 'Q')
	{

	//put the input on the server request
	srv.request.input = inputUsr;

	//wait for the existence of the server
	client.waitForExistence();
	//call the server
	client.call(srv);

	speed.speed = srv.response.output;
	
	//check if the speed is not too low or too high
	if(min_linear_vel < speed.speed < max_linear_vel)
	{
		system("clear");
	}
	
	//makes that the max value of the speed is set and couldn't be infinite
	if(speed.speed > max_linear_vel)
	{
		speed.speed = max_linear_vel;
		system("clear");
		std::cout << "You can't increase the speed more than this.\n\n";
	}
	
	//avoid that the robot could goes backward	
	if(speed.speed < min_linear_vel)
	{
		speed.speed = min_linear_vel;	
       	system("clear");
		std::cout << "You can't decrease the speed less than 0.\n\n";
	}
		
	//publish the speed
	pub.publish(speed);
	std::cout << "Now the speed of the robot is: "<< speed.speed <<"\n\n";
	}
	
	//if the input is not correct don't create the server and saying to the user to use a valid 	 one
	else
   	{
	system("clear");
	std::cout <<"The input is invalid. Please try again.\n";
    	}
}

//main function
int main(int argc, char ** argv)
{
	//initialization of the node
	ros::init(argc, argv, "user_interface");
	
	//initialization of the node handle
	ros::NodeHandle nh;
		
	//advertise the topic /speed
	pub = nh.advertise<second_assignment::Speed>("/Speed",100);
	
	//call the service with the client
	client = nh.serviceClient<second_assignment::Speed_srv>("/Speed_srv");
	
	//defining subscriber to subscribe to the /base_scan topics to take infos
	ros::Subscriber subscriber = nh.subscribe("/base_scan", 100, readInput);
	
	//using this to spin the program
	ros::spin();
	
	return 0;
}
