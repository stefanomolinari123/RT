#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/Twist.h"
#include "second_assignment/Speed_srv.h"
#include "second_assignment/Speed.h"

//definition of the size for the three arrays in the 'avoidCollision' function
#define SIZE 100

//definition of the publisher
ros::Publisher pub;

//definition of the variabile speed 
geometry_msgs::Twist vel;

//initial speed of the robot
float speed = 0;

//frontal threshold
float front_th = 1.5;

//function to calculate the minimun distance among array values
double min_value_of_dist(double array[])
{
	// setting dist at the maximux for the laser, to avoid errors
	double dist = 30;
	
	//test all the elements of the array
	for(int i=0; i < SIZE; i++)
	{
		//check if the value is less than the distance
		if(array[i] < dist)
			//update of the minimum distance value
			dist = array[i];
	}
	
	//return the minimun distance founded in the array
	return dist;
}

//avoidCollision function
void avoidCollision(const sensor_msgs::LaserScan::ConstPtr &msg)
{
	//initialization of the array a, with lenght equal to ranges' size
	float a[msg->ranges.size()];
	
	//initialization of the array a
	for(int i = 0; i < msg->ranges.size(); i++)
	{	
		a[i] = msg->ranges[i];
	}
	
	//creation of the three arrays, representing the right, the front and the left of the robot
	//each array has the same size of the others
	double left[SIZE];
	double front[SIZE];
	double right[SIZE];

	//left loop
	for(int i = 0; i < SIZE; i++)
	{
		left[i] = a[i+msg->ranges.size()-SIZE];
	}

	//front loop
	for(int i = 0; i < SIZE; i++)
	{
		front[i] = a[i+(msg->ranges.size()-SIZE)/2];
	}

	//right loop
	for(int i = 0; i < SIZE; i++)
	{
		right[i] = a[i];
	}

	//check if the robot is far from a wall
	if(min_value_of_dist(front) > front_th)
	{	
		//setting the speed according to the service response
		vel.linear.x = speed;
		vel.angular.z = 0;			
	}
	
	//check if the robot is close from a wall
	else if(min_value_of_dist(front) < front_th)
	{
		//check if the the right wall is closer than the left one
		if(min_value_of_dist(right) < min_value_of_dist(left))
		{
			//setting the speed to drive out the robot from the corner
			vel.linear.x = 0.5;
			vel.angular.z = 1;
		}
		
		//check if the the left wall is closer than the right one
		else if(min_value_of_dist(right) > min_value_of_dist(left))
		{
			//setting the speed to drive out the robot from a corner
			vel.linear.x = 0.5;
			vel.angular.z = -1;
		}
	}
	
	// publish the speed computed
	pub.publish(vel);
}

//speedHandler function
void speedHandler(const second_assignment::Speed::ConstPtr &sp)
{
    speed = sp->speed;
}

//main function
int main(int argc, char ** argv)
{
	//initialization of the node
	ros::init(argc, argv, "controller");
	
	//initialization of the node handle
	ros::NodeHandle nh;
	
	//advertise the topic /cmd_vel
	pub = nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
	
	//defining two subscribers to subscribe to the /speed and /base_scan topics to take infos
	ros::Subscriber sub = nh.subscribe("/base_scan", 100, avoidCollision);
	ros::Subscriber sub2 = nh.subscribe("/Speed", 100, speedHandler);
	
	//using this to spin the program
	ros::spin();
	return 0;
}
