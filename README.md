Research Track1 - Second Assignment
================================

Stefano Molinari 4703592

The second assignment consists on the development of a software program, written in ROS and C++ language, which drives a pre-programmed robot around an enviroment, represented in this assignment by the 'Autodromo Nazionale di Monza'.

The robot in its path must not collide with the walls.

The circuit is shown above:

![tracciato](https://user-images.githubusercontent.com/62506638/146203922-582116cc-449a-4adb-a4e8-a1ee5aa9ab36.png)

Running 
----------------------

To simulate the code that controls the behavior of the robot, you need to install operating system for robot on the pc: 
[ROS(Robot Operating System)](http://wiki.ros.org). In particoular, for this programm, we have to use the [Noetic Version of ROS](http://wiki.ros.org/noetic/Installation)

To run this script you have to do, for first the ```$ catkin_make``` on the root folder of ROS, in this case we used the folder called 'my_ros_ws'. 
After this, you have to communicate to the ROS master, using the command ```$ roscore &``` on the terminal. In this case it's used the '&' for launching it in background.

Then you should run the following commands, one for console:

```console
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
This command will open the enviroment shown before

```console
$ rosrun second_assignment controller_node
```
This command will run the controller node, which makes that the robot will go autonomously in the circuit, without avoids the wall

```console
$ rosrun second_assignment server_node
```
This command will run the service node, which makes that the user can increase or decrease the robot's speed. In this programm it's assumed that the minimum speed of the robot can be 0 (it can not goes backwards), while the maximum speed can be 4 (in order to avoid that the robot won't be so close to the wall and so it can't crash with these). The maximum and minimum possible speeds can be changed in the user_interfacce.ccp file.

```console
$ rosrun second_assignment user_interface_node
```
This command will run the user interface, which shows the menù. In this menù there are four choices:
-if you press 'a'(or 'A') the robot will increase its speed;
-if you press 'd'(or 'D') the robot will decrease its speed;
-if you press 'r'(or 'R') the robot will return to its initial position with speeds equal to 0;
-if you press 'q'(or 'Q') you will close the server node;

### Faster Running ###

A faster way to start the developed program is to run it with the [.launch file](https://github.com/stefanomolinari123/second_assignment/blob/main/launch/launch.launch), which is able to start the three cpp files, the file containing the track and to communicate with the ros master.
To run this launch file you need to type the following command on the terminal:
```console
$ roslaunch second_assignment launch.launch
```

[Controller Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/controller.cpp)
----------------------

Thanks to the controller node the robot is able to follow the desired path until there will be an interrupt by keyboard, which kills the server node, making the programm to close. 
With this node the robot is able to detect when there are no wall in front of it and so it can goes straight, and also when there is a wall in front of it and so it have to turn in the right way, defined by the further wall.
The "ability" to see where walls are, is due to the fact that the robot uses the sensors recieved by the /base_scan publisher after the subscribction on it. This topic is characterized by 720 sensors, which allow the robot to see obstacles and walls. Each sensor has 0.25 degress of view, so the robot can see from 0 (right) to 180 (left) degrees.

When a message from /base_scan is recieved, the avoidCollision function is called, and all the data found by the /base_scan should be in an array, declared in this function. These values are put in three array, and not all data are used:

-right array: uses the first 25 element of the /base_scan, so its view is from 0 to 25 degrees;
-front array: uses the central 25 element of the /base_scan, so its view is from 77.5 to 102.5 degrees;
-left array: uses the last 25 element of the /base_scan, so its view is from 155 to 180 degrees.

Now the avoidCollision function checks for the minimum value in each of the three arrays, and thanks to this should know which is the action to be done to follow the path as desired:
-if the wall is far from the robot current position it have to go straight;
-else if the robot is close to a wall, it checks which wall is closer:
 -if the left one is closer than the right one it have to turn right;
 -obviously, if the right one is closer it have to turn left.

When no wall are detected in front of the robot it have to go straight, using the /Speed value as speed value. This value is managed by the user_interface node, which set its speed. The speed can be increased until it will reach the max_linear_vel, value set in the [user_interface.cpp](https://github.com/stefanomolinari123/second_assignment/blob/main/src/user_interface.cpp).

After this, the data will be publish to the /cmd_vel topic by the controller node, in order to control the robot movement in the desired way.

[Server Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/server.cpp)
----------------------

The server node control and modify the speed of the robot, following the input recived by the user in thw user_interface node. This node, indeed, recivied the input insert in the user_interface node, and thanks to a switch function, in which there is a check if the input is correct, and following this the robot's speed can be incremented or decremented. Furthermore, the position of the robot and its speed can be resetted, thanks to the /reset_position service. This node can be also closed when the referring command is pressed.

[User Interface Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/user_interface.cpp)
----------------------
The user interface node shows to the user the menù of the possible command that can be give to the robot. Indeed, as previous said, is possible to increase or decrease the speed of the robot, resetting its position and close the service node. 
The keyboard command is recieved in this node, it is ridden and, if it is correct, it is send to the service node, which modify the speed of the robot or reset its position according to the input. After the modification of the robot's speed the new speed is printed in this interface. 

Graph Project
----------------------
Thanks to the command ```$ rqt_graph``` is possible to see the graph of the project, which shows the relationship between the nodes of the programm, showing so the structure communication.

Below there is the graph of this project shown:

![rosgraph](https://user-images.githubusercontent.com/62506638/146358604-6cd983ab-303d-41bd-af60-e9450ca25376.png)

Future Improvements
----------------------

In certain situation is possible to see that the driving of the robot is not so fluent, like a zig-zag drives, this is due to the fact that the turning method used is no optimal. Also the fact that the speed of the robot cannot be higher than a certain value, otherwise the robot could be avoid the wall. With the use of more controls that can be avoided. 
Indeed in this project the speed of the robot is limitated from 0 to 4, and if you try to increase the speed after the robot has reached the maximum value this doesn't change. And when the robot is stopped you can't decrease its speed, otherwise it will go backward. The maximum value of the speed is set at 4, because if it goes over this value there could be the possibility that it avoid some walls.
