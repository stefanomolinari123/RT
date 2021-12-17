Research Track1 - Second Assignment
================================

Stefano Molinari 4703592

The second assignment consists on the development of a software program, written in ROS and C++ language, which drives a pre-programmed robot around an environment, represented in this assignment by the 'Autodromo Nazionale di Monza'.

The robot in its path must not collide with the walls.

The graph is shown below:

![tracciato](https://user-images.githubusercontent.com/62506638/146203922-582116cc-449a-4adb-a4e8-a1ee5aa9ab36.png)

Running
----------------------

To simulate the code that controls the behaviour of the robot, you need to install operating system for robot on the pc:
[ROS(Robot Operating System)](http://wiki.ros.org). In particular, for this program we have to use the [Noetic Version of ROS](http://wiki.ros.org/noetic/Installation)

To run this script you must do, first the ```$ catkin_make``` on the root folder of ROS, to build your workspace.
After this, you must communicate to the ROS master, using the command ```$ roscore &``` on the terminal. In this case it is used the '&' for launching it in background.

Then, you should run the following commands, one for console:

```console
$ rosrun stage_ros stageros $(rospack find second_assignment)/world/my_world.world
```
This command will open the environment shown before

```console
$ rosrun second_assignment controller_node
```
This command will run the controller node, which will make the robot to go autonomously in the circuit, without crashing the wall

```console
$ rosrun second_assignment server_node
```
This command will run the service node, that will allow the user to increase or decrease the robot's speed. In this program it is assumed that the minimum speed of the robot can be 0 (it cannot go backwards), while the maximum speed 4, to avoid that the robot will be too close to the wall and crash with it. The maximum and minimum possible speed can be changed in the user_interfacce.ccp file.

```console
$ rosrun second_assignment user_interface_node
```
This command will run the user interface, which will show the menu. In this menu there are four choices:

- if you press 'a'(or 'A') the robot will increase its speed;
- if you press 'd'(or 'D') the robot will decrease its speed;
- if you press 'r'(or 'R') the robot will return to its initial position with speeds equal to 0;
- if you press 'q'(or 'Q') you will close the server node;

### Running with Launch File ###

A faster way to start the developed program is to run it with the [.launch file](https://github.com/stefanomolinari123/second_assignment/blob/main/launch/launch.launch), which can start the three cpp files. This file holds the circuit, and it allows it to communicate with the ros master.
To run this launch file, you need to type the following command on the terminal:

```console
$ roslaunch second_assignment launch.launch
```

[Controller Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/controller.cpp)
----------------------

Thanks to the controller node the robot can follow the desired path. There could be an interruption by the keyboard, which will kill the server node, making the program to collapse.
With this node the robot is able to detect when there are no walls in front of it, so it can go straight. When there is a wall in front of the robot, this node allows it to turn to the right way, defined by the further wall.
The sensors give the robot the "ability" to see where the walls are, received by the /base_scan publisher after the subscription on it. This topic is characterized by 720 sensors, which allows the robot to see obstacles and walls. Each sensor has 0.25 degrees of view, which allows the robot to see from 0 (right) to 180 (left) degrees.

When a message from /base_scan is received, the avoidCollision function is called into account and all the data that are found by the /base_scan should be in an array, declared in this function. These values are put in three arrays, and not all data are used:

- Right array: uses the first 25 element of the /base_scan, so its view is from 0 to 25 degrees;
- Front array: uses the central 25 element of the /base_scan, so its view is from 77.5 to 102.5 degrees;
- Left array: uses the last 25 element of the /base_scan, so its view is from 155 to 180 degrees.

Now the avoidCollision function checks for the minimum value in each of the three arrays and thanks to this it can know which path to follow, as desired:

```c++
If the wall is far from the robot current position:
  it must go straight.

Instead, if the robot is close to a wall, it checks which wall is closer:
  if the left one is closer than the right one, the robot must turn right;
  obviously, if the right one is closer, it must turn left.
 ```

When no wall is detected in front of the robot it must go straight, using the /Speed value as speed value. This value is managed by the user_interface node, which set its speed. The speed can be increased until it will reach the max_linear_vel, value set in the [user_interface.cpp](https://github.com/stefanomolinari123/second_assignment/blob/main/src/user_interface.cpp).

After this, the data will be published to the /cmd_vel topic by the controller node, to control the robot movement in the desired way.

[Server Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/server.cpp)
----------------------

The server node control and modify the speed of the robot, following the input received by the user in the user_interface node. This node, indeed, received the input inserted in the user_interface node, and thanks to a switch function, the robot's speed can be increased or decreased (always in the range defined in the user_interface node). Furthermore, the position of the robot and its speed can be resetted, thanks to the /reset_position service. This node can also be closed when the referring command ('q') is pressed.

[User Interface Node](https://github.com/stefanomolinari123/second_assignment/blob/main/src/user_interface.cpp)
----------------------
The user interface node shows to the user the menu of the possible command that can be given to the robot. As previously stated, it is possible to increase or decrease the speed of the robot, resetting its position and close the service node.
The keyboard command is received in this node. After this, it is read and, if it is correct, it is sent to the service node, which modifies the speed of the robot or resets its position according to the input. After the modification of the robot's speed, the new speed is printed in this interface.

Graph Project
----------------------
Thanks to the command ```$ rqt_graph``` it is possible to see the graph of the project, which shows the relationship between the nodes of the program, showing the structure communication.

Below there is the graph of this project shown:

![rosgraph](https://user-images.githubusercontent.com/62506638/146358604-6cd983ab-303d-41bd-af60-e9450ca25376.png)

Possible Future Improvements
----------------------

In certain situation, it is possible to see that the driving of the robot is not so fluent, like a zig-zag drives, since the turning method used is not optimal. Besides, the speed of the robot cannot be increased more than a certain level, because if it would be possible, the robot could crash against the walls. This problem could be eliminated with the use of more controls.

Indeed, in this project the speed of the robot is limited from 0 to 4, and if you try to increase the speed after the robot has reached the maximum value, this won’t change. When the robot is stopped you can't decrease its speed, otherwise it will go backward. The maximum value of the speed is set at 4, because if it goes over this value there could be the possibility that it would not be able to avoid some walls and if not, it would crash against them.
