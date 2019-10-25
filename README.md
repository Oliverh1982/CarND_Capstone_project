# Self driving car nanodegree

## Capstone project: Programming a real self driving car

This project consist on the integration of different systems in order to drive an autonomous car first on simulation and then on the real world, the real autonomous car is named Carla. Carla will drive on a test track used by Udacity in California, USA.

The software packages are programmed in Python 3 and C++11 using the libraries Scipy, Tensorflow, OpenCV, and others. In order to integrate all software packages, build, debug and run the system the middleware ROS was used, specifically the Kinetic version. For the simulation, the [Udacity system integration simulator](https://github.com/udacity/CarND-Capstone/releases) was used.

This project was done on a group of three people: Jorge Rilling, Martin Sommer and Chia-Hao Huang.

### About the software structure
The system consits on 4 smaller subsystems: Perception subsystem, planning subsystem, control subsystem and hardware (simulator or real car). From the ros perspective the system is built with 7 ros nodes and more than 15 ros topics that make the communication between the nodes. 

The following image ilustrates the subsystems, nodes and the connection between them through topics:

![](./ReportImages/final-project-ros-graph-v2.png) 

#### Control subsystem
This subsystem is the one closer to the hardware and is the one responsible of giving control commands to the car actuators. These commands are the steering angle, the throttle and the brake. It consists on two nodes, the DBW Node written by us in Python and the Waypoint Follower written by [Autoware](https://www.autoware.org/) in C++.

The DBW node uses values of linear speed and angular speed arround the Z axis (the one pointing to the sky) to get the steering angle, throttle and brake needed to make the car follow a given trajectory.

The waypoint follower node receives a list of waypoints with calculated linear and angular velocities that represent a trajectory and with them calculates the needed linear and angular velocities for a given time step. These linear and angular velocities are the ones used by the DBW node.

#### Planning subsystem
This subsystem is the one responsible of defining the trajectory the car will follow on the next cycles. This trajectory is then given to the waypoint follower node of the control subsystem in the form of waypoints. This subsystem consist on two nodes, both written in Python, the waypoint loader node written by Udacity and the waypoint updater node written by us.

The waypoint loader determines all the valid waypoints the car can drive on and send them to be used by the waypoint updater node.

The waypoint updater node determines the trajectory the car will follow on the next cycles using the base waypoints sent by the waypoint loader node. For that it does not only determine which waypoints the car will follow, but also determines the linear and angular velocities the car will need on those points. Then the updated waypoints are sent to the waypoint follower node.

#### Perception subsystem
This subsystem is the one responsible of determining changes on the trajectory based on what the car senses. In the case of this project, the test track on the simulator and the real world site have only a traffic light, so the subsystem consists only on a traffic light detection node which was written by us in Python.

The traffic light detection node uses the base waypoints sent by the waypoint loader node, the current position of the car and an image from a camera attached to the car in order to determine the first waypoint the car will encounter on its trajectory where a red traffic light is located. This information is then used by the waypoint updater node to reduce the speed of the waypoints until it reaches 0 for the waypoint where the stop line for the traffic light is located.

### About the DBW (Drive by Wire) Node
As explained before, the task of this package is to convert the desired linear and angular speeds to the commands needed for the physical actuators. This node works with a group of PID controllers and a specialized steering controller which uses the physical properties of the car. This node runs with a frequency of 50 Hz and cannot be changed since it alters the work of the PID controllers.

#### Control of linear velocity
The linear velocity is controlled using two PID controllers, one for the throttle and one for the brake. The control strategy is relatively simple, the throttle PID controller is used when the target linear velocity is higher or equal than the current speed and the brake controller on the contrary case. In the case the car is stopped and that condition is wanted (if target and current linear velocities are near to 0) a constant brake torque is applied that avoids the car to roll. The following diagram ilustrates this control loop:

![](./ReportImages/Control_diagram_lin_speed.png)

#### Control of angular velocity
The target angular velocity is the one needed so the car stays on the trajectory defined by the waypoints and it is controlled using a combination between a specialized yaw controller and a PID controller. 

The specialized yaw controller uses some physical properties of the car like the lenght of the wheel base, the maximum lateral acceleration or a desired steer ratio in order to determine the steering angle. Using this controller alone, the car is capable of following the trajectory but with big oscillations. 

In order to make the car follow the trajectory without oscillations, the yaw controller is combined with a PID controller for the angular velocity using the following equation:

$$ st(av_{SP}, av_{PV}) = (1 + |PID_{out}(av_{SP}, av_{PV})|)\cdot YAW_{out}(av_{SP}, av_{PV}) (1)$$

Where:
$ st $: Steering angle [rad]
$ PID_{out} $: Output of PID controller
$ YAW_{out} $: Output of yaw controller [rad]
$ av_{SP} $: Setpoint of angular velocity [rad/s]
$ av_{PV} $: Current angular velocity [rad/s]

In the equation it can be seen that the yaw controller determines the sign of the steering while the PID controller makes this value bigger, making the control more exact.

The following diagram ilustrates the angular velocity control:

![](./ReportImages/Control_diagram_ang_speed.png)


The linear speed SP and angular speed SP (Setpoint) are received from the waypoint follower node within the topic "twist_cmd". The throttle, brake and steering commands are sent to the car hardware interface software or to the simulator through the topics "vehicle/throttle_cmd", "vehicle/brake_cmd" and "vehicle/steering_cmd".

### About the waypoint updater node
This is the node responsible of creating the trajectory the car will follow on the next cycles. This trajectory consists on a list of waypoints with a given position, linear velocity and angular velocity. In order to generate the trajectory, the base waypoints that correspond to all the possible waypoints the car can be in on the map are used.

The first step in order to generate the trajectory is to determine the closest waypoint to the car, and then select the next 50 waypoints ahead to the position of the car. The closest waypoint is found by using KDTree.query in order to do this efficiently. It needs to be checked if the closest waypoint is ahead of the car, if not, the closest waypoint ahead can be found at the next index. 

After having selected the waypoints, it is searched within the next 100 waypoints if there is a red traffic light. As the vehicles location is determined at the middle of the car, two waypoints earlier are assigned to the stop line in order to avoid overshooting. In the case that a red traffic light is found and the car is moving, the distance to stop line gets calculated and the speed is reduced linearly. 

A required average decelaration rate is calculated to decrease speed. In case the vehicle is really slow or is currently in stand still or too far ahead of the stop line, this deceleration rate is small. The speed gets reduced as soon as the deceleration rate is bigger then 0.5 m/s², which is found to be an appropriate value to brake smoothly. As the waypoint follower needs a certain deviation to the current linear velocity to start braking, the velocity for the first waypoint is reduced more than for the following waypoints. 

Basically the following states can be outlined:
1.	Vehicle Moving

1.1.	Deceleration needed

1.2.	Keep Moving -> acceleration required

2.	Vehicle in Standstill

2.1.	Acceleration needed (e.g. at simulation begin)

2.2.	Keep stopping

With regards to the desired state, which can be determined by evaluating the current speed, the required deceleration rate and the distance to stop line, the target speeds for the upcoming waypoints can be calculated.


### About the traffic light detection node
The tasks for this package were broken into two parts. In the first part, we need to implement the tl_detector.py module. The walkthrough section gives enough details to implement this module. What is not mentioned in the walkthrough code is the second part, to build a traffic light classifier. Most people used the tensorflow object dection API for this project. There is a very good reference from Alex Lechner at https://github.com/alex-lechner/Traffic-Light-Classification. It gives a detailed tutorial on how to build a traffic light classifier in this project. I followed the same methodoligy to test a couple of pre-trained models in the tensowflow library.

I end up using the SSD Inception V2 model for this project. Two seperate models are trained for simulator and real-world testing. Both models were trained for 20,000 steps.

The performance is good for simullator, here is an example:

![](./ReportImages/TL_Result_Sim.png)

Althought the model performs good for simulator, but it didn't perform good enough for real world test. To improve the performance, Gamma correction was used to enhance too bright images.

Here are some examples:

![](./ReportImages/TL_RealWorldResult_1.png)

![](./ReportImages/TL_RealWorld_Result2.png)
