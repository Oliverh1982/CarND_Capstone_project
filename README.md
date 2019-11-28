# Self driving car nanodegree

## Capstone project: Programming a real self driving car

This project consist on the integration of different systems in order to drive an autonomous car first on simulation and then on the real world using a real autonomous car named Carla. Carla will drive on a test track used by Udacity in California, USA.

The software packages are programmed in Python 3 and C++11 using the libraries Scipy, Tensorflow, OpenCV, and others. In order to integrate all software packages, build, debug and run the system the middleware Robot operating system (ROS) was used, specifically the Kinetic version. For the simulation, the [Udacity system integration simulator](https://github.com/udacity/CarND-Capstone/releases) was used that communicates with ROS using uWebSockets.

This project was done on a group of three people: 

Jorge Rilling (jorgea.rilling@gmail.com)

Martin Sommer (msomme10@ford.com)

Chia-Hao Huang (asamiyangyang@gmail.com)

### About the software structure
The system consits on 4 smaller subsystems: Perception subsystem, planning subsystem, control subsystem and hardware (simulator or real car). From the ros perspective the system is built with 7 ros nodes and more than 15 ros topics that make the communication between the nodes. 

The following image ilustrates the subsystems, nodes and the connection between them through topics:

![](./ReportImages/final-project-ros-graph-v2.png) 

#### Control subsystem
This subsystem is the one closer to the hardware and is the one responsible of giving control commands to the car actuators. These commands are the steering angle, the throttle and the brake. It consists on two nodes, the DBW Node written by us in Python and the Waypoint Follower written by [Autoware](https://www.autoware.org/) in C++.

The DBW node uses values of linear speed and angular speed arround the Z axis (the one pointing to the sky) to get the steering angle, throttle and brake needed to make the car follow a given trajectory.

The waypoint follower node receives a list of waypoints with calculated linear and angular velocities that represent a trajectory and with them calculates the needed linear and angular velocities for a given time step. These linear and angular velocities are the ones used by the DBW node.

#### Planning subsystem
This subsystem is the one responsible of defining the trajectory the car will follow on the next cycles. This trajectory is then given to the waypoint follower node of the control subsystem in the form of waypoints. This subsystem consists on two nodes, both written in Python, the waypoint loader node written by Udacity and the waypoint updater node written by us.

The waypoint loader determines all the valid waypoints the car can drive on and send them to be used by the waypoint updater node.

The waypoint updater node determines the trajectory the car will follow on the next cycles using the base waypoints sent by the waypoint loader node. For that it does not only determine which waypoints the car will follow, but also determines the linear the car should drive on those points. Then the updated waypoints are sent to the waypoint follower node.

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

<a href="https://www.codecogs.com/eqnedit.php?latex=$$&space;st(av_{SP},&space;av_{PV})&space;=&space;(1&space;&plus;&space;|PID_{out}(av_{SP},&space;av_{PV})|)\cdot&space;YAW_{out}(av_{SP},&space;av_{PV})&space;(1)$$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?$$&space;st(av_{SP},&space;av_{PV})&space;=&space;(1&space;&plus;&space;|PID_{out}(av_{SP},&space;av_{PV})|)\cdot&space;YAW_{out}(av_{SP},&space;av_{PV})&space;(1)$$" title="$$ st(av_{SP}, av_{PV}) = (1 + |PID_{out}(av_{SP}, av_{PV})|)\cdot YAW_{out}(av_{SP}, av_{PV}) (1)$$" /></a>

Where:

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$&space;st&space;$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$&space;st&space;$" title="$ st $" /></a>: Steering angle [rad]
<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$&space;PID_{out}&space;$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$&space;PID_{out}&space;$" title="$ PID_{out} $" /></a>: Output of PID controller

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$&space;YAW_{out}&space;$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$&space;YAW_{out}&space;$" title="$ YAW_{out} $" /></a>: Output of yaw controller [rad]

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$&space;av_{SP}&space;$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$&space;av_{SP}&space;$" title="$ av_{SP} $" /></a>: Setpoint of angular velocity [rad/s]

<a href="https://www.codecogs.com/eqnedit.php?latex=\inline&space;$&space;av_{PV}&space;$" target="_blank"><img src="https://latex.codecogs.com/gif.latex?\inline&space;$&space;av_{PV}&space;$" title="$ av_{PV} $" /></a>: Current angular velocity [rad/s]

In the equation it can be seen that the yaw controller determines the sign of the steering while the PID controller makes this value bigger, making the control more exact.

The following diagram ilustrates the angular velocity control:

![](./ReportImages/Control_diagram_ang_speed.png)


The linear speed SP and angular speed SP (Setpoint) are received from the waypoint follower node within the topic "twist_cmd". The throttle, brake and steering commands are sent to the car hardware interface software or to the simulator through the topics "vehicle/throttle_cmd", "vehicle/brake_cmd" and "vehicle/steering_cmd".

### About the waypoint updater node
This is the node responsible for creating the trajectory the car will follow on the next cycles. This trajectory consists of a list of waypoints with a given position, linear velocity and angular velocity. In order to generate the trajectory, the base waypoints that correspond to all the possible waypoints the car can be in on the map are used.

The first step in order to generate the trajectory is to determine the closest waypoint to the car, and then select the next 100 waypoints ahead to the position of the car. The closest waypoint is found by using [KDTree.query](https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.spatial.KDTree.html) which is a very efficient algorithm to adress this problem. After having the closest waypoint found, it needs to be checked if it is ahead of the car. If not, the closest waypoint ahead can be found at the next index.

After having selected the waypoints, it is searched within the selected waypoints if there is a red traffic light. Since the vehicles location is determined at the middle of the car, two waypoints earlier are assigned as the stop line in order to avoid overshooting. In the case that a red traffic light is found and the car is moving, the distance to stop line gets calculated and the speed is reduced linearly. 

A required average deceleration rate is calculated to decrease speed. In case the vehicle is really slow or is currently in stand still or too far ahead of the stop line, this deceleration rate will be small. The speed gets reduced as soon as the deceleration rate is bigger than 0.5 m/s², which is found to be an appropriate value to brake smoothly. Since the waypoint follower needs a certain deviation to the current linear velocity to start braking, the velocity for the first waypoint is reduced more than for the following waypoints. 

In the case the vehicle is stopped and it is wanted to keep it stopped, a brake torque against creep needs to be applied due to the automatic transmision Carla has. Therefore in this case a bool signal is sent from the waypoint updater node to the DBW node through the topic "brake_against_creep".

Basically the following states can be outlined:
1.	Vehicle Moving

1.1.	Deceleration needed

1.2	Keep Moving -> acceleration required

2.	Vehicle in Standstill

2.1	Acceleration needed (e.g. at simulation begin)

2.2	Keep stopping

With regards to the desired state, which can be determined by evaluating the current speed, the required deceleration rate and the distance to stop line, the target speeds for the upcoming waypoints can be calculated.

### About the traffic light detection node
This node is the one responsible of finding where the next traffic light is located and what color it currently has. The node can be divided in two parts, traffic light detection (file ros/src/tl_detector/tl_detector.py) and traffic light classification (file ros/src/tl_detector/light_classification/tl_classifier.py), while the traffic light detection part is the main part of this node and it makes use of the traffic light classification part.

#### Traffic light detector
Similar to what the waypoint updater node does, the first step here is to determine what is the closest waypoint to the current position of the vehicle. After that, it is iterated through all the traffic lights on the map and calculated the distance from the vehicle to it. If the distance to a traffic light is smaller than a given range (in this case 300 m), the closest waypoint to that light is determined the same way the closest waypoint to the vehicle is detected.

After getting the closest traffic light, its status is determined using the traffic light classifier. Then, if the traffic light determined to be red or yellow, its closest waypoint index is sent to the waypoint updater node using the topic "traffic_waypoint". If no traffic light is detected or the closest traffic light is determined to be green, the closest waypoint index sent is -1. This information is used then by the waypoint updater to determine the linear velocity of the next trajectory waypoints.

#### Traffic light classifier
The traffic light classifier is big enough to be considered an independent part on the software structure. It detects and classifies traffic lights from camera pictures using a single shot detection (SSD) neural network and computer vision techniques. SSD neural networks are a good approach for this kind of problem since they can detect multiple objects on an image and perform well on real time problems. A very good resource to help understanding this kind of neural networks can be found [here](https://towardsdatascience.com/understanding-ssd-multibox-real-time-object-detection-in-deep-learning-495ef744fab).

In order to detect where the traffic lights are on the camera images the  pre-trained on the COCO dataset model "ssdlite_mobilenet_v2_coco" has been selected from the [Tensorflow detection model zoo](https://github.com/tensorflow/models/blob/master/research/object_detection/g3doc/detection_model_zoo.md). The model was selected based on the processing speed.

Once the model finds the traffic lights and provides the boundary boxes, the traffic light images are cropped from the scene based on those boxes and the color is identified. The steps done are the following:

1. Convert the image into LAB color space and isolate the L channel. Good support material for this can be found [here](https://www.learnopencv.com/color-spaces-in-opencv-cpp-python/)

2. Split the traffic light cropped image onto three equal segments - upper, middle, and lower corresponding to read, yellow, and green lights respectively.

3. To identify the color, it is needed to find out which segment is brighter. This can be done thanks to the LAB color space, since L channel gives exactly that information. All we need to do is to find the sum of all pixels in each of the three segments. The highest score gives us the traffic light color.

Althought the "ssdlite_mobilenet_v2_coco" performs good for simulator, it didn't perform good enough for real world test. To improve the performance, another model "ssd_mobilenet_v1_coco_2017_11_17" was used and was retrained on a dataset kindly shared [here](https://drive.google.com/file/d/0B-Eiyn-CUQtxdUZWMkFfQzdObUE/view). Also the repository of [coldKnight](https://github.com/coldKnight/TrafficLight_Detection-TensorFlowAPI) explaining how to re-train the models was used for this purpose. After training, the result was quite satisfactory. 

Here an example on the simulator can be seen:
![](./ReportImages/Sim_1.png)

And an example on the real world:
![](./ReportImages/Real_1.png)

### Results and conclusions
This project covers contents discussed along the nanodegree on individual projects like computer vision, deep learning, trajectory generation and control and puts everything together using ROS in order to control a real autonomous vehicle. Using ROS makes the integration of all different software modules easier, even if they are programmed on different languages like Python or C++.  

After implementing all software modules and putting everything together we achieved on getting the vehicle to drive very well on the simulator as it can be seen on the following video:

[![Video 1](https://i.imgur.com/ZVyDqN5.png)](https://youtu.be/JLUz2sKhGX4 "Video simulation") 

The traffic light detection was also tested using real life bag files, getting the traffic light status without errors as it can be seen on the following video:

[![Video 2](https://i.imgur.com/gdmpxFa.png)](https://youtu.be/eTK4t1wxpVc "Video real world") 

The biggest limitation encountered on this project was the high latency that can be generated when all the modules are running, specially the traffic light detection makes a big use of computing resources causing a big bottle neck if not well implemented or if the wrong model is selected. The simulator provided by Udacity also consumes a lot of resources, even using a powerfull computer and setting the graphics on the lowest detail level. The problem is that with high latency the system starts to not being able to run all commands on time and can cause driving errors like stopping a bit after a stop line on a red traffic light or even a complete loss of control of the vehicle.

### How to use
Firstly set your environment as explained on the original [Udacity's self driving car capstone project repository](https://github.com/udacity/CarND-Capstone). 

In particular for the development of this project a virtual machine with the following software was used:
- Operating system Ubuntu 16.04 Xenial Xerus
- Robot operating system (ROS) version Kinetic

After setting the environment clone this repository:

```bash
git clone https://github.com/j-rilling/SDCND_Capstone
```

Install python dependencies
```bash
cd SDCND_Capstone
pip install -r requirements.txt
```
Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```







