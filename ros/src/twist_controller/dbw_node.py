#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from dbw_mkz_msgs.msg import ThrottleCmd, SteeringCmd, BrakeCmd, SteeringReport
from geometry_msgs.msg import TwistStamped
import math

from twist_controller import Controller

'''
You can build this node only after you have built (or partially built) the `waypoint_updater` node.

You will subscribe to `/twist_cmd` message which provides the proposed linear and angular velocities.
You can subscribe to any other message that you find important or refer to the document for list
of messages subscribed to by the reference implementation of this node.

One thing to keep in mind while building this node and the `twist_controller` class is the status
of `dbw_enabled`. While in the simulator, its enabled all the time, in the real car, that will
not be the case. This may cause your PID controller to accumulate error because the car could
temporarily be driven by a human instead of your controller.

We have provided two launch files with this node. Vehicle specific values (like vehicle_mass,
wheel_base) etc should not be altered in these files.

We have also provided some reference implementations for PID controller and other utility classes.
You are free to use them or build your own.

Once you have the proposed throttle, brake, and steer values, publish it on the various publishers
that we have created in the `__init__` function.

'''

class DBWNode(object):
    def __init__(self):
        rospy.loginfo("DBW init started")
        rospy.init_node('dbw_node')
        
        vehicle_mass = rospy.get_param('~vehicle_mass', 1736.35)
        fuel_capacity = rospy.get_param('~fuel_capacity', 13.5)
        brake_deadband = rospy.get_param('~brake_deadband', .1)
        decel_limit = rospy.get_param('~decel_limit', -5)
        accel_limit = rospy.get_param('~accel_limit', 1.)
        wheel_radius = rospy.get_param('~wheel_radius', 0.2413)
        wheel_base = rospy.get_param('~wheel_base', 2.8498)
        steer_ratio = rospy.get_param('~steer_ratio', 14.8)
        max_lat_accel = rospy.get_param('~max_lat_accel', 3.)
        max_steer_angle = rospy.get_param('~max_steer_angle', 8.)
        
        speed_limit = rospy.get_param('waypoint_loader/velocity', 0.0)
        rospy.loginfo("speed_limit: %.2f", speed_limit)

        self.steer_pub = rospy.Publisher('/vehicle/steering_cmd',
                                         SteeringCmd, queue_size=1)
        self.throttle_pub = rospy.Publisher('/vehicle/throttle_cmd',
                                            ThrottleCmd, queue_size=1)
        self.brake_pub = rospy.Publisher('/vehicle/brake_cmd',
                                         BrakeCmd, queue_size=1)

        # TODO: Create `Controller` object
        # self.controller = Controller(<Arguments you wish to provide>)
        self.controller = Controller(vehicle_mass=vehicle_mass,
                                     fuel_capacity=fuel_capacity,
                                     brake_deadband=brake_deadband,
                                     decel_limit=decel_limit,
                                     accel_limit=accel_limit,
                                     wheel_radius=wheel_radius,
                                     wheel_base=wheel_base,
                                     steer_ratio=steer_ratio,
                                     max_lat_accel=max_lat_accel,
                                     max_steer_angle=max_steer_angle,
                                     speed_limit=speed_limit)
        # TODO: Subscribe to all the topics you need to
        rospy.Subscriber('/vehicle/dbw_enabled', Bool, self.dbw_enabled_cb)
        rospy.Subscriber('/twist_cmd', TwistStamped, self.twist_cb)
        rospy.Subscriber('current_velocity', TwistStamped, self.velocity_cb)
        rospy.Subscriber('brake_against_creep', Bool, self.brake_against_creep_cb)
        
        self.current_vel_lin_x = 0.0
        self.current_vel_ang_z = 0.0
        self.target_vel_ang_z = 0.0
        self.target_vel_lin_x = 0.0
        self.dbw_enabled = 0
        self.linear_vel = 0.0
        self.angular_vel = 0.0
        self.timestamp = rospy.get_time()
        self.throttle = self.steering = self.brake = 0.0
        self.brake_against_creep_enabled = False
        # rospy.loginfo("DBW init finished")
        self.loop()

    def loop(self):
        rospy.loginfo("Loop function started")
        rate = rospy.Rate(50) # 50Hz
        while not rospy.is_shutdown():
            rospy.loginfo("While Loop started")
            # TODO: Get predicted throttle, brake, and steering using `twist_controller`
            # You should only publish the control commands if dbw is enabled
            # throttle, brake, steering = self.controller.control(<proposed linear velocity>,
            #                                                     <proposed angular velocity>,
            #                                                     <current linear velocity>,
            #                                                     <dbw status>,
            #                                                     <any other argument you need>)
            # if <dbw is enabled>:
            #   self.publish(throttle, brake, steer)
            
            # Get current timestamp and calculate delta time   
            current_timestamp = rospy.get_time()
            delta_t = current_timestamp - self.timestamp           
            #rospy.loginfo("current_timestamp: %.2f, last_timestamp: %.2f, delta_t: %.2f", current_timestamp, self.timestamp, delta_t)
            self.timestamp = current_timestamp
            
            # Just execute controller and publish if Drive by Wire is enabled
            if self.dbw_enabled:
                #rospy.loginfo("If dbw_enabled entered")
                self.throttle, self.brake, self.steering = self.controller.control(self.current_vel_lin_x,
                        self.current_vel_ang_z, 
                        self.target_vel_lin_x,
                        self.target_vel_ang_z,
                        self.dbw_enabled,
                        self.brake_against_creep_enabled,
                        delta_t)
                self.publish(self.throttle, self.brake, self.steering)
                #rospy.loginfo("If dbw_enabled finished")
                
            rate.sleep()
            #rospy.loginfo("While Loop end")
        #rospy.loginfo("Loop function finished")
            
    def dbw_enabled_cb(self, msg):
        #rospy.loginfo("DBW_cb started")
        self.dbw_enabled = msg
        #rospy.loginfo("DBW_cb finished")
        
    def brake_against_creep_cb(self, msg):
        self.brake_against_creep_enabled = msg
        #rospy.loginfo("self.brake_against_creep_enabled: %b", self.brake_against_creep_enabled)
        
    def twist_cb(self, msg):
        #rospy.loginfo("Twist_cb started")
        self.target_vel_lin_x = msg.twist.linear.x
        self.target_vel_ang_z = msg.twist.angular.z
        #rospy.loginfo("target_vel_lin_x: %.2f, target_vel_ang_z: %.2f", self.target_vel_lin_x, self.target_vel_ang_z)
        #rospy.loginfo("Twist_cb finished")
        
    def velocity_cb(self, msg):
        #rospy.loginfo("Velocity_cb started")
        self.current_vel_lin_x = msg.twist.linear.x
        self.current_vel_ang_z = msg.twist.angular.z
        #rospy.loginfo("current_vel_lin_x: %.2f, current_vel_ang_z: %.2f", self.current_vel_lin_x, self.current_vel_ang_z)
        #rospy.loginfo("Velocity_cb finished")
        
    def publish(self, throttle, brake, steer):
        # rospy.loginfo("Publish function started")
        # rospy.loginfo("CRTL Throttle : %.2f, Brake : %.2f, Steer : %.2f", throttle, brake, steer)
        
        tcmd = ThrottleCmd()
        tcmd.enable = True
        tcmd.pedal_cmd_type = ThrottleCmd.CMD_PERCENT
        tcmd.pedal_cmd = throttle
        self.throttle_pub.publish(tcmd)

        scmd = SteeringCmd()
        scmd.enable = True
        scmd.steering_wheel_angle_cmd = steer
        self.steer_pub.publish(scmd)

        bcmd = BrakeCmd()
        bcmd.enable = True
        bcmd.pedal_cmd_type = BrakeCmd.CMD_TORQUE
        bcmd.pedal_cmd = brake
        self.brake_pub.publish(bcmd)
        # rospy.loginfo("Publish function finished")


if __name__ == '__main__':
    DBWNode()
