#!/usr/bin/env python

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree
from std_msgs.msg import Int32

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
LOOKAHEAD_TRAFFIC_LIGHT = LOOKAHEAD_WPS*2
Speed_limit_m_s = 40*0.44704

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('current_velocity', TwistStamped, self.velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
		
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        self.base_waypoints = None
        self.waypoints_2d = None
        self.waypoint_tree = None
        self.current_vel_lin_x = None
        self.stop_line_wp_idx = -1
        self.stop_line_wp_idx = -1
        self.last_closest_wp = -1
        self.last_vel_lin_x_array = None
        self.last_avg_dec = 0.
        #print("----------Waypoint Tree None initialized-----------")
        self.loop()
        
    def loop(self):
        #print("self.loop started")
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.pose and self.base_waypoints and self.waypoint_tree:
                # Get closest waypoint
                #print("while started")
                closest_waypoint_idx = self.get_closest_waypoint_idx()
                self.publish_waypoints(closest_waypoint_idx)
                #print("closest_waypoint_idx:")
                #print(closest_waypoint_idx)
                rate.sleep()
                #print("while finished")
        #print "self.loop finished"

    def get_closest_waypoint_idx(self):
        #print("get_closest_waypoint_idx started") 
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        if not self.waypoint_tree is None:
            closest_idx = self.waypoint_tree.query([x,y],1)[1]
        #print "self.waypoint_tree:"
        #print self.waypoint_tree
        #print "self.waypoint_tree printed"
        #print "closest_idx: ".format(closest_idx)
        #print "car.x: {}, car.y: {}".format(x, y)
        # Check if closest is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx -1]
        #print "closest_coord:" 
        #print closest_coord
        #print "prev_coord:"
        #print prev_coord
        
        # Equation for hyperplane through closest coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array (prev_coord)
        pos_vect = np.array([x, y])
        
        val = np.dot(cl_vect - prev_vect, pos_vect - cl_vect)
        #print "dot product: {}".format(val)
        
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
            #print "closest_idx returned: {}".format(closest_idx)
        return closest_idx
        
        #print("get_closest_waypoint_idx finished")            
        
    def publish_waypoints(self, closest_idx):
        # rospy.loginfo("publish_waypoints started")
        lane = Lane()
        lane.header = self.base_waypoints.header
        if closest_idx is not None:
            lane.waypoints = self.base_waypoints.waypoints[closest_idx : closest_idx + LOOKAHEAD_WPS]
            #rospy.loginfo("Lenght of waypoints is: %.2f", len(lane.waypoints))
            if len(lane.waypoints) < LOOKAHEAD_WPS:
                rospy.logwarn("Generating points for new loop. This can make the car unstable for a while")
                new_loop_waypoints = self.base_waypoints.waypoints[0:LOOKAHEAD_WPS - len(lane.waypoints)]
                extended_waypoints = lane.waypoints + new_loop_waypoints
                lane.waypoints = extended_waypoints
            
            if self.stop_line_wp_idx > 1:
                stop_line_wp_idx = self.stop_line_wp_idx -2
                #rospy.loginfo("closest_idx: %.2f   stop_line_wp_idx: %.2f", closest_idx, stop_line_wp_idx)
            else:
                #rospy.loginfo("closest_idx: %.2f", closest_idx)
                stop_line_wp_idx = 9999999 # infinite

            if (self.stop_line_wp_idx >= 0) and (stop_line_wp_idx < (closest_idx + LOOKAHEAD_TRAFFIC_LIGHT)): 
                # Idx > 0 if upcoming traffic light = Red // otherwise it is not published or -1
                # If stop line idx within the planning horizon, then an action is needed
                
                # Calculate distance from current waypoint to stop line
                # try to stop 2 waypoints ahead of the stopline so that the vehicle does not overshoot
                distance_along_trace = self.distance(self.base_waypoints.waypoints, closest_idx, stop_line_wp_idx) 
                rospy.loginfo("Red traffic light ahead")
                              
                if self.current_vel_lin_x and distance_along_trace > 0.5: 
                    #Calculate average deceleration to stop in front of traffic light stop line
                    avg_dec =  self.current_vel_lin_x / distance_along_trace
                    rospy.loginfo("distance_along_trace: %.2f, self.current_vel_lin_x: %.2f, avg_dec: %.2f", distance_along_trace, self.current_vel_lin_x, avg_dec)
                else:
                    # No Stop Line available
                    avg_dec = 0
                    rospy.loginfo("distance_along_trace: %.2f, self.current_vel_lin_x: None, avg_dec: 0", distance_along_trace)
                
                if avg_dec < 0.5:
                    # As long as the average deceleration is low, the vehicle shall keep driving
                    do_not_brake = 1
                    rospy.loginfo("Acceleratio needed")

                
                # Set velocity for first waypoint. 
                # If acceleration needed: v_current_wp must be greater than current_vel_lin_x
                # If deceleration needed: v_current_wp must be smaller than current_vel_lin_x
                if self.current_vel_lin_x > 0.1 and self.last_vel_lin_x_array is not None:
                    # Vehicle is Moving Case
                    
                    # if one would take the current speed or a speed which is close to the waypoint,
                    # then the changes for the waypoint follower are to small. So take a waypoint a few indices
                    # further and set the current waypoint speed to that value. This serves as the target speed
                    # for twist controller, thus a significant delta to actual current speed is calculated, so that
                    # pure pursuit calculates /twist_cmd by new.
                    
                    if avg_dec >= 0.5:
                        # Moving vehicle but deceleration needed
                        if self.last_avg_dec < 0.5 and avg_dec > 0.5:
                            v_current_wp = self.current_vel_lin_x - 1
                            rospy.loginfo("Moving & Decel needed & decel trigger")
                        else:
                            if closest_idx == self.last_closest_wp:
                                v_current_wp = min(self.last_vel_lin_x_array[0], self.current_vel_lin_x)
                                rospy.loginfo("Moving & Decel needed & no decel trigger --> same wp idx ")
                            else:
                                delta_idx = closest_idx - self.last_closest_wp
                                v_current_wp = min(self.last_vel_lin_x_array[delta_idx], self.current_vel_lin_x)
                                rospy.loginfo("Moving & Decel needed & no decel trigger --> next wp idx ")
                                
                                
                            #v_current_wp = self.current_vel_lin_x - (0.5 * avg_dec)
                            #rospy.loginfo("Moving & Decel needed & no decel trigger")

                    else:
                        #Vehicle is moving and it shall keep moving
                        if self.last_vel_lin_x_array[0] < self.last_vel_lin_x_array[10]:
                            # Target waypoints already contain acceleration
                            v_current_wp = min(self.last_vel_lin_x_array[1], self.current_vel_lin_x + 0.3)
                            rospy.loginfo("Moving & shall keep moving & Last Target WP contained acceleration")
                        else:
                            # Target waypoints contain deceleration
                            v_current_wp = self.current_vel_lin_x + 0.1
                            rospy.loginfo("Moving & shall keep moving & Last Target WP contained deceleration")
                    #rospy.loginfo("v_current_wp: {}".format(v_current_wp))
                    self.last_avg_dec = avg_dec 
                        
                else:
                    # Vehicle in Standstill Case
                    if do_not_brake == 1 and distance_along_trace > 3:
                        # Acceleration needed
                        v_current_wp = 0.1
                        rospy.loginfo("Standstill & Acceleration needed")
                    else:
                        # Standstill state and it shall stay in this state
                        v_current_wp = 0
                        rospy.loginfo("Standstill & Standstill needed --> 0 m/s")
                    #rospy.loginfo("v_current_wp: 0")
                    
                #rospy.loginfo("self.current_vel_lin_x: %.2f", self.current_vel_lin_x)
                
                dist_seg_sum = 0
                planning_horizon_meters = self.distance(self.base_waypoints.waypoints, closest_idx, closest_idx + LOOKAHEAD_WPS) 
                iterator_idx = 0
                # Calculating a linear velocity reduction
                for current_wp_idx in range(0, LOOKAHEAD_WPS-1):
                    # Calculate distance between current waypoint and the waypoint
                    # of the current index
                    # try to stop 2 waypoints ahead of the stopline so that the vehicle does not overshoot
                    dist_seg_sum = self.distance(self.base_waypoints.waypoints, closest_idx, closest_idx+iterator_idx) 
                    
                    #rospy.loginfo("LOOKAHEAD_WPS: %.2f, current_wp_idx: %.2f, len(lane.waypoints): %.2f", LOOKAHEAD_WPS, current_wp_idx, len(lane.waypoints))
                    rospy.loginfo("i: %.2f, v_current_wp: %.4f, avg_dec: %.4f, dist_seg_sum: %.4f", current_wp_idx, v_current_wp, avg_dec, dist_seg_sum)
                    lane.waypoints[current_wp_idx].twist.twist.linear.x = v_current_wp
                    
                    
                    if avg_dec > 0.5: # and v_current_wp > 0.01:
                        # Calculate velocity reduction for each lookahead waypoint.
                        # Reduce velocity by amount of the particular waypoint distance to stop divided by
                        # the cars distance to stop line
                        #rospy.loginfo("planning_horizon_meters: %.2f, distance_along_trace: %.2f", planning_horizon_meters, distance_along_trace)
                        v_current_wp = max(0, lane.waypoints[0].twist.twist.linear.x -  lane.waypoints[0].twist.twist.linear.x * (dist_seg_sum/(min(planning_horizon_meters, distance_along_trace)-2)))
                        #rospy.loginfo("i: %.2f, v_current_wp: %.4f, avg_dec: %.4f, dist_seg_sum: %.4f", current_wp_idx, v_current_wp, avg_dec, dist_seg_sum)

                        lane.waypoints[current_wp_idx + 1].twist.twist.linear.x = v_current_wp
                        lane.waypoints[current_wp_idx + 1].twist.twist.linear.y = 0
                        #rospy.loginfo("current_wp_idx: %.2f, wp.twist.twist.linear.x: %.2f", current_wp_idx, lane.waypoints[current_wp_idx + 1].twist.twist.linear.x)
                    #elif avg_dec <= 0.5:
                        #v_current_wp = max(0, v_current_wp+((dist_seg_sum-last_dist_seg_sum)/(distance_along_trace)*10*0.44704))
                        #last_dist_seg_sum = dist_seg_sum
                        ##rospy.loginfo("i: %.2f, v_current_wp: %.4f, avg_dec: %.4f, dist_seg_sum: %.4f", current_wp_idx, v_current_wp, avg_dec, dist_seg_sum)
                        #lane.waypoints[current_wp_idx + 1].twist.twist.linear.x = v_current_wp
                        #lane.waypoints[current_wp_idx + 1].twist.twist.linear.y = 0
                    else:
                        if avg_dec > 0.2 and avg_dec < 0.5:
                            lane.waypoints[current_wp_idx + 1].twist.twist.linear.x = self.current_vel_lin_x
                            lane.waypoints[current_wp_idx + 1].twist.twist.linear.y = 0
                        else:
                            #lane.waypoints[current_wp_idx + 1].twist.twist.linear.x = 40*0.44704
                            v_current_wp = max(0, min(v_current_wp+0.05, Speed_limit_m_s))
                            lane.waypoints[current_wp_idx + 1].twist.twist.linear.x = v_current_wp
                            lane.waypoints[current_wp_idx + 1].twist.twist.linear.y = 0
                        
                    iterator_idx +=1
                    
            else: # traffic light state is not red or not within planning horizon, proceed with max velocity
                rospy.loginfo("Planning horizon free of red traffic lights")
                for wp in lane.waypoints:
                    wp.twist.twist.linear.x = Speed_limit_m_s
                    wp.twist.twist.linear.y = 0
           
            # Log velocity vector -- just for debugging purposes
            v_lin_x_array = []
            for wp in lane.waypoints:
                    v_lin_x_array.append(wp.twist.twist.linear.x)
            rospy.loginfo("v_lin_x_array {}".format(v_lin_x_array))
            
            # Update attributes and publish
            self.last_vel_lin_x_array = v_lin_x_array
            self.last_closest_wp = closest_idx
            self.final_waypoints_pub.publish(lane)
                
        else: 
            rospy.loginfo("closest_idx: None", )

        #for wp in lane.waypoints:
            #print(wp.twist.twist.linear.x)
        # rospy.loginfo("publish_waypoints finished")
        
    def pose_cb(self, msg):
        self.pose = msg
        
    def velocity_cb(self, msg):
        self.current_vel_lin_x = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        #print("waypoints_cb started")
        self.base_waypoints = waypoints
        if self.waypoints_2d == None:
            #print("----------Waypoint Tree BP1-----------")
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
            #print("self.waypoints_2d[1]:") 
            #print(self.waypoints_2d[1])
            #print("----------Waypoint Tree filled-----------")
        #print("waypoints_cb finished")      
              

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stop_line_wp_idx = msg.data
        #pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        #rospy.loginfo("distance started")
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        #rospy.loginfo("wp1: %.2f, wp2: %.2f", wp1, wp2)
        for i in range(wp1, wp2+1):
            #rospy.loginfo("wp1: %.2f, i: %.2f", wp1, i)
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        #rospy.loginfo("dist_along_trace: %.2f", dist)
        #rospy.loginfo("distance finished")
        return dist
    

if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
