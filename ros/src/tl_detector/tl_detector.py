#!/usr/bin/env python
import rospy
from math import sqrt
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3
TRAFFIC_LIGHT_SEARCH_RANGE = 300 # Parameter used to search traffic lights only within a short distance 
                                 # to the car in order to make the code faster


class TLDetector(object):
    def __init__(self):
        rospy.loginfo("Init started")

        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.waypoint_tree = None
        self.waypoints_2d = None

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier(self.config['is_site'])
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        rospy.loginfo("Init finished")

        self.loop()

    def loop(self):
        rate = rospy.Rate(5)
        while not rospy.is_shutdown(): 

            light_wp, new_state = self.process_traffic_lights()

            '''
            Publish upcoming red lights at camera frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            if self.state != new_state: # or self.last_wp is -1:
                self.state_count = 0
                self.state = new_state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                if new_state != TrafficLight.RED:
                    light_wp = -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            rate.sleep()

    def pose_cb(self, msg):
        #rospy.loginfo("pose_cb started")
        self.pose = msg
        #rospy.loginfo("pose.x: %.2f, pose.y: %.2f", self.pose.pose.position.x, self.pose.pose.position.y)
        #rospy.loginfo("pose_cb finished")

    def waypoints_cb(self, waypoints):
        #rospy.loginfo("waypoints_cb started")
        self.waypoints = waypoints
        if self.waypoints_2d == None:
            #print("----------Waypoint Tree BP1-----------")
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
        #rospy.loginfo("waypoints_cb finished")

    def traffic_cb(self, msg):
        #rospy.loginfo("traffic_cb started")
        self.lights = msg.lights
        #rospy.loginfo("traffic_cb finished")

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        #rospy.loginfo("image_cb started")
        
        #self.has_image = True
        self.camera_image = msg
        #rospy.loginfo("image_cb finished")

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
        #rospy.loginfo("get_closest_waypoint started")
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]
        #rospy.loginfo("closest_idx: %.2f", closest_idx)
        #rospy.loginfo("get_closest_waypoint finished")
        return closest_idx

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)
        """
        if self.camera_image is None:
            rospy.logwarn("No image from the camera received. Using state to classify images")
            return light.state
        else: 
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "passthrough")
            state_classifier = self.light_classifier.get_classification(cv_image)        
            return state_classifier
        
    def traffic_light_on_range(self, car_x, car_y, light_x, light_y):
        """
        Verifies that the traffic light is within the search range in
        order to spare processing resources
        """
        dist = sqrt((car_x - light_x)**2 + (car_y - light_y)**2)
        return dist < TRAFFIC_LIGHT_SEARCH_RANGE

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        
        #rospy.loginfo("process_traffic_lights started")
        closest_light = None
        stop_line_wp_idx = -1
        car_wp_idx = 0

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)
            #rospy.loginfo("car_wp_idx %.2f", car_wp_idx)
            temp_stop_line_wp_idx = -1
            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                # Get Sstop line waypoint index
                line = stop_line_positions[i]
                if self.traffic_light_on_range(self.pose.pose.position.x, self.pose.pose.position.y, line[0], line[1]): 
                    temp_stop_line_wp_idx = self.get_closest_waypoint(line[0], line[1])
                    # FInd closest stop line waypoint index
                    d = temp_stop_line_wp_idx - car_wp_idx
                    if d>=0 and d<diff:
                        diff = d
                        closest_light = light
                        stop_line_wp_idx = temp_stop_line_wp_idx
                    #rospy.loginfo("stop line i: %.2f, temp_stop_line_wp_idx: %.2f, car_wp_idx: %.2f, 
                    # d: %.2f, diff: %.2f, stop_line_wp_idx: %.2f", i, temp_stop_line_wp_idx,  
                    # car_wp_idx, d, diff, stop_line_wp_idx)

                    
        
        if closest_light:
            state = self.get_light_state(closest_light)
            rospy.loginfo("car_wp_idx: %.2f, tl_line_wp_idx: %.2f, tl_state: %.2f", car_wp_idx, stop_line_wp_idx, state)
            #rospy.loginfo("process_traffic_lights_finished")
            return stop_line_wp_idx, state
        
        """ COMMENTED FOR TESTING WITH REAL WORD BAG FILE
        rospy.loginfo("car_wp_idx: %.2f, tl_line_wp_idx: %.2f, tl_state: %.2f", car_wp_idx, -1.0, TrafficLight.UNKNOWN)
        rospy.loginfo("process_traffic_lights finished")
        """
        #return -1, TrafficLight.UNKNOWN
        
        
        ## ONLY USE WITH REAL TIME BAG FILE, AFTER TESTING DELETE
        state = self.get_light_state(0)
        return -1, state
        
if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
