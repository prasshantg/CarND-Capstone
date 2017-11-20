#!/usr/bin/env python
import rospy
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
import math

STATE_COUNT_THRESHOLD = 1

class TLDetector(object):
    tl_detector_init = False

    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

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
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb,queue_size=1, buff_size=2*52428800)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()

        use_real_tl_classifier = rospy.get_param('~tl_classifier_real')
        if use_real_tl_classifier:
            rospy.loginfo('"Real" TL Classifier DNN to load')
        else:
            rospy.loginfo('"Sim" TL Classifier DNN to load')

        self.light_classifier = TLClassifier(real = use_real_tl_classifier)
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.last_car_position = 0
        self.last_light_pos_wp = []
        self.FAR_LIGHT_DIST = 100.0
        self.tl_detector_init = True

        rospy.spin()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        if self.tl_detector_init == False:
            return

        self.has_image = True
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state is
        used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def distance(self, pos1, pos2):
        """Compute the distance between two positions
        Args:
            pos1: pose from
            pos2: pose to
        Returns:
            float: euclidean distance
        """
        diff_x = pos1.x - pos2.x
        diff_y = pos1.y - pos2.y
        return math.sqrt(diff_x * diff_x + diff_y * diff_y)


    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        closest_waypoint_id = None
        min_dist = None

        if self.waypoints is None:
            rospy.logwarn("tl_detector.get_closest_waypoint :: No waypoints yet")
            return

        waypoints = self.waypoints.waypoints
        for i, w in enumerate(waypoints):
            dist = self.distance(pose.position, w.pose.pose.position)
            if min_dist is None or dist < min_dist:
                min_dist = dist
                closest_waypoint_id = i
        return closest_waypoint_id

    def distance_light(self, lp, wp):
        """Compute the distance between light position and waypoint
        Args:
            lp: stop line position
            pos2: waypoint position
        Returns:
            float: euclidean distance
        """
        diff_x = lp[0] - wp.x
        diff_y = lp[1] - wp.y
        return math.sqrt(diff_x * diff_x + diff_y * diff_y)

    def get_closest_waypoint_light(self, wp, lp):
        """Identifies the closest path waypoint to the given stop line position.
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            light_pose (Pose): position to match a waypoint to
            way_point : Waypoint
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        closest_waypoint_id = None
        min_dist = None

        if wp is None:
            rospy.logwarn("tl_detector.get_closest_waypoint_light :: No waypoints yet")
            return

        waypoints = wp.waypoints
        for i, w in enumerate(waypoints):
            dist = self.distance_light(lp, w.pose.pose.position)
            if min_dist is None or dist < min_dist:
                min_dist = dist
                closest_waypoint_id = i
        return closest_waypoint_id

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']

        light_pos_wp = []
        if self.waypoints is not None:
            wp = self.waypoints
            for i in range(len(stop_line_positions)):
                l_pos = self.get_closest_waypoint_light(wp, stop_line_positions[i])
                light_pos_wp.append(l_pos)
            self.last_light_pos_wp = light_pos_wp
        else:
            light_pos_wp = self.last_light_pos_wp

        if (self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)
            if car_position is not None:
                self.last_car_position = car_position

        if (self.last_car_position > max(light_pos_wp)):
            light_num_wp = min(light_pos_wp)
        else:
            light_delta = light_pos_wp[:]
            light_delta[:] = [x - self.last_car_position for x in light_delta]
            try:
               light_num_wp = min(i for i in light_delta if i > 0) + self.last_car_position
            except ValueError:
                rospy.logwarn("Invalid Traffic light waypoint")
                return -1, TrafficLight.UNKNOWN

        light_idx = light_pos_wp.index(light_num_wp)
        light = stop_line_positions[light_idx]

        light_distance = self.distance_light(light, self.waypoints.waypoints[self.last_car_position].pose.pose.position)
        print("Test Code")
        #rospy.loginfo('light_distance:: {}'.format(light_distance))

        if light:
            if (light_distance >= self.FAR_LIGHT_DIST):
                return -1, TrafficLight.UNKNOWN
            else:
                state = self.get_light_state(light)
                return light_num_wp, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
