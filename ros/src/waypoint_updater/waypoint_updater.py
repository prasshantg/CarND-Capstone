#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose, TwistStamped
from styx_msgs.msg import Lane, Waypoint, TrafficLight

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

LOOKAHEAD_WPS = 150 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):

    base_waypoints = []
    last_waypoint = 0
    base_waypoints_received = False
    current_position_received = False
    final_wp_seq = 0
    traffic_light_wp = -1
    current_pose = Pose()
    orig_velocity = 0
    traffic_wp_start = -1

    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	# TODO: yet to implement trafic sign classifier
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=10)

        # TODO: Add other member variables you need below
        self.current_linear_velocity = 0

        self.loop()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_position_received = True
        self.current_pose = msg.pose
        self.publish_waypoints()

    def closest_waypoint(self):
        current_wp = self.last_waypoint
        min_dist = 10000
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        wp = current_wp
        for i in range(len(self.base_waypoints)):
            dist = dl(self.current_pose.position, self.base_waypoints[wp].pose.pose.position)
            if (dist < min_dist):
                current_wp = wp
                min_dist = dist
            if current_wp != wp:
                break
            if wp == (len(self.base_waypoints) - 1):
                wp = 0
            else:
                wp = wp + 1

        return current_wp

    def publish_waypoints(self):
        if (self.base_waypoints_received == False or self.current_position_received == False):
            return

        if self.last_waypoint == 10901:
            rospy.logdebug("one track completed")

        current_wp = self.closest_waypoint()

        #rospy.logdebug("cwp {}".format(current_wp))

        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)

        if self.traffic_light_wp != -1:
            deaccel = 0
            dist = dl(self.current_pose.position, self.base_waypoints[self.traffic_light_wp].pose.pose.position)
            if dist != 0:
                deaccel = (0 - (self.current_linear_velocity * self.current_linear_velocity)) / (2 * dist)

            #rospy.logdebug("deaccel {}".format(deaccel))
            #if deaccel != 0 and deaccel > -10.0 and deaccel < -5.0:
            if deaccel != 0 and (deaccel > -5.0 or self.traffic_wp_start == -1):
                #rospy.logdebug("clv {} deaccel {}".format(self.current_linear_velocity, deaccel))
                if self.traffic_wp_start == -1:
                    self.traffic_wp_start = current_wp
                    self.orig_velocity = self.get_waypoint_velocity(self.base_waypoints[self.traffic_light_wp])

                for i in range(self.traffic_light_wp - current_wp + 1):
                    target_vel = 0
                    dist = dl(self.current_pose.position, self.base_waypoints[current_wp + i].pose.pose.position)
                    #rospy.logdebug("wp {} dist {}".format(current_wp + i, dist))
                    temp1 = self.current_linear_velocity * self.current_linear_velocity
                    temp2 = 2 * deaccel * dist
                    temp3 = temp1 + temp2
                    if temp3 > 0:
                        target_vel = math.sqrt(temp3)
                    #target_vel = target_vel - self.current_linear_velocity
                    #rospy.logdebug("target vel {} for wp {} dist {}".format(target_vel, current_wp + i, dist))
                    self.set_waypoint_velocity(self.base_waypoints, current_wp + i, target_vel)
                    self.set_waypoint_velocity(self.base_waypoints, self.traffic_light_wp - 1, 0)
                    self.set_waypoint_velocity(self.base_waypoints, self.traffic_light_wp, 0)

        self.last_waypoint = current_wp
        final_waypoints_ = Lane()
        final_waypoints_.header.seq = self.final_wp_seq
        self.final_wp_seq = self.final_wp_seq + 1

        if (current_wp + LOOKAHEAD_WPS) < len(self.base_waypoints):
            final_waypoints_.waypoints.extend(self.base_waypoints[current_wp:current_wp+LOOKAHEAD_WPS])
        else:
            final_waypoints_.waypoints.extend(self.base_waypoints[current_wp:(len(self.base_waypoints)-1)])
            final_waypoints_.waypoints.extend(self.base_waypoints[0:LOOKAHEAD_WPS-(len(self.base_waypoints)-current_wp)])

        self.final_waypoints_pub.publish(final_waypoints_)

    def loop(self):
        rate = rospy.Rate(30)

        rospy.logdebug("### loop start")
        while not rospy.is_shutdown():
            current_wp = self.closest_waypoint()
            if current_wp > self.last_waypoint:
                self.publish_waypoints()
            rate.sleep()

    def current_velocity_cb(self, msg):
        self.current_linear_velocity = msg.twist.linear.x

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        now = rospy.get_rostime()
        self.base_waypoints = waypoints.waypoints
        self.base_waypoints_received = True
        rospy.logdebug("num of wps {}".format(len(self.base_waypoints)))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        current_wp = self.closest_waypoint()
        if msg.data != -1 and self.traffic_light_wp == -1:
            rospy.logdebug("### Traffic light is RED at {} and car is at {}".format(msg.data, self.last_waypoint))
        elif msg.data == -1 and self.traffic_light_wp != -1:
            rospy.logdebug("### RED light changed cwp {} tlwp {} clv {}".format(current_wp, self.traffic_light_wp, self.current_linear_velocity))

            if ((current_wp >= (self.traffic_light_wp - 1)) or self.current_linear_velocity < 0.05):
                rospy.logdebug("restore velocity")
                for i in range(self.traffic_light_wp - self.traffic_wp_start):
                    #rospy.logdebug("wp {} vel {}".format(self.traffic_wp_start + i, self.orig_velocity))
                    self.set_waypoint_velocity(self.base_waypoints, self.traffic_wp_start + i, self.orig_velocity)
                self.set_waypoint_velocity(self.base_waypoints, self.traffic_light_wp, self.orig_velocity)
                self.traffic_wp_start = -1

        self.traffic_light_wp = msg.data
        self.publish_waypoints()

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
