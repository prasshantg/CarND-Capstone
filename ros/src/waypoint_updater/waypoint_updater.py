#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):

    base_waypoints = []
    last_waypoint = 0
    base_waypoints_received = False
    current_position_received = False
    final_wp_seq = 0
    traffic_light_state = TrafficLight.UNKNOWN
    current_pos = PoseStamped()

    def __init__(self):
        rospy.init_node('waypoint_updater', log_level=rospy.DEBUG)

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
	# TODO: yet to implement trafic sign classifier
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        self.loop()

    def pose_cb(self, msg):
        # TODO: Implement
        self.current_position_received = True
        self.current_pose = msg.pose

    def loop(self):
        rate = rospy.Rate(10)

        rospy.logdebug("### loop start")
        while not rospy.is_shutdown():
            if (self.base_waypoints_received == True and self.current_position_received == True):
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

                self.last_waypoint = current_wp
                final_waypoints_ = Lane()
                final_waypoints_.header.seq = self.final_wp_seq
                self.final_wp_seq = self.final_wp_seq + 1

                rospy.logdebug("cur pos wp {}".format(current_wp))
                for i in range(LOOKAHEAD_WPS):
                    index = i + current_wp
                    if index >= len(self.base_waypoints):
                        index = index - len(self.base_waypoints)
                    final_waypoints_.waypoints.append(self.base_waypoints[index])

                self.final_waypoints_pub.publish(final_waypoints_)
                rate.sleep()

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        now = rospy.get_rostime()
        self.base_waypoints = waypoints.waypoints
        self.base_waypoints_received = True
        rospy.logdebug("num of wps {}".format(len(self.base_waypoints)))

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_light_state = msg.data

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
