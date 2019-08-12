#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree # to build kdtree to search waypoints in a better way
import numpy as np
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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.pose = None
        # Will store the original waypoints of the track
        self.base_lane = None
        # Will store waypoints only 2d pose x and y coordinates
        self.waypoints_2d = None
        # Will store the kd tree to search waypoints in a better way
        self.waypoint_tree = None
        self.stopline_wp_idx = -1
        # instead of spin we will be looping around functions to read pose and perform tasks
        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane and self.waypoint_tree:
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_id(self):
        """
        Returns the index of the closest waypoint ahead of the vehicle
        """
        x =self.pose.pose.position.x
        y =self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y],1)[1]
        prev_closest_idx = closest_idx-1

        # check if closest is behind or in front of the vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_closest_coord = self.waypoints_2d[prev_closest_idx]

        #equation for hyperplane through closes coords
        cl_vect = np.array(closest_coord)
        prev_vect = np.array(prev_closest_coord)
        pos_vect = np.array([x,y])

        val = np.dot(cl_vect-prev_vect, pos_vect - cl_vect)
        if val > 0:
            closest_idx = (closest_idx + 1 ) % len(self.waypoints_2d)

        return  closest_idx
    def publish_waypoints (self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        closest_wp_idx = self.get_closest_waypoint_id()
        farthest_wp_idx = closest_wp_idx + LOOKAHEAD_WPS
        lane = Lane()
        lane.header = self.base_lane.header
        base_waypoints = self.base_lane.waypoints[closest_wp_idx:farthest_wp_idx]
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_wp_idx):
            lane.waypoints = base_waypoints
        else :
            lane.waypoints = self.decelerate_waypoints(base_waypoints,closest_wp_idx)
        return  lane
    def decelerate_waypoints(self, waypoints, closest_idx):
        temp =[]
        for i ,wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx -closest_idx -2,0) # two waypoints back of the line for some security
            dist = self.distance(waypoints,i,stop_idx)
            vel = math.sqrt(2*MAX_DECEL*dist)
            if vel < 1.:
                vel = 0.
            p.twist.twist.linear.x = min(vel, wp.twist.twist.linear.x)
            temp.append(p)

        return temp
    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[w.pose.pose.position.x, w.pose.pose.position.y] for w in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint_id, velocity):
        waypoints[waypoint_id].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2 + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
