#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import tf

import math, time

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

LOOKAHEAD_WPS    = 60 # Number of waypoints we will publish. You can change this number
DEBUG = True


class WaypointUpdater(object):

    x = -1
    yaw = -1
    y = -1
    waypoints = []
    future_waypoints = []
    nearest_traffic_light_waypoint_index = -1
    car_x = -1
    car_y = -1

    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below

        #rospy.spin()
        self.spin()

    def spin(self):
        rate = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.publish_points()
            rate.sleep()

    def publish_points(self):
        start = False
        length_of_future_waypoints = 0
        self.future_waypoints = []
        red_tl_index = -1
        speed_reduction = 0
        while length_of_future_waypoints < LOOKAHEAD_WPS:
            for waypoint_index, waypoint in enumerate(self.waypoints):
                quaternion = (
                    waypoint.pose.pose.orientation.x,
                    waypoint.pose.pose.orientation.y,
                    waypoint.pose.pose.orientation.z,
                    waypoint.pose.pose.orientation.w)
                euler = tf.transformations.euler_from_quaternion(quaternion)
                waypoint_yaw = euler[2]
                waypoint_x = waypoint.pose.pose.position.x
                waypoint_y = waypoint.pose.pose.position.y
                car_x_in_waypoint = (self.car_x - waypoint_x) * math.cos(-waypoint_yaw) - (self.car_y - waypoint_y) * math.sin(-waypoint_yaw)
                if not start and car_x_in_waypoint < 0:
                    start = True
                    # for i in range(LOOKAHEAD_WPS):
                    #     if waypoint_index + i == self.nearest_traffic_light_waypoint_index:
                    #         red_tl_index = i
                    #         break
                    # if red_tl_index > -1:
                    #     speed_reduction = 11.11 / red_tl_index
                if start:
                    #self.set_waypoint_velocity(self.waypoints, waypoint_index, 0)
                    #if length_of_future_waypoints <= red_tl_index + 20:
                    #    waypoint.twist.twist.linear.x = 11.11 - (speed_reduction * length_of_future_waypoints)
                    self.future_waypoints.append(waypoint)
                    length_of_future_waypoints += 1
                if length_of_future_waypoints >= LOOKAHEAD_WPS:
                    break
        lane = Lane()
        lane.header.seq = 1
        seconds = time.time()
        secs, nsecs = math.modf(seconds)
        lane.header.stamp.secs = secs
        lane.header.stamp.nsecs = nsecs
        lane.header.frame_id = 'world'
        lane.waypoints = self.future_waypoints
        self.final_waypoints_pub.publish(lane)

    def pose_cb(self, msg):
        quaternion = (
           msg.pose.orientation.x,
           msg.pose.orientation.y,
           msg.pose.orientation.z,
           msg.pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        roll = euler[0]
        pitch = euler[1]
        yaw = euler[2]
        car_yaw = yaw
        self.car_x = msg.pose.position.x
        self.car_y = msg.pose.position.y


    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints

    def traffic_cb(self, msg):
        self.nearest_traffic_light_waypoint_index = msg.data

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
