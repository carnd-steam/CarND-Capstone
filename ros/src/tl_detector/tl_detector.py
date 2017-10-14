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
import numpy as np

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.light_stop_positions = []

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

    def get_closest_waypoint(self, pose):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        pose_x = pose.position.x
        pose_y = pose.position.y
        return self.get_closest_waypoint_by_x_y(pose_x, pose_y)


    def get_closest_waypoint_by_x_y(self, pose_x, pose_y):
        min_distance = 99999
        min_index = -1
        if self.waypoints:
            for i,waypoint in enumerate(self.waypoints.waypoints):
                x = waypoint.pose.pose.position.x
                y = waypoint.pose.pose.position.y
                distance = math.sqrt(math.pow(pose_x - x, 2) + math.pow(pose_y - y, 2))
                if distance < min_distance:
                    min_distance = distance
                    min_index = i
            return min_index
        else:
            return None


    def project_to_image_plane(self, point_in_world):
        """Project point from 3D world coordinates to 2D camera image location

        Args:
            point_in_world (Point): 3D location of a point in the world

        Returns:
            x (int): x coordinate of target point in image
            y (int): y coordinate of target point in image

        """

        fx = self.config['camera_info']['focal_length_x']
        fy = self.config['camera_info']['focal_length_y']
        image_width = self.config['camera_info']['image_width']
        image_height = self.config['camera_info']['image_height']

        # get principal point
        cx = image_width / 2
        cy = image_height / 2

        # get transform between pose of camera and world frame
        trans = None
        rot = None
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/base_link", "/world", now, rospy.Duration(1.0))
            (trans, rot) = self.listener.lookupTransform("/base_link", "/world", now)

        except (tf.Exception, tf.LookupException, tf.ConnectivityException):
            rospy.logerr("Failed to find camera to map transform")

        # Use tranform and rotation to calculate 2D position of light in image
        # convert quaternion
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(rot)
        rvec = np.array([roll, pitch, yaw])
        tvec = np.array(trans)

        # camera intrinsic matrix
        camera_intrinsic = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])

        # world map point
        world_point = np.array([[point_in_world.x, point_in_world.y, point_in_world.z]])      

        # projection point using Opencv
        projection_point, _ = cv2.projectPoints(world_point, rvec, tvec, camera_intrinsic, None)
        projection_point_pixel = np.int32(projection_point).reshape(-1, 2)
        
        x = projection_point_pixel[0][0]
        y = projection_point_pixel[0][1]

        return (x, y)

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

        x, y = self.project_to_image_plane(light.pose.pose.position)

        #TODO use light location to zoom in on traffic light in image
        if (x < 0) or (y < 0) or (x >= cv_image.shape[1]) or (y >= cv_image.shape[0]):
            return False
        
        tmp_img = cv_image
        crop_value = 90
        xmin = x - crop_value if (x - crop_value) >= 0 else 0
        ymin = y - crop_value if (y - crop_value) >= 0 else 0

        xmax = x + crop_value if (x + crop_value) <= tmp_img.shape[1] - 1 else tmp_img.shape[1] - 1
        ymax = y + crop_value if (y + crop_value) <= tmp_img.shape[0] - 1 else tmp_img.shape[0] - 1
        crop_img = tmp_img[ymin:ymax, xmin:xmax]
             
        #Get classification
        #return self.light_classifier.get_classification(crop_img)
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
        if(self.pose):
            car_position = self.get_closest_waypoint(self.pose.pose)

        if not self.light_stop_positions:
            for stop_line_position in stop_line_positions:
                stop_line_position_x, stop_line_position_y = stop_line_position
                position_index = self.get_closest_waypoint_by_x_y(stop_line_position_x, stop_line_position_y)
                self.light_stop_positions.append(position_index)

        min_distance = 10000
        min_traffic_light_index = -1
        if self.pose and car_position:
            for light_stop_position in self.light_stop_positions:
                distance_car_traffic_light = light_stop_position - car_position
                if distance_car_traffic_light < min_distance and distance_car_traffic_light > 0:
                    min_distance = distance_car_traffic_light
                    min_traffic_light_index = light_stop_position

        if min_distance < 200:
            light = self.waypoints.waypoints[min_traffic_light_index]

        if light:
            state = self.get_light_state(light)

            # TODO: ground truth, must be removed before deployed
            min_distance = 10000
            nearest_light_index = None
            for index_light, ground_truth_light in enumerate(self.lights):
                nearest_light_x = light.pose.pose.position.x
                nearest_light_y = light.pose.pose.position.y
                light_x = ground_truth_light.pose.pose.position.x
                light_y = ground_truth_light.pose.pose.position.y
                distance_between_traffic_lights = math.sqrt(math.pow(nearest_light_x - light_x, 2) + math.pow(nearest_light_y - light_y, 2))
                if distance_between_traffic_lights < min_distance:
                    min_distance = distance_between_traffic_lights
                    nearest_light_index = index_light
            state = self.lights[nearest_light_index].state
            x = self.lights[nearest_light_index].pose.pose.position.x
            y = self.lights[nearest_light_index].pose.pose.position.y
            # end_of_todo

            tl_waypoint_index = self.get_closest_waypoint_by_x_y(x, y)
            return tl_waypoint_index, state
        #self.waypoints = None
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
