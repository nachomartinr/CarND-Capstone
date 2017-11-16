#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
from light_classification.color_classifier import ColorClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')
	self.i = 0
	#stop line
	self.traffic_line_index = []
	config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

	#traffic light
        self.lights = []
	self.traffic_light_index = []
	self.signal_classes = ['STOP', 'STOP', 'GO', 'Unknown', 'Unknown'] 
	sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)

	#position
	self.position = None
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)

	#waypoints
        self.waypoints = None
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

	#camera_image
        self.camera_image = None
	self.has_image = False
	sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

	self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)
	self.car_position = rospy.Publisher('/car_position', Int32, queue_size=1)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''

        self.bridge = CvBridge()
        self.listener = tf.TransformListener()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.i =0
	#load classifier,time comsuming
	self.light_classifier = TLClassifier()
	self.color_Classifier = ColorClassifier()
        rospy.logwarn('You can launch simulator now!')
	#set spin rate according to time required for classification
	rate = rospy.Rate(5)
	start_time = 0
	while not start_time:
	    start_time = rospy.Time.now().to_sec()

	while not rospy.is_shutdown():
	    self.detect_tl()
	    rate.sleep()

    def detect_tl(self):
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
            light_wp = light_wp if (state == TrafficLight.RED or state == TrafficLight.YELLOW) else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def pose_cb(self, msg):
	#find position of the ego car
	if(self.waypoints):
	    self.position = self.get_closest_waypoint(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
	    self.car_position.publish(Int32(self.position))
	    

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints
	#record position of the stop line
	stop_line_positions = self.config['stop_line_positions']
	for i in range(len(stop_line_positions)):
	    traffic_light_position = self.get_closest_waypoint(stop_line_positions[i][0], stop_line_positions[i][1], 0)
	    self.traffic_line_index.append(traffic_light_position)
	    

    def traffic_cb(self, msg):
        self.lights = msg.lights
	#record position of the traffic light
	if ( len(self.traffic_light_index) is 0 and len(self.traffic_line_index) is not 0):

	    for i in range(len(self.traffic_line_index)):
		line_x = self.waypoints.waypoints[self.traffic_line_index[i]].pose.pose.position.x
		line_y = self.waypoints.waypoints[self.traffic_line_index[i]].pose.pose.position.y
		line_z = self.waypoints.waypoints[self.traffic_line_index[i]].pose.pose.position.z
		nearest_dist = 99999
		for j in range(len(self.lights)):
		    light_x = self.lights[j].pose.pose.position.x
		    light_y = self.lights[j].pose.pose.position.y
		    light_z = self.lights[j].pose.pose.position.z
		    dist = math.sqrt((light_x-line_x)**2 + (light_y-line_y)**2  + (light_z-line_z)**2)
		    if dist < nearest_dist:
			nearest_dist = dist
			index = j
		self.traffic_light_index.append(index)	    

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        
    def get_closest_waypoint(self, px, py, pz):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints

        """
        #TODO implement
	wp_length = len(self.waypoints.waypoints)	    
	nearest_dist = 99999
	for i in range(0, wp_length, 1):
	    wp_px = self.waypoints.waypoints[i].pose.pose.position.x
	    wp_py = self.waypoints.waypoints[i].pose.pose.position.y
	    wp_pz = self.waypoints.waypoints[i].pose.pose.position.z
	    dist = math.sqrt((wp_px-px)**2 + (wp_py-py)**2  + (wp_pz-pz)**2)

	    if dist < nearest_dist: 
		nearest_dist = dist
		pos_index = i
        return pos_index

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

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
	#cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
	box_coords = self.light_classifier.get_classification(cv_image)
	if len(box_coords)>0:
	    #rospy.logwarn('traffic light detected')
            bot, left, top, right = box_coords[0, ...]
	    if ( (top-bot)*(right-left)>1500 ):
                img = cv_image[int(bot):int(top), int(left):int(right)]	    
	        state = self.color_Classifier.get_color(img)
	        name = str(self.i)+'.jpg'
	        #cv2.imwrite('/home/student/CarND-Capstone/ros/'+name,cv2.cvtColor(img, cv2.COLOR_RGB2BGR))
	        #self.i +=1
	    else:
	        #rospy.logwarn('traffic light box is too small')
		state = TrafficLight.UNKNOWN
	else:
	    #rospy.logwarn('Cant find traffic light')
	    state = TrafficLight.UNKNOWN
        return state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None
	
	traffic_light_state = TrafficLight.UNKNOWN
	traffic_line_index = -1
        if(self.position):            
	    car_position = self.position
	    #chech if the ego car is close to one traffic line
	    for i in range(len(self.traffic_line_index)):
	        if (self.traffic_line_index[i]-car_position)%len(self.waypoints.waypoints)<120:
		    traffic_line_index = self.traffic_line_index[i]
		    if (len(self.traffic_light_index) is not 0 and len(self.lights) is not 0 and self.has_image):    
			traffic_light_gt = self.lights[self.traffic_light_index[i]].state			
			time = rospy.Time.now().to_sec()
			#TODO find the closest visible traffic light (if one exists)
			traffic_light_state = self.get_light_state(light)
			time = rospy.Time.now().to_sec()-time
			rospy.logwarn('Used time: %4.2f, Ground truth: %5s, Detected: %5s', time, self.signal_classes[traffic_light_gt], self.signal_classes[traffic_light_state])
		    
        return traffic_line_index, traffic_light_state

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
