#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
import math
import tf

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
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/car_position', Int32, self.carpose_cb)
        
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.traffic_position = None
        #self.target_speed = 10

        self.position = None
        
        rospy.spin()
    
    def carpose_cb(self, msg):
        self.position = msg.data
        if self.base_waypoints:
            wp_length = len(self.base_waypoints)

            #generate waypoints that ego car will follow ahead    
            pos_index = self.position
            lane = Lane()
            lane.header.frame_id = '/world'
            lane.header.stamp = rospy.Time.now()
        
            waypoints = []
            pos_index +=1
            for i in range(LOOKAHEAD_WPS):      
                index = (pos_index+i)%wp_length
                waypoints.append(self.base_waypoints[index])
                if (self.traffic_position):
                    if (self.traffic_position is not -1):

                        delta_s = (self.traffic_position-4) - (pos_index+i)
                        if abs(delta_s%wp_length) < abs(delta_s):
                            delta_s = delta_s%wp_length
                        delta_s = max(delta_s, 0)
                        safe_speed = math.sqrt(2*2*delta_s)
                        set_speed = min(self.target_speed, safe_speed)
                        self.set_waypoint_velocity(waypoints, i, set_speed)
                    else:
                        self.set_waypoint_velocity(waypoints, i, self.target_speed)
                else:
                    #if haven't got traffic condition, wait for safty reason
                    self.set_waypoint_velocity(waypoints, i, 0) 
            lane.waypoints = waypoints
            #publish waypoints  
            self.final_waypoints_pub.publish(lane)
 
    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints.waypoints
        self.target_speed=self.get_waypoint_velocity(self.base_waypoints[0])        

    def traffic_cb(self, position):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_position = position.data

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
