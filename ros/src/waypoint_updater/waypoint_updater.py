#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

import tf
from scipy.spatial import KDTree
import numpy as np


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

LOOKAHEAD_WPS = 100 # Number of waypoints we will publish. You can change this number


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        #incoming Topics
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # outgoing Topics
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need
        self.pose           = None
        self.base_waypoints = None
        self.waypoints_2d   = None
        self.waypoint_tree  = None
        
        # Start the loop to get Msgs/Topics
        # Control publishing frequency
        rate = rospy.Rate(20)  # 50 Hz
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()            

    def loop(self):               
        if (self.pose and self.base_waypoints): 
            # Get closest waypoint  
            #closest_waypoint_idx = self.get_closest_waypoint_idx()
            # get the next Waypoint, NOT the closest waypoint, pass the Next waypoint
            closest_waypoint = self.get_closest_waypoint(self.pose)
            next_index = self.get_next_waypoint(self.pose, closest_waypoint)
            
            '''
            final_waypoints = []
            for i in range(next_index, next_index + LOOKAHEAD_WPS):
                i = i % len(self.base_waypoints.waypoints)
                p = self.base_waypoints.waypoints[i]
                final_waypoints.append(p)
            '''
            self.publish_waypoints(next_index)

 
    def dist(self, p1, p2):
        x, y, z = p1.x - p2.x, p1.y - p2.y, p1.z - p2.z
        return math.sqrt(x*x + y*y + z*z)
    
    def get_closest_waypoint(self, pose):
        closest_index = 0
        closest_dist = 100000
        p1 = self.pose.pose.position
        
        for i in range(len(self.base_waypoints.waypoints)):
            p2 = self.base_waypoints.waypoints[i].pose.pose.position
            d = self.dist(p1, p2)
            if (d < closest_dist):
                closest_dist = d
                closest_index = i
              
        return closest_index        
        
        
    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        closest_idx = self.waypoint_tree.query([x,y], 1)[1]  # kd tree (1st closest, idx)
        
        # Check if closest waypoint is ahead or behind vehicle
        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord    = self.waypoints_2d[closest_idx - 1]

        # Equation for hyperplane through closest_coors
        cl_vect       = np.array( closest_coord)
        prev_vect     = np.array( prev_coord)
        pos_vect      = np.array( [x,y])

        val = np.dot( cl_vect - prev_vect, pos_vect - cl_vect)
        # Car is ahead of the closest waypoint
        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)

        return closest_idx

    def get_next_waypoint(self, pose, index):
        next_index = index
        p1 = self.pose.pose.position
        p2 = self.base_waypoints.waypoints[index].pose.pose.position
        heading = math.atan((p2.y-p1.y), (p2.x-p1.x))
        quad = (    self.pose.pose.orientation.x,
                    self.pose.pose.orientation.y,
                    self.pose.pose.orientation.z,
                    self.pose.pose.orientation.w)
        _, _, yaw = tf.transformations.euler_from_quaternion(quad)
        #angle = abs(yaw)
        
        if ( abs(yaw) > math.pi/4 ):
            next_index += 1
        return next_index           

    

    def publish_waypoints(self, closest_idx):
        lane = Lane()

        end_pt = min( closest_idx + LOOKAHEAD_WPS, len(self.base_waypoints.waypoints) )
        lane.header    = self.base_waypoints.header
        lane.waypoints = self.base_waypoints.waypoints[ closest_idx: end_pt]
        self.final_waypoints_pub.publish( lane)                                              
            
    #-------- Topic Callback Functions----------------------------
    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

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
