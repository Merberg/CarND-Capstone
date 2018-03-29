#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math
from scipy._lib._ccallback_c import idx

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

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=2)

        
        # TODO: Add other member variables you need below
        self.base_init = False
        self.base = Lane()
        self.final = Lane()
        self.first_index = 0

        # spin() keeps python from exiting until this node is stopped
        rospy.spin()                   

    def pose_cb(self, msg):
        if self.base_init:
            self.first_index = self.get_next_waypoint_index(self.base.waypoints,
                                                            msg.pose.position,
                                                            self.first_index)
            self.final.waypoints = self.set_final_waypoints(self.base.waypoints, self.first_index, 5)
            self.final_waypoints_pub.publish(self.final)                                                    
        pass

    def waypoints_cb(self, waypoints):
        self.base = waypoints
        rospy.loginfo('%i Base Waypoints', len(waypoints.waypoints))
        self.base_init = True            
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass
    
    def get_waypoint_position(self, waypoint):
        return waypoint.pose.pose.position

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def get_next_waypoint_index(self, waypoints, position, first_index):
        rospy.loginfo('Prev %i to %i', first_index, len(waypoints))
        prev_idx = max(first_index-1, 0)
                
        for idx, wp in enumerate(waypoints[first_index:]):            
            wp_pos = self.get_waypoint_position(wp)
            prev_wp_pos = self.get_waypoint_position(waypoints[prev_idx])
            
            if prev_wp_pos.x <= position.x and position.x < wp_pos.x:
                rospy.loginfo('(%.3f, %.3f, %.3f) Next waypoint = %i (%.3f, %.3f, %.3f)',
                              position.x, position.y, position.z, 
                              first_index + idx,
                              wp_pos.x, wp_pos.y, wp_pos.z)
                return first_index + idx
            else:
                prev_idx = max(first_index+idx-1, 0)
        
        rospy.loginfo('Next waypoint rolling over, car at (%.3f, %.3f, %.3f)', 
                     position.x, position.y, position.z)
        first_index = 0
        idx = self.get_next_waypoint_index(waypoints, position, first_index)
        return idx
    
    def set_final_waypoints(self, waypoints, first_index, velocity):
        #Function for the partial implementation testing
        final = []
        wp = Waypoint()
        for idx in range(LOOKAHEAD_WPS):
            wp.pose.pose.position = self.get_waypoint_position(waypoints[idx+first_index])
            wp.twist.twist.linear.x = velocity
            final.append(wp)
        return final
        
    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(self.get_waypoint_position(waypoints[wp1]), self.get_waypoint_position(waypoints[i]))
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
