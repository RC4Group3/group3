#!/usr/bin/env python

import roslib; roslib.load_manifest('marker_map_transition')
import rospy
import tf

import math

from brics_msgs.srv import markerMapTransition, markerMapTransitionResponse, setMapMode, setMapModeRequest
from brics_msgs.msg import map_mode
from geometry_msgs.msg import PoseWithCovarianceStamped, Point, Quaternion
from std_srvs.srv import Empty, EmptyRequest


class markerMapTransitioner():
    def __init__(self):
        # needs to initialize the server ...
        self.server = rospy.Service("/transition_marker2map", markerMapTransition, self.server_cb)
        
        amcl1_service = "/amcl1/global_localization"
        self.amcl1_client = rospy.ServiceProxy(amcl1_service, Empty)
        rospy.loginfo("waiting for amcl1 server ...")
        rospy.wait_for_service(amcl1_service)
        rospy.loginfo("...got amcl1 server!")

        amcl2_service = "/amcl2/global_localization"
        self.amcl2_client = rospy.ServiceProxy(amcl2_service, Empty)
        rospy.loginfo("waiting for amcl2 server ...")
        rospy.wait_for_service(amcl2_service)
        rospy.loginfo("...got amcl2 server!")


        self.amcl1_pose_pub = rospy.Publisher("amcl1/initialpose", PoseWithCovarianceStamped)
        self.amcl2_pose_pub = rospy.Publisher("amcl2/initialpose", PoseWithCovarianceStamped)

        map_service = "/set_map_mode"
        self.map_client = rospy.ServiceProxy(map_service, setMapMode)
        rospy.loginfo("waiting for set_map_mode server ...")
        rospy.wait_for_service(map_service)
        rospy.loginfo("...got set_map_mode server!")


    def server_cb(self, transition_req):
        map_mode_msg = map_mode()
        # re-init appropriate amcl estimator ...
        resetAMCLreq = EmptyRequest()
        # this is ugly, but the marker_map_transition code uses a diff enum than the various messages going through the mode_mapper.py
        if transition_req.map_id == map_mode_msg.MAP_A:
            self.amcl1_client(resetAMCLreq)
        elif transition_req.map_id == map_mode_msg.MAP_B:
            self.amcl2_client(resetAMCLreq)
        else:
            rospy.logerror("in markerMapTransitioner.server_cb, requested invalid map_id!! request was: %r", transition_req)

        # give amcl the prior as to where the robot is now ...
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = rospy.Time.now()
        position = Point(transition_req.map_x, transition_req.map_y, 0.0)
        qq = tf.transformations.quaternion_from_euler(0, 0, transition_req.map_th)
        orientation = Quaternion(*qq)
        pose_msg.pose.pose.position = position
        pose_msg.pose.pose.orientation = orientation
        # copying covariance estimate from rviz .cpp file:
        covariance = 36*[0.0]
        covariance[0] = 0.5**2
        covariance[7] = 0.5**2
        covariance[21] = (math.pi/12.0)**2
        pose_msg.pose.covariance = covariance

        if transition_req.map_id == map_mode_msg.MAP_A:
            pose_msg.header.frame_id = "/map1"
            self.amcl1_pose_pub.publish(pose_msg)
        elif transition_req.map_id == map_mode_msg.MAP_B:
            pose_msg.header.frame_id = "/map2"
            self.amcl2_pose_pub.publish(pose_msg)

        # send info to the mapper module to keep track of what mode we're in
        # rosservice call set_map_mode map_frame={0, 1} -1
        set_mode_req = setMapModeRequest()
        if transition_req.map_id == map_mode_msg.MAP_A:
            set_mode_req.mode = map_mode_msg.MAP_A
        elif transition_req.map_id == map_mode_msg.MAP_B:
            set_mode_req.mode = map_mode_msg.MAP_B
        self.map_client(set_mode_req)

        # need to instantiate an empty response message
        transition_resp = markerMapTransitionResponse()
        return transition_resp

if __name__=="__main__":
    rospy.init_node('marker_map_transitioner')
    my_transitioner = markerMapTransitioner()
    rospy.spin()
