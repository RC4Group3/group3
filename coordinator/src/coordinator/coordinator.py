#!/usr/bin/env python

import roslib; roslib.load_manifest('coorinator')

import rospy

# uses pretty much every service interface we've defined...
from brics_msgs.srv import *



class bricsCoordinator():
    def __init__(self):
        # for interacting with ar_follower
        ar_service = "/run_ar_follower"
        self.ar_nav_service = rospy.ServiceProxy(ar_service, markerFollower)
        rospy.loginfo("waiting for AR follower service ...")
        rospy.wait_for_service(ar_service)
        rospy.loginfo("....got AR follower service!")

        # for interacting with the map-based navigation
        map_service = "/run_map_planner"
        self.map_nav_service = rospy.ServiceProxy(map_service, mapPlanner)
        rospy.loginfo("waiting for map_service service ...")
        rospy.wait_for_service(map_service)
        rospy.loginfo("....got map_service service!")
        
        # for calling the marker -> map transition
        map_transition_service = "/transition_marker2map"
        self.map_transition_service = rospy.ServiceProxy(map_transition_service, marker_map_transition)
        rospy.loginfo("waiting for transition_marker2map service ...")
        rospy.wait_for_service(transition_marker2map)
        rospy.loginfo("....got transition_marker2map service!")

        # for calling the map -> marker transition
        marker_transition_service = "/transition_map2marker"
        self.marker_transition_service = rospy.ServiceProxy(marker_transition_service, map_marker_transition)
        rospy.loginfo("waiting for transition_map2marker service ...")
        rospy.wait_for_service(transition_map2marker)
        rospy.loginfo("....got transition_map2marker service!")
        

    def run_test_script(self):
        # this script assumes that the robot is currently located at marker 35
        # initialize the mode_mapper
        init_mapper_req = getMapModeRequest()
        init_mapper_req.mode = init_mapper_req.MARKER_MODE
        init_mapper_req.marker = 35
        self.marker_transition_service(init_mapper_req)


        trail_1_2 = [0, 1, 2, 3, 4, 5, 6, 7, 8]
        trail_2_1 = [10, 11, 12, 13, 14, 15, 16, 17, 18]

        # send the robot to room 2/B
        ar_req = markerFollowerRequest()
        ar_req.ids = trail_1_2
        self.ar_nav_service(ar_req)

        #transition the robot to the map mode in room 2/B
        map_transition_req = markerMapTransitionRequest()
        map_transition_req.map_id = map_transition_req.MAP_B
        map_transition.req.map_x = 0.56
        map_transition.req.map_y = 2.03
        map_transition.req.map_th = -1.56
        self.map_transition_service(map_transition_req)
        
        # move the robot around in the room ...
        map_plan_req = mapPlannerRequest()
        map_plan_req.frame_id = "/map2"
        map_plan_req.goal_x = 1
        map_plan_req.goal_y = 0
        map_plan_req.goal_th = 0
        self.map_nav_service(map_plan_req)

        map_plan_req.goal_x = 0.63
        map_plan_req.goal_y = 1.24
        map_plan_req.goal_x = 1.63
        self.map_nav_service(map_plan_req)

        # transition into marker-following mode
        marker_transition_req = mapMarkerTransitionRequest()
        marker_transition_req.marker_id = 10
        self.marker_transition_service(marker_transition_req)
        
        # follow markers back to room 1
        # send the robot to room 2/B
        ar_req = markerFollowerRequest()
        ar_req.ids = trail_2_1
        self.ar_nav_service(ar_req)

        # transition marker->map in room 1/A
        map_transition_req = markerMapTransitionRequest()
        map_transition_req.map_id = map_transition_req.MAP_A
        map_transition.req.map_x = -0.55
        map_transition.req.map_y = -0.05
        map_transition.req.map_th = 0
        self.map_transition_service(map_transition_req)

        # move the robot around in the room ...
        map_plan_req = mapPlannerRequest()
        map_plan_req.frame_id = "/map1"
        map_plan_req.goal_x = 2
        map_plan_req.goal_y = 0
        map_plan_req.goal_th = 0
        self.map_nav_service(map_plan_req)
        


if __name__=="__main__":
    rospy.init_node('coordinator')
    my_coordinator = bricsCoordinator()
    my_coordinator.run_test_script()

