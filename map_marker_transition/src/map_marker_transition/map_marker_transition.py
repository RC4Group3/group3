#!/usr/bin/env python

import roslib; roslib.load_manifest('map_marker_transition')
import rospy

from brics_msgs.msg import map_mode
from brics_msgs.srv import setMapMode, setMapModeRequest, mapMarkerTransition, mapMarkerTransitionResponse

class mapMarkerTransitioner():
    def __init__(self):
        self.server = rospy.Service("/transition_map2marker", mapMarkerTransition, self.server_cb)

        map_service = "/set_map_mode"
        self.map_client = rospy.ServiceProxy(map_service, setMapMode)
        rospy.loginfo("waiting for set_map_mode server ...")
        rospy.wait_for_service(map_service)
        rospy.loginfo("...got set_map_mode server!")

    def server_cb(self, transition_req):
        # pretty much all we have to do is send the message to the mode_mapper 
        # saying that we're now in marker mode, and giving the initial marker ...
        map_mode_msg = map_mode()
        set_mode_req = setMapModeRequest()
        set_mode_req.mode = map_mode_msg.MARKER_MODE
        set_mode_req.marker = transition_req.marker_id
        self.map_client(set_mode_req)
        
        transition_resp = mapMarkerTransitionResponse()
        return transition_resp

if __name__=="__main__":
    rospy.init_node('map_marker_transition')
    my_transitioner = mapMarkerTransitioner()
    rospy.spin()
