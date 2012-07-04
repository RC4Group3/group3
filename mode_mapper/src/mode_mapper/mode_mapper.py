#!/usr/bin/env python
import roslib; roslib.load_manifest('mode_mapper')
import brics_msgs
import rospy

from brics_msgs.msg import map_mode
from brics_msgs.srv import getMapMode, getMapModeResponse, setMapMode, setMapModeResponse

class modeMapper():
    def __init__(self):
        self.status = map_mode()
        self.status.mode = self.status.MAP_A

        self.status_pub = rospy.Publisher("/global_map_status", map_mode)
        self.set_mode_server = rospy.Service("/set_map_mode", setMapMode, self.set_mode_cb)
        self.get_mode_server = rospy.Service("/get_map_mode", getMapMode, self.get_mode_cb)

    def set_mode_cb(self, req):
        self.status.mode = req.mode
        if req.mode == self.status.MARKER_MODE:
            self.status.marker = req.marker
        resp = setMapModeResponse()
        return resp

    def get_mode_cb(self, req):
        resp = getMapModeResponse()
        resp.map_status = self.status
        return resp

    def publish_mode(self):
        self.status_pub.publish(self.status)


if __name__=="__main__":
    rospy.init_node("mode_mapper")
    my_mapper = modeMapper()
    while not rospy.is_shutdown():
        my_mapper.publish_mode()
        rospy.sleep(0.2)
