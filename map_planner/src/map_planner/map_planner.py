#!/usr/bin/env python
import roslib; roslib.load_manifest('map_planner')
import actionlib
import rospy
import tf

# for service call from Andreas's graph planner...
from brics_msgs.srv import mapPlanner, mapPlannerResponse
# for handling the interaction with move_base
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from move_base_msgs.msg import *

# can't have class name and message name be the same....
class foomapPlanner():
    def __init__(self):
        self.server = rospy.Service("/run_map_planner", mapPlanner, self.service_cb)
        self.controller_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        print "waiting for move_base..."
        self.controller_client.wait_for_server()
        print "got move_base"

    def service_cb(self, req):
        # create goal message
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = req.frame_id
        goal_pose.header.frame_id = rospy.Time.now() # ... should probably be aprt of the request...
        goal_pt = Point(req.goal_x, req.goal_y, 0.0)
        goal_pose.pose.position = goal_pt
        qq = tf.transformations.quaternion_from_euler(0, 0, req.goal_th)
        goal_pose.pose.orientation = Quaternion(*qq)
        goal_msg = MoveBaseGoal()
        goal_msg.target_pose = goal_pose
        
        # send goal message and wait ...
        self.controller_client.send_goal(goal_msg)
        self.controller_client.wait_for_result()

        resp = mapPlannerResponse()
        return resp

if __name__=="__main__":
    rospy.init_node('map_planner')
    my_planner = foomapPlanner()
    rospy.spin()
