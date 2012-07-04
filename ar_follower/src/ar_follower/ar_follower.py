#!/usr/bin/env python
import roslib; roslib.load_manifest('ar_follower')
import actionlib
import rospy
import smach
import smach_ros
import tf

import math

# TODO: replace this with the real message type, as soon as I have real data to test with ...

from actionlib_msgs.msg import GoalStatus
from brics_msgs.srv import markerFollower, markerFollowerResponse
from mbn_msgs.msg import MarkersPoses
from geometry_msgs.msg import Quaternion, PoseStamped, Point
from move_base_msgs.msg import *
from smach_ros import SimpleActionState

class Init(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                             outcomes=['done', 'keep_looking', 'not_seen', 'tag_seen'],
                             input_keys=['tag_ids', 'init_iter_count'],
                             output_keys=['tag_location', 'init_iter_count'])

        self.tag_subscriber = rospy.Subscriber("/markers_poses_topic", MarkersPoses, self.marker_cb)
        self.curr_msg = None

    def marker_cb(self, marker_msg):
        self.curr_msg = marker_msg
        
    def execute(self, userdata):
        print "executing state ar_follower.Init with id list: %r" % (userdata.tag_ids)
        if userdata.tag_ids == []:
            return 'done'
        # make sure we get the *next* tag detection message
        self.curr_msg = None
        while self.curr_msg is None:
            rospy.sleep(2.0)
        # TODO: this is sloppy ... should cycle through all and choose best, rather than just looking for first
        # however, it shouldn't matter, as the next callback will find the following marker
        # TODO: this duplicates code in the GotoTag.execute method...
        for marker_pose in self.curr_msg.markersPoses:
            if marker_pose.marker_id in userdata.tag_ids:
                marker_x = marker_pose.poseWRTRobotFrame.position.x
                marker_y = marker_pose.poseWRTRobotFrame.position.y
                # TODO: is this thread-safe? I'm still confused w/ python's threading...
                userdata.tag_location=[marker_x, marker_y]
                userdata.init_iter_count = 0 # gotta reset this
                return 'tag_seen'

        userdata.init_iter_count += 1
        if userdata.init_iter_count < 10:
            return 'keep_looking'
        else:
            userdata.init_iter_count = 0
            return 'not_seen'

class GotoTag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_tag', 'goto_failed', 'new_tag'],
                             input_keys=['tag_location', 'tag_ids'],
                             output_keys=['tag_location', 'tag_ids'])
        self.controller_client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.controller_client.wait_for_server()

        self.tag_subscriber = rospy.Subscriber("/markers_poses_topic", MarkersPoses, self.marker_cb)
        self.curr_msg = None

    def marker_cb(self, marker_msg):
        self.curr_msg = marker_msg

    def execute(self, userdata):
        goal_msg = MoveBaseGoal()
        goal_pose = PoseStamped()
        # TODO: this should get the frame from the input marker message...
        goal_pose.header.frame_id = "/map"
        goal_pose.header.stamp = rospy.Time.now()
        goal_pt = Point(userdata.tag_location[0], userdata.tag_location[1], 0.0)
        goal_pose.pose.position = goal_pt
        # TODO: this should be aligned with the april tag pose!
        qq = tf.transformations.quaternion_from_euler(0, 0, 0.0)
        goal_pose.pose.orientation = Quaternion(*qq)
        goal_msg.target_pose = goal_pose
        self.controller_client.send_goal(goal_msg)
        goal_status = self.controller_client.get_state()
        while goal_status in [GoalStatus.PENDING, GoalStatus.ACTIVE]:
            for marker_pose in self.curr_msg.markersPoses:
                if marker_pose.marker_id in userdata.tag_ids[1:]:
                    marker_x = marker_pose.poseWRTRobotFrame.position.x
                    marker_y = marker_pose.poseWRTRobotFrame.position.y
                    # TODO: is this thread-safe? I'm still confused w/ python's threading...
                    userdata.tag_location=[marker_x, marker_y]
                    tag_idx = userdata.tag_ids.index(marker_pose.marker_id)
                    userdata.tag_ids = userdata.tag_ids[tag_idx:]
                    return 'new_tag'
            rospy.sleep(0.2)
            goal_status = self.controller_client.get_state()

        self.controller_client.wait_for_result()
        print "got controller result!"

        controller_state = self.controller_client.get_state()
        if controller_state == GoalStatus.PREEMPTED:
            return 'goto_failed'
        elif controller_state == GoalStatus.ABORTED:
            print "ar_follower.GotoTag's controller call aborted!"
            return 'goto_failed'
        elif controller_state == GoalStatus.SUCCEEDED:
            print "ar_follower.GotoTag's controller call succeeded!"
            userdata.tag_ids = userdata.tag_ids[1:]
            return 'at_tag'
        else:
            print "ar_follower.GotoTag's controller returned %r" % (controller_state)
            return 'goto_failed'

class my_sm_server():
    def __init__(self):
        sm = smach.StateMachine(outcomes=['DONE'])
        with sm:
            smach.StateMachine.add('INIT', Init(), 
                                   transitions={'done':'DONE', 
                                                'not_seen':'TURN', 
                                                'keep_looking':'INIT', 
                                                'tag_seen':'FOLLOW_PATH'})
            goal_msg = MoveBaseGoal()
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "/base_link"
            goal_pose.header.stamp = rospy.Time.now()
            goal_pt = Point(0.0, 0.0, 0.0)
            goal_pose.pose.position = goal_pt
            qq = tf.transformations.quaternion_from_euler(0, 0, math.pi/6)
            goal_pose.pose.orientation = Quaternion(*qq)
            goal_msg.target_pose = goal_pose
            smach.StateMachine.add('TURN',
                                   SimpleActionState('move_base', MoveBaseAction, 
                                                     goal=goal_msg),
                                   transitions={'succeeded':'INIT', 
                                                'preempted':'INIT',
                                                'aborted':'INIT'})
        
            smach.StateMachine.add('FOLLOW_PATH', GotoTag(),
                                   transitions={'at_tag':'INIT', 
                                                'goto_failed':'INIT',
                                                'new_tag':'FOLLOW_PATH'})
            
        self.sm = sm
        self.server = rospy.Service("rum_ar_follower", markerFollower, self.service_cb)

    def service_cb(self, req):
        self.sm.userdata.tag_ids = req.ids
        self.sm.userdata.init_iter_count = 0
        result = self.sm.execute()
        resp = markerFollowerResponse()
        return resp


def main():
    rospy.init_node('ar_follower')
    foo = my_sm_server()
    rospy.spin()

if __name__=="__main__":
    main()
