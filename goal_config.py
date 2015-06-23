#!/usr/bin/python

#This node is for casting ar_pose msgs to ar_track_alvar messages for use in hrl_pr2_ar_servo 
import roslib
roslib.load_manifest('hrl_pr2_ar_servo')
roslib.load_manifest('ar_track_alvar')

import rospy
import rostopic
from hrl_pr2_ar_servo.msg import ARServoGoalData
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from ar_pose.msg import ARMarker

from std_msgs.msg import Bool, Int8, String
from geometry_msgs.msg import PoseStamped

class recastMarkerMsg:
    def __init__(self):
        rospy.init_node('pr2_ar_servo_goal_config')
        self.defined=False        
        rospy.Subscriber("ar_servo_goal_data", ARServoGoalData, self.goal_cb)
        
    def goal_cb(self, msg):
        goal=msg
        if goal.marker_topic[0]=="/": #check if there is a '/' in front of the topic name so rostopic doesn't error
            marker_type, real_name, fn=rostopic.get_topic_type(goal.marker_topic)
        else:
            marker_type, real_name, fn=rostopic.get_topic_type("/"+goal.marker_topic)
        
        if marker_type == "ar_pose/ARMarker" and self.defined=False:
            self.marker_repub=rospy.Publisher(real_name, AlvarMarkers)           
            rospy.Subscriber(real_name, ARMarker, self.marker_switch)
            self.defined=True:

    def marker_switch(self, msg):
        old_msg=msg
        new_msg=AlvarMarkers()
        new_msg.header=old_msg.header
        marker=AlvarMarker()
        marker.header=old_msg.header
        marker.id=old_msg.id
        marker.confidence=old_msg.confidence
        marker.pose.header=old_msg.header
        marker.pose.pose=old_msg.pose.pose
        new_msg.markers=[marker]
        self.marker_repub.publish(new_msg)                
            
if __name__=="__main__":
    a=recastMarkerMsg()
    while not (rospy.is_shutdown()):
        rospy.spin()
