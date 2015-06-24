import roslib
roslib.load_manifest('hrl_pr2_ar_servo')
roslib.load_manifest('smach_ros')
roslib.load_manifest('ar_track_alvar')
roslib.load_manifest('ar_pose')

import rospy

from hrl_pr2_ar_servo.msg import ARServoGoalData
from ar_track_alvar.msg import AlvarMarkers
from ar_pose.msg import ARMarker

import numpy as np
from std_msgs.msg import Int8, String, Bool
import tf


class SendTagMsg:
    def __init__ (self):
        rospy.init_node('ar_servo_talker')
        self.goal_publisher=rospy.Publisher('ar_servo_goal_data', ARServoGoalData)
        #self.tag_confirm=rospy.Publisher("/pr2_ar_servo/tag_confirm", Bool)        
        self.marker_id=0
        self.confirm=False
        self.sent=False        
        #rospy.Subscriber('/pr2_ar_servo/state_feedback', Int8, self.feedback_cb)
        #rospy.Subscriber('r_pr2_ar_pose_marker', ARMarker, self.ar_pose_cb)
        #rospy.Subscriber('r_pr2_ar_pose_marker', AlvarMarkers, self. alvar_marker_cb) 
        self.ar_pose_pub()
    
    def ar_pose_pub(self):
        rospy.sleep(rospy.Duration(2))
        goal=ARServoGoalData()
        if not self.sent:
            goal.tag_id=self.marker_id
            goal.marker_topic='r_pr2_ar_pose_marker' 
            goal.base_pose_goal.header.stamp=rospy.Time.now()
            #goal.base_pose_goal.header.frame_id='base_link'
            goal.base_pose_goal.pose.position.x=1.6557879542599779-0.5
            goal.base_pose_goal.pose.position.y= 1.024584175222559287
            goal.base_pose_goal.pose.position.z=1.2336137059145866
            goal.base_pose_goal.pose.orientation.x=0.38268343
            goal.base_pose_goal.pose.orientation.y=-0.92387953
            goal.base_pose_goal.pose.orientation.z=0.38268343
            goal.base_pose_goal.pose.orientation.w=0.92387953
                          
            self.goal_publisher.publish(goal)
            print (goal)
            print "goal sent"
            self.sent=True        

    def alvar_marker_cb(self, msg):        
        data=msg
        tag=data.markers
        goal=ARServoGoalData()
        #print "got this far"
        for mark in tag:
            if mark.id is self.marker_id and not self.sent:
                pose=mark.pose
                goal.tag_id=mark.id
                goal.marker_topic='r_pr2_ar_pose_marker'
                mark.pose.pose.position.x=mark.pose.pose.position.x-0.5                
                goal.tag_goal_pose=mark.pose
                self.goal_pub.publish(goal)
                print "goal sent"
                self.sent=True
                

        
    def feedback_cb(self, msg):
        state=msg.data
        if state is 1:
            print "Searching for ARTag"
        if state is 2:
            print "Found ARTag, begin approach"
            rospy.sleep(rospy.Duration(2))
            if not self.confirm:
                self.tag_confirm.publish(True)
                self.confirm=True
        if state is 3:
            print "Unable to find ARTag...something went wrong"
        if state is 4:
            print "Servoing to ARTag"
        if state is 5:
            print "Servo completed successfully"
        if state is 6:
            print "Detected collision with arms. Servoing stopped."
        if state is 7:
            print "Detected collision with base laser. Servoing stopped."
        if state is 8:
            print "View of ARTag lost. Servoing Stopped."
        if state is 9:
            print "Servoing stopped by user"


if __name__=="__main__":
    servo_to_tag=SendTagMsg()
    while not (rospy.is_shutdown()):
        rospy.spin()



