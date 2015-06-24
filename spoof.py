#! /usr/bin/python
import roslib
roslib.load_manifest('tf')
roslib.load_manifest('actionlib')
roslib.load_manifest('ar_pose')
roslib.load_manifest('ar_track_alvar')


import rospy
import actionlib
from actionlib_msgs.msg import *
from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from ar_pose.msg import ARMarker
from geometry_msgs.msg import PointStamped, Pose, Quaternion, PoseStamped
from std_msgs.msg import String, Bool
import tf





class Spoof:
    
    def __init__(self):
        rospy.init_node('spoof_marker')
        self.broadcaster=tf.TransformBroadcaster()
        self.listener=tf.TransformListener()
        self.ar_pose_pub=rospy.Publisher('r_pr2_ar_pose_marker', ARMarker)
        #self.ar_pub=rospy.Publisher('r_pr2_ar_pose_marker', AlvarMarkers)
        
        self.spoof2()        
        #self.spoof()

    def spoof2(self):
        goal=ARMarker()
        rate=rospy.Rate(1)
        while not rospy.is_shutdown():
            goal.header.stamp=rospy.Time.now()
            goal.pose.pose.position.x=1.6557879542599779-0.5
            goal.pose.pose.position.y= 1.024584175222559287
            goal.pose.pose.position.z=1.2336137059145866
            goal.pose.pose.orientation.x=0.38268343
            goal.pose.pose.orientation.y=-0.92387953
            goal.pose.pose.orientation.z=0.38268343
            goal.pose.pose.orientation.w=0.92387953
            #goal.pose.covariance=0.1
            self.ar_pose_pub.publish(goal)
            print "Goal published"
            rate.sleep()

    def spoof(self):
        rate=rospy.Rate(1)
        goal=AlvarMarkers()
        self.broadcaster.sendTransform((1.6557879542599779, 1.024584175222559287, 1.2336137059145866),( 0.38268343, -0.92387953,  0.38268343,  0.92387953), rospy.Time.now(),"/ar_marker_0", "/base_link")
        while not rospy.is_shutdown():
            #Send a transform for a AR Tag
            #self.broadcaster.sendTransform((0.6557879542599779, 0.024584175222559287, 1.2336137059145866),(0.0, -1, 0.0, 1), rospy.Time.now(),"/ar_marker_1", "/base_link")     
            try:
                now=rospy.Time.now()
                #self.listener.waitForTransform('/base_link', '/ar_marker_0', now, rospy.Duration(1))
                #trans, rot=self.listener.lookupTransform('/base_link', '/ar_marker_0', now)            
                goal.header.stamp=rospy.Time.now()
                goal.header.frame_id='base_link'
                marker=AlvarMarker()
                marker.header.stamp=rospy.Time.now()
                marker.header.frame_id='base_link'
                marker.id=0
                marker.pose.header.stamp=rospy.Time.now()
                marker.pose.header.frame_id='base_link'
                marker.pose.pose.position.x=1.6557879542599779-0.5
                marker.pose.pose.position.y= 1.024584175222559287
                marker.pose.pose.position.z=1.2336137059145866
                marker.pose.pose.orientation.x=0.38268343
                marker.pose.pose.orientation.y=-0.92387953
                marker.pose.pose.orientation.z=0.38268343
                marker.pose.pose.orientation.w=0.92387953
                goal.markers=[marker]            
                self.ar_pub.publish(goal)
            except(tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, tf.Exception):
                print"TF exception, try again"

            rate.sleep()
              


if __name__=='__main__':
    a=Spoof()
    while not rospy.is_shutdown():
        rospy.spin()
