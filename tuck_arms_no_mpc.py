#!/usr/bin/python
import roslib
roslib.load_manifest('actionlib')
roslib.load_manifest('hrl_haptic_mpc')
roslib.load_manifest('pr2_tuck_arms_action')

import actionlib
import rospy
from pr2_common_action_msgs.msg import TuckArmsAction, TuckArmsGoal
from pr2_tuck_arms_action.tuck_arms_main import TuckArmsActionServer
from geometry_msgs.msg import PointStamped, Pose, Quaternion, PoseStamped
from hrl_haptic_manipulation_in_clutter_srvs.srv import EnableHapticMPC
from std_msgs.msg import String, Bool



class tuckArmsNoMPC(object):
    def __init__ (self, tuck=True):
        rospy.init_node('tuck_arms_no_mpc')
        action_name='tuck_arms'
        tuck_arms_action_server = TuckArmsActionServer(action_name)  #Start the Tuck Arms Action Server
        self.tuck_arms_client = actionlib.SimpleActionClient(action_name, TuckArmsAction)
        self.enable_haptic=rospy.ServiceProxy('haptic_mpc/enable_mpc', EnableHapticMPC)
        self.r_enable_haptic=rospy.ServiceProxy('right/haptic_mpc/enable_mpc', EnableHapticMPC)
        success=self.disable_mpc(tuck)
        #self.tuck_arms(tuck)

    def disable_mpc (self, tuck):
        succeeded=False
        enabled_l=True   #Haptic MPC assumed to be enabled
        enabled_r=True
        try: 
            enabled_l=self.enable_haptic('False')   #Try turning off Haptic MPC in order to use pr2_tuck_amrs_action
            enabled_r=self.r_enable_haptic('False')
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))

        if not enabled_l and not enabled_r:
           succeeded=self.tuck_arms(tuck) #tuck the arms if Haptic MPC was successfully disabled
        elif not enabled_l and enabled_r:
            print "Left arm failed to disable haptic mpc"
        elif enabled_l and not enabled_r:
            print "Right arm failed to disable haptic mpc"
        else:
            print "Both arms failed to disable haptic mpc"

        if succeeded:  #check to see if the arms were tucked successfully 
            try:
                enabled_l=self.enable_haptic('True')  #Enable Haptic MPC after tucking the arms
                enabled_r=self.r_enable_haptic('True')
            except rospy.ServiceException as exc:
                print("Service did not process request: " + str(exc))
            return True
        elif not succeeded:
            print "Tuck arms failed"
            return False


    def tuck_arms (self, tuck):
        self.tuck_arms_client.wait_for_server(rospy.Duration(10.0))
        print "waiting for server"
        g=TuckArmsGoal()        
        g.tuck_left=tuck
        g.tuck_right=tuck
        self.tuck_arms_client.send_goal_and_wait(g, rospy.Duration(30.0), rospy.Duration(5.0))
        status=self.tuck_arms_client.get_result()
        print (status)
        if tuck:
            if status.tuck_left and status.tuck_right:
                print"Both arms tucked successfully"
                return True        
        elif not tuck:
            if not status.tuck_left and not status.tuck_right:
                print "Both arms untucked successfully"            
                return True
        print"Tucking arms action client failed"
        return False
            
  



if __name__=="__main__":
    a=tuckArmsNoMPC()
    while not (rospy.is_shutdown()):
        rospy.spin()

