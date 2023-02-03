#!/usr/bin/env python

import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from controllers import ImageBasedController, IBVS
from MAV import MAV

# define state GoToGoal
class GoToGoal(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.mav = mav
        
    def execute(self, userdata):
        goal_pose = PoseStamped()
        goal_pose.pose.position.x = 0
        goal_pose.pose.position.y = 0
        goal_pose.pose.position.z = 6

        
        self.mav.takeoff(4)
        while not self.mav.chegou(goal_pose):
            self.mav.set_position(0,0,6)
            self.mav.rate.sleep()

        self.mav.go_to(-6 ,-5 ,6)

        goal_pose.pose.position.x = -6
        goal_pose.pose.position.y = -5
        goal_pose.pose.position.z = 6


        while not self.mav.chegou(goal_pose):
            # self.mav.set_position(-3, 5, 5)
            self.mav.set_position(-6, -5, 6)
            # self.mav.go_to(0 ,0 ,5)
            self.mav.rate.sleep()
        return "success"
    


# define state Land
class Land(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.mav = mav
    
    def execute(self, userdata):
        rospy.loginfo('Executing state LAND')
        self.mav.go_to(-6, -5, 2)
        self.mav.land_with_obj()
        return 'success'
        




def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
        mav = MAV("giuseppe")
        # Add states to the container
        smach.StateMachine.add('GO_TO_GOAL', GoToGoal(mav), 
                                transitions={'success':'LAND', 'fail':'done'})
        smach.StateMachine.add('LAND', Land(mav), 
                                transitions={'success':'done'})
        

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()