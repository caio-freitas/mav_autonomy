#!/usr/bin/env python

from sqlalchemy import true
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from controllers import ImageBasedController, IBVS
from MAV import MAV


obj_detected_before = False
last_position = PoseStamped()


# define state Search
class Search(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success','fail'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)
        self.detections = 0

    def execute(self, userdata):
        self.mav.takeoff(1.5)

        self.mav.go_to(6 ,5 ,4)

        while not self.detections >= 3:

            self.mav.go_to(6 ,5 ,5)
            self.mav.rate.sleep()
        return "success"
    
    def point_cb(self, data):
        self.detections += 1

# define state Approach
class Approach(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)



        self.obj_pose = PoseStamped()
        
        self.code = ""
        self.controller = ImageBasedController()
        # self.controller = IBVS()
        self.detection = Quaternion()

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')

        init_time = rospy.get_time()
        area_ratio = 0.05
        if self.detection.z != 0:
            area_ratio = self.detection.z
            rospy.loginfo("Setting initial area_ratio: {}".format(area_ratio))
        self.controller.set_target([400, 480, area_ratio, 0]) # center image
        # print(self.controller.s_d)
        while not rospy.get_time() - init_time >= 10: # centering

            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            if self.detection != Quaternion():
                input = self.controller.calc_input()
                self.mav.set_vel(input[0], input[1], 0, yaw = input[3])
                # self.mav.set_vel(0,0,0)
            else:
                self.mav.set_vel(0,0,0)
            if rospy.is_shutdown():
                break

        while self.detection.z < 0.3: # approaching
            area_ratio = min(area_ratio + 0.0002, 1)
            self.controller.set_target([400, 480, area_ratio, 0])
            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            if self.detection != Quaternion():
                input = self.controller.calc_input()
                self.mav.set_vel(input[0], input[1], input[2], yaw = input[3])
            else:
                self.mav.set_vel(0,0,0)

            self.mav.rate.sleep()
            if rospy.is_shutdown():
                break

        self.mav.set_vel(0, 0, -0.5)
        # self.mav._disarm()
        return 'success'
        
    def visp_callback(self, data):
        self.obj_pose = data.pose

    def code_callback(self, data):
        self.code = data.data
    
    def point_cb(self, data):
        self.detection = data

# define state Land
class Land(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.mav = mav
    
    def execute(self, userdata):
        rospy.loginfo('Executing state LAND')
        self.mav.land()
        return 'success'
        




def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done'])

    # Open the container
    with sm:
        mav = MAV("giuseppe")
        # Add states to the container

        smach.StateMachine.add('SEARCH', Search(mav), 
                                transitions={'success':'APPROACH', 'fail':'done'})
        smach.StateMachine.add('APPROACH', Approach(mav), 
                                transitions={'success':'LAND'})
        smach.StateMachine.add('LAND', Land(mav), 
                                transitions={'success':'done'})
        

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()