#!/usr/bin/env python

from sqlalchemy import true
import rospy
import smach
import math
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest
from controllers import ImageBasedController, IBVS
from MAV import MAV


obj_detected_before = False
last_position = Quaternion()
last_detection = 0
last_height = 0

# define state Search
class GoToPose(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, 
                            outcomes=['detected'],
                            input_keys=['pose_in'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)
        self.detections = 0
        self.TIMEOUT = 10 #s
        self.HEIGHT = 5


    def execute(self, userdata):
        rospy.loginfo("Going to position {}".format(userdata.pose_in))
        self.mav.takeoff(self.HEIGHT)

        self.mav.go_to(userdata.pose_in[0] ,userdata.pose_in[1] , self.HEIGHT, 1.2)
        init_time = rospy.get_time()
        while rospy.get_time() - init_time < self.TIMEOUT: # 10s timeout   
            self.mav.set_position(userdata.pose_in[0] ,userdata.pose_in[1] , self.HEIGHT)
            if self.detections >= 3:
                return "detected"
            self.mav.rate.sleep()
        
        if obj_detected_before:
            return "detected_before"
        return "not_detected"

    def point_cb(self, data):
        global last_position
        self.detections += 1
        last_position = self.mav.drone_pose

    
class Center(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['done'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.obj_pose = PoseStamped()
        
        self.code = ""
        self.controller = ImageBasedController()
        # self.controller = IBVS()
        self.detection = Quaternion()

    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
        self.detection = data

    def execute(self, userdata):
        rospy.loginfo('Executing state CENTER')

        init_time = rospy.get_time()
        area_ratio = 0.05
        if self.detection.z != 0:
            area_ratio = self.detection.z
            rospy.loginfo("Setting initial area_ratio: {}".format(area_ratio))
        self.controller.set_target([400, 400, area_ratio, 0]) # center image

        while not rospy.get_time() - init_time >= 10: # centering

            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], 0, yaw = input[3])
            if rospy.is_shutdown():
                break
            e = math.sqrt((self.detection.x - 400)**2 + (self.detection.y - 400)**2)
            if e < 20:
                return "done"
        return "done"

    
class Rise(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['done'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.obj_pose = PoseStamped()
        
        self.code = ""
        self.controller = ImageBasedController()
        # self.controller = IBVS()
        self.detection = Quaternion()

    def point_cb(self, data):
        global last_detection, last_height
        last_detection = rospy.get_time()
        self.detection = data
        last_height = self.mav.drone_pose.pose.position.z

    def execute(self, userdata):
        global last_detection, last_height
        rospy.loginfo('Executing state RISE')

        init_time = rospy.get_time()
        
        init_x = self.mav.drone_pose.pose.position.x
        init_y = self.mav.drone_pose.pose.position.y
        height = self.mav.drone_pose.pose.position.z
        while rospy.get_time() - last_detection < 1: #1 s
            self.mav.set_position(init_x, init_y, height)
            height += 1/100 # 1 m/s
            self.mav.rate.sleep()
            
        rospy.logwarn("Height of last detection: {last_height}")
        return "done"
    
class RTL(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.mav = mav

    def execute(self, userdata):
        rospy.loginfo('Executing state RTL')
        self.mav.RTL()
        return "success"





def main():
    rospy.init_node('state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'aborted'],
                            input_keys=['obj_pose'])
    sm.userdata.obj_pose = [6, 5]

    # Open the container
    with sm:
        mav = MAV("giuseppe")
        # Add states to the container
        smach.StateMachine.add('GO_TO_POSE', GoToPose(mav), 
                                transitions={'detected':'CENTER'},
                                remapping={'pose_in': 'obj_pose'})


        smach.StateMachine.add('CENTER', Center(mav), 
                                transitions={'done':'RISE'})


        smach.StateMachine.add('RISE', Rise(mav), 
                                transitions={'done':'RTL'})


        smach.StateMachine.add('RTL', RTL(mav), 
                                transitions={'success':'aborted'})
        

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()