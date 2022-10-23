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
last_position = PoseStamped()


# define state Search
class GoToPose(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, 
                            outcomes=['detected','detected_before', 'not_detected'],
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

class Search(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.mav = mav
        self.MAX_HEIGHT = 10
        self.detections = 0
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)
        

    def execute(self, userdata):
        rospy.loginfo('Executing state SEARCH')
        while self.mav.drone_pose.pose.position.z < self.MAX_HEIGHT:
            if self.detections >= 3:
                return "success"
            self.mav.set_position(self.mav.drone_pose.pose.position.x,
                                    self.mav.drone_pose.pose.position.y,
                                    self.mav.drone_pose.pose.position.z + 15/100)
            self.mav.rate.sleep()
        # TODO implement rest of search
        return "fail"

    def point_cb(self, data):
        global last_position
        self.detections += 1
        last_position = self.mav.drone_pose

class GoToLastDetection(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.mav = mav

    def execute(self):
        rospy.loginfo('Executing state GO_TO_LAST_DETECTION')
        self.mav.go_to(last_position)
        return "success"

    
class Center(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.mav = mav
        self.controller_type = rospy.get_param("controller_type")
        setpoint_x = rospy.get_param("setpoint_features_x")
        setpoint_y = rospy.get_param("setpoint_features_y")
        setpoint_z = rospy.get_param("setpoint_features_z")
        setpoint_yaw = rospy.get_param("setpoint_features_yaw")

        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.obj_pose = PoseStamped()
        
        self.code = ""
        if self.controller_type == "image_uncoupled":
            self.controller = ImageBasedController()
        elif self.controller_type == "IBVS":
            self.controller = IBVS()
        
        self.controller.set_target([setpoint_x,
                                    setpoint_y,
                                    setpoint_z,
                                    setpoint_yaw])
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
        

        while not rospy.get_time() - init_time >= 30: # centering

            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], 0)
            if rospy.is_shutdown():
                break
            e = math.sqrt((self.detection.x - 400)**2 + (self.detection.y - 480)**2)
            if e < 20:
                return "success"
        return "fail"
    
class RTL(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.mav = mav

    def execute(self, userdata):
        rospy.loginfo('Executing state RTL')
        self.mav.RTL()
        return "success"


# define state Approach
class Approach(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'object_losted'])
        self.controller_type = rospy.get_param("controller_type")
        self.setpoint_x = rospy.get_param("setpoint_features_x")
        self.setpoint_y = rospy.get_param("setpoint_features_y")
        self.setpoint_z = rospy.get_param("setpoint_features_z")
        self.setpoint_yaw = rospy.get_param("setpoint_features_yaw")
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)

        self.obj_pose = PoseStamped()
        
        self.code = ""
        if self.controller_type == "image_uncoupled":
            self.controller = ImageBasedController()
        elif self.controller_type == "IBVS":
            self.controller = IBVS()
        self.controller.set_target([self.setpoint_x,
                                    self.setpoint_y,
                                    self.setpoint_z,
                                    self.setpoint_yaw])
        self.last_detection_time = rospy.get_time()
        self.detection = Quaternion()

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')
        area_ratio = 0.05
        if self.detection.z != 0:
            area_ratio = self.detection.z
            rospy.loginfo("Setting initial area_ratio: {}".format(area_ratio))
        while self.detection.z < 0.3: # approaching
            area_ratio = min(area_ratio + 0.0004, 1)
            self.controller.set_target([self.setpoint_x, self.setpoint_y, area_ratio, self.setpoint_yaw])
            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], input[2], yaw = input[3])

            self.mav.rate.sleep()
            if rospy.is_shutdown():
                break
        # self.mav._disarm()
        return 'success'
        # TODO if no tag is detected, go to other state
    
    def point_cb(self, data):
        self.detection = data
        self.last_detection_time = rospy.get_time()

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
    sm = smach.StateMachine(outcomes=['done', 'aborted'],
                            input_keys=['obj_pose'])
    sm.userdata.obj_pose = [6, 5]

    # Open the container
    with sm:
        mav = MAV("giuseppe")
        # Add states to the container
        smach.StateMachine.add('GO_TO_POSE', GoToPose(mav), 
                                transitions={'detected':'CENTER',
                                            'detected_before':'GO_TO_LAST_DETECTION',
                                            'not_detected': 'SEARCH'},
                                remapping={'pose_in': 'obj_pose'})

        smach.StateMachine.add('GO_TO_LAST_DETECTION', GoToLastDetection(mav), 
                                transitions={'success':'CENTER', 'fail':'RTL'})

        smach.StateMachine.add('SEARCH', Search(mav), 
                                transitions={'success':'CENTER', 'fail':'RTL'})

        smach.StateMachine.add('CENTER', Center(mav), 
                                transitions={'success':'APPROACH', 'fail': 'SEARCH'})

        smach.StateMachine.add('APPROACH', Approach(mav), 
                                transitions={'success':'LAND', 'object_losted': 'GO_TO_LAST_DETECTION'})
        
        smach.StateMachine.add('LAND', Land(mav), 
                                transitions={'success':'done'})

        smach.StateMachine.add('RTL', RTL(mav), 
                                transitions={'success':'aborted'})
        

    # Execute SMACH plan
    outcome = sm.execute()

if __name__ == '__main__':
    main()