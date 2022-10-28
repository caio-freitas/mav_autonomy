#!/usr/bin/env python

from sqlalchemy import true
import rospy
import smach
import math
import numpy as np
import smach_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from mavros_msgs.msg import State
from tf.transformations import quaternion_matrix
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
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.detections = 0
        self.TIMEOUT = 10 #s
        self.HEIGHT = 2


    def execute(self, userdata):
        rospy.loginfo("Going to position {}".format(userdata.pose_in))
        self.mav.takeoff(self.HEIGHT)

        self.mav.go_to(userdata.pose_in[0] ,userdata.pose_in[1] , self.HEIGHT)
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
        self.MAX_HEIGHT = 5
        self.detections = 0
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        

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
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.obj_pose = PoseWithCovarianceStamped()
        
        self.code = ""
        self.detection = Quaternion()

    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
        self.detection = data

    def execute(self, userdata):
        rospy.loginfo('Executing state CENTER')

        init_time = rospy.get_time()
        init_height = self.mav.drone_pose.pose.position.z

        while not rospy.get_time() - init_time >= 30: # centering
            # print(self.detection)
            # print(self.mav.drone_pose.pose)
            # cov = max(self.detection.pose.covariance[0],
            #           self.detection.pose.covariance[6],
            #           self.detection.pose.covariance[13],
            #           self.detection.pose.covariance[20])
            # print("cov: {}".format(cov))
            # if (cov > 0.1):
            #     self.mav.set_position(6,5,init_height)
            
            o_r_b = np.array([[self.mav.drone_pose.pose.position.x],
                                [self.mav.drone_pose.pose.position.y],
                                [self.mav.drone_pose.pose.position.z]])
            # print("o_r_b: {}".format(o_r_b))
            c_r_m = np.array([[self.detection.x],
                              [self.detection.y],
                              [self.detection.z]])
            # print("c_r_m: {}".format(c_r_m))
            o_R_b = quaternion_matrix([self.mav.drone_pose.pose.orientation.x,
                                        self.mav.drone_pose.pose.orientation.y,
                                        self.mav.drone_pose.pose.orientation.z,
                                        self.mav.drone_pose.pose.orientation.w])[:3,:3]
            # print("o_R_b: {}".format(o_R_b))
            b_R_c = np.array([[0, -1, 0],
                            [-1, 0, 0],
                            [ 0, 0, -1]])
            # print("b_R_c: {}".format(b_R_c))
            o_r_m = o_r_b + np.matmul(o_R_b, np.matmul(b_R_c, c_r_m))
            print("o_r_m: {}".format(o_r_m))
            goal_pose = o_r_m + np.array([[0],
                                        [0],
                                        [2]])
            # goal_pose[2] = init_height # overwrite with current height
            # print("goal_pose: {}".format(goal_pose))
            # rel_position = PoseStamped()
            # rel_position.pose.position.x = goal_pose[0]
            # rel_position.pose.position.y = goal_pose[1]
            # rel_position.pose.position.z = goal_pose[2]

            self.mav.set_position(*goal_pose)
            # self.mav.set_position(6,5,3)
            if rospy.is_shutdown():
                break
            e = (c_r_m[0] - 0)**2 + (c_r_m[1] - 0) **2 + (c_r_m[2] - 2)**2
            if e < 0.1:
                rospy.logwarn("e: {}".format(e))
                return "success"
        return "fail"
    
    def point_cb(self, data):
        self.detection = data
        self.last_detection_time = rospy.get_time()

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
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", PoseStamped, self.point_cb)

        self.obj_pose = PoseStamped()
        
        self.code = ""
        self.controller = ImageBasedController()
        # self.controller = IBVS()
        self.last_detection_time = rospy.get_time()
        self.detection = Quaternion()

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')
        init_height = self.mav.drone_pose.pose.position.z
        init_time = rospy.get_time()
        while not rospy.get_time() - init_time >= 10:
            o_r_b = np.array([[self.mav.drone_pose.pose.position.x],
                                [self.mav.drone_pose.pose.position.y],
                                [self.mav.drone_pose.pose.position.z]])
            print("o_r_b: {}".format(o_r_b))
            c_r_m = np.array([[self.detection.x],
                              [self.detection.y],
                              [self.detection.z]])
            print("c_r_m: {}".format(c_r_m))
            o_R_b = quaternion_matrix([self.mav.drone_pose.pose.orientation.x,
                                        self.mav.drone_pose.pose.orientation.y,
                                        self.mav.drone_pose.pose.orientation.z,
                                        self.mav.drone_pose.pose.orientation.w])[:3,:3]
            print("o_R_b: {}".format(o_R_b))
            b_R_c = np.array([[0, -1, 0],
                            [-1, 0, 0],
                            [ 0, 0, -1]])
            print("b_R_c: {}".format(b_R_c))
            o_r_m = o_r_b + np.matmul(o_R_b, np.matmul(b_R_c, c_r_m))
            print("o_r_m: {}".format(o_r_m))
            goal_pose = o_r_m + np.array([[0],
                                        [0],
                                        [2 - (rospy.get_time() - init_time)/10]])
            print("goal_pose: {}".format(goal_pose))

            yaw_d = self.detection.w 
            if self.detection.w > np.pi:
                yaw_d = self.detection.w - 2*np.pi

            self.mav.set_position_target(0b0001101111111000,
                                        goal_pose[0],
                                        goal_pose[1],
                                        goal_pose[2],
                                        yaw=0)
            # self.mav.set_position(*goal_pose)
            # self.mav.set_position(6,5,3)
            if rospy.is_shutdown():
                break
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