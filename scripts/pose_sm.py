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




class GoToPose(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, 
                            outcomes=['detected','detected_before', 'not_detected'],
                            input_keys=['pose_in'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)

        self.detections = 0
        self.TIMEOUT = 10 #s
        self.HEIGHT = 5


    def execute(self, userdata):
        self.state_pub.publish("GoToPose")
        rospy.loginfo('Executing state GO_TO_POSE')
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
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        self.detections = 0
        self.mav = mav
        self.MAX_HEIGHT = 10
        self.r_size = 1 # m
        self.point_sub = rospy.Subscriber("apritag_detector/detection", Quaternion, self.point_cb)
        

    def execute(self, userdata):
        self.detections = 0
        self.state_pub.publish("Search")
        rospy.loginfo('Executing state SEARCH')
        init_x = self.mav.drone_pose.pose.position.x
        init_y = self.mav.drone_pose.pose.position.y
        init_z = self.mav.drone_pose.pose.position.z

        while self.mav.drone_pose.pose.position.z < self.MAX_HEIGHT:
            if self.detections >= 3:
                return "success"
            self.mav.set_position(init_x,
                                init_y,
                                self.mav.drone_pose.pose.position.z + 15/100)
            self.mav.rate.sleep()
        init_time = rospy.get_time()
        finished = False
        for i in range(1, 4):
            if (self.go_to_step(init_x - i*self.r_size,
                            init_y - (i + 1)*self.r_size,
                            self.MAX_HEIGHT, 0.5)):
                return "success"
            
            if (self.go_to_step(init_x - i*self.r_size,
                            init_y + i*self.r_size,
                            self.MAX_HEIGHT, 0.5)):
                return "success"
            
            if (self.go_to_step(init_x + i*self.r_size,
                            init_y + i*self.r_size,
                            self.MAX_HEIGHT, 0.5)):
                return "success"
            
            if (self.go_to_step(init_x + i*self.r_size,
                            init_y - i*self.r_size,
                            self.MAX_HEIGHT, 0.5)):
                return "success"
        
        return "fail"

    def go_to_step(self, x_goal, y_goal, z_goal, avg_vel=0.8):
        """
        Goes to goal cartesian position in linear trajectory with
        predefined average velocity avg_vel
        """

        x_init = self.mav.drone_pose.pose.position.x
        y_init = self.mav.drone_pose.pose.position.y
        z_init = self.mav.drone_pose.pose.position.z

        T = math.sqrt((x_goal - x_init)**2 + (y_goal - y_init)**2 + (z_goal - z_init)**2)/avg_vel
        init_t = rospy.get_time()
        t = 0
        while t <= T:
            self.mav.set_position(x_init + (t/T)*(x_goal - x_init),
                                y_init + (t/T)*(y_goal - y_init),
                                z_init + (t/T)*(z_goal - z_init))
            t = rospy.get_time() - init_t
            if self.detections >= 3:
                return True
            self.mav.rate.sleep()
        self.mav.set_position(x_goal, y_goal, z_goal)
        return False

    def point_cb(self, data):
        global last_position
        self.detections += 1
        last_position = self.mav.drone_pose

class GoToLastDetection(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        
        self.mav = mav

    def execute(self, userdata):
        self.state_pub.publish("GoToLastDetection")
        rospy.loginfo('Executing state GO_TO_LAST_DETECTION')
        z = max(last_position.pose.position.z, 1)
        self.mav.go_to(last_position.pose.position.x, last_position.pose.position.y, z, avg_vel = 0.2)
        return "success"

    
class Center(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.mav = mav
        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
        self.obj_pose = PoseWithCovarianceStamped()
        
        self.code = ""
        self.detection = Quaternion()
        self.b_R_c = np.array([[0, -1, 0],
                            [-1, 0, 0],
                            [ 0, 0, -1]])

    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
        self.detection = data

    def execute(self, userdata):
        rospy.loginfo('Executing state CENTER')

        init_time = rospy.get_time()
        init_height = self.mav.drone_pose.pose.position.z

        while not rospy.get_time() - init_time >= 30: # centering
            
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
            
            o_r_m = o_r_b + np.matmul(o_R_b, np.matmul(self.b_R_c, c_r_m))
            print("o_r_m: {}".format(o_r_m))
            goal_pose = o_r_m + np.array([[0],
                                        [0],
                                        [o_r_b[2][0]]])
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
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        
        self.mav = mav

    def execute(self, userdata):
        self.state_pub.publish("RTL")
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
        self.b_R_c = np.array([[0, -1, 0],
                            [-1, 0, 0],
                            [ 0, 0, -1]])

    def execute(self, userdata):
        rospy.loginfo('Executing state APPROACH')
        init_height = self.mav.drone_pose.pose.position.z
        init_time = rospy.get_time()
        goal_pose = np.ones(3,1)
        while goal_pose[2] > 0.3:
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
            o_r_m = o_r_b + np.matmul(o_R_b, np.matmul(self.b_R_c, c_r_m))
            print("o_r_m: {}".format(o_r_m))
            goal_pose = o_r_m + np.array([[0],
                                        [0],
                                        [init_height - (rospy.get_time() - init_time)/10]])

            print("goal_pose: {}".format(goal_pose))
            yaw_d = self.detection.w 
            if self.detection.w > np.pi:
                yaw_d = self.detection.w - 2*np.pi

            self.mav.set_position_target(0b0001101111111000,
                                        goal_pose[0]+0.15,
                                        goal_pose[1],
                                        goal_pose[2],
                                        yaw=-yaw_d)
            
            if rospy.is_shutdown():
                break
            if rospy.get_time() - self.last_detection_time >= 1:
                return 'object_losted'
            
        return "success"
        # TODO if no tag is detected, go to other state
    
    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
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
    sm.userdata.obj_pose = [5.8, 4.8]

    # sm.userdata.obj_pose = [2, -2]

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
                                transitions={'success':'CENTER','fail':'SEARCH'})

        smach.StateMachine.add('SEARCH', Search(mav), 
                                transitions={'success':'CENTER', 'fail':'RTL'})

        smach.StateMachine.add('CENTER', Center(mav), 
                                transitions={'success':'APPROACH', 'fail': 'GO_TO_LAST_DETECTION'})

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