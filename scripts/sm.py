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
rospy.init_node('state_machine')
global_init_time = rospy.get_time()


# define state Search
class GoToPose(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, 
                            outcomes=['detected','detected_before', 'not_detected'],
                            input_keys=['pose_in'])
                        
        global global_init_time
        global_init_time = rospy.get_time()

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

        self.mav.go_to(userdata.pose_in[0] ,userdata.pose_in[1] , self.HEIGHT, 2.8)
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
        global global_init_time
        if rospy.get_time() - global_init_time >= 180:
            rospy.logerr("Aborting attempt after {} seconds".format(rospy.get_time() - global_init_time))
            return "fail"
        self.state_pub.publish("GoToLastDetection")
        rospy.loginfo('Executing state GO_TO_LAST_DETECTION')
        z = max(last_position.pose.position.z, 1)
        self.mav.go_to(last_position.pose.position.x, last_position.pose.position.y, z, avg_vel = 0.2)
        return "success"

class Center(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'fail'])
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)

        self.mav = mav
        self.controller_type = rospy.get_param("controller_type")
        self.setpoint_x = rospy.get_param("setpoint_features_x")
        self.setpoint_y = rospy.get_param("setpoint_features_y")
        self.setpoint_z = rospy.get_param("setpoint_features_z")
        self.setpoint_yaw = rospy.get_param("setpoint_features_yaw")

        self.point_sub = rospy.Subscriber("/apritag_detector/detection", Quaternion, self.point_cb)
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
        self.detection = Quaternion()
        self.last_detection_time = rospy.get_time()

    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
        self.detection = data
        self.last_detection_time = rospy.get_time()

    def execute(self, userdata):
        self.state_pub.publish("Center")
        rospy.loginfo('Executing state CENTER')

        init_time = rospy.get_time()
        area_ratio = 0.05
        if self.detection.z != 0:
            area_ratio = self.detection.z
            rospy.loginfo("Setting initial area_ratio: {}".format(area_ratio))
        
        self.controller.set_target([self.setpoint_x,
                                    self.setpoint_y,
                                    area_ratio,
                                    self.setpoint_yaw])
        while not rospy.get_time() - init_time >= 30: # centering
            
            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], input[2], yaw=input[3])
            # self.mav.set_vel(input[0], input[1], input[2], roll=0, pitch=0, yaw=input[5])
            if rospy.is_shutdown():
                break
            e = math.sqrt((self.detection.x - 400)**2 + (self.detection.y - 480)**2) + 180*self.detection.w/math.pi

            if e <= 20:
                return "success"
            
            if rospy.get_time() - self.last_detection_time >= 1:
                return 'fail'
        return "fail"
    
class RTL(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        
        self.mav = mav

    def execute(self, userdata):
        self.state_pub.publish("RTL")
        rospy.loginfo('Executing state RTL')
        self.mav.go_to(0,0,self.mav.drone_pose.pose.position.z, 2)
        self.mav.RTL()
        return "success"


# define state Approach
class Approach(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success', 'object_losted'])
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        
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
        self.state_pub.publish("Approach")
        rospy.loginfo('Executing state APPROACH')
        area_ratio = 0.05
        if self.detection.z != 0:
            area_ratio = self.detection.z
            rospy.loginfo("Setting initial area_ratio: {}".format(area_ratio))
        az = math.sqrt(area_ratio)
        while self.detection.z < 0.2: # approaching
            # rospy.loginfo(az)
            az += 0.0002
            area_ratio = az**2
            # area_ratio = 0.2
            self.controller.set_target([self.setpoint_x, self.setpoint_y, area_ratio, self.setpoint_yaw])
            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = 1.1*self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], input[2], yaw = input[3])
            # self.mav.set_vel(input[0], input[1], input[2], roll=input[3], pitch=input[4], yaw=input[5])
            

            self.mav.rate.sleep()
            if rospy.is_shutdown():
                break
            if rospy.get_time() - self.last_detection_time >= 1:
                return 'object_losted'
        e = math.sqrt((self.detection.x - 400)**2 + (self.detection.y - 480)**2)
        while e > 10 and rospy.get_time() - self.last_detection_time <= 0.2:
            e = math.sqrt((self.detection.x - 400)**2 + (self.detection.y - 480)**2)
            area_ratio = 0.3
            self.controller.set_target([self.setpoint_x, self.setpoint_y, area_ratio, self.setpoint_yaw])
            self.controller.update_measurements(self.detection, self.mav.drone_pose.pose.position.z)
            input = 1.6*self.controller.calc_input()
            self.mav.set_vel(input[0], input[1], -0.1)
            self.mav.rate.sleep()
        return 'success'
    
    def point_cb(self, data):
        global last_position
        last_position = self.mav.drone_pose
        self.detection = data
        self.last_detection_time = rospy.get_time()
        
# define state Land
class Land(smach.State):
    def __init__(self, mav):
        smach.State.__init__(self, outcomes=['success'])
        self.state_pub = rospy.Publisher("/state_transitions", String, queue_size=10)
        
        self.mav = mav
    
    def execute(self, userdata):
        self.state_pub.publish("Land")
        rospy.loginfo('Executing state LAND')
        self.mav.land(0.2)
        return 'success'
        




def main():
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['done', 'aborted'],
                            input_keys=['obj_pose'])
    sm.userdata.obj_pose = [5.8, 4.8]

    # sm.userdata.obj_pose = [2, -2]

    # Open the container
    with sm:
        mav = MAV("giuseppe")
        global_init_time = rospy.get_time()


        # Add states to the container
        smach.StateMachine.add('GO_TO_POSE', GoToPose(mav), 
                                transitions={'detected':'CENTER',
                                            'detected_before':'GO_TO_LAST_DETECTION',
                                            'not_detected': 'SEARCH'},
                                remapping={'pose_in': 'obj_pose'})

        smach.StateMachine.add('GO_TO_LAST_DETECTION', GoToLastDetection(mav), 
                                transitions={'success':'CENTER','fail':'RTL'})

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