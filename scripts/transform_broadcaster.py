#!/usr/bin/env python
import rospy
import tf
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped


def pub_mavros_tf(msg):
    br = tf.TransformBroadcaster()
#     q1 = (msg.pose.orientation.x,
#           msg.pose.orientation.y,
#           msg.pose.orientation.z,
#           msg.pose.orientation.w)
#     p1 = (msg.pose.position.x,
#             msg.pose.position.y,
#             msg.pose.position.z)
#     br.sendTransform(p1,
#                     q1,
#                     rospy.Time.now(),
#                     "base_link",
#                     "map")

#     br.sendTransform((0, 0.1, 0.1),
#                 quaternion_from_euler(0, 1.5708, 0),
#                 rospy.Time.now(),
#                 "robot_camera_link",
#                 "base_link")

def pub_visp_tf(msg):
    br = tf.TransformBroadcaster()
    q1 = (msg.pose.orientation.x,
          msg.pose.orientation.y,
          msg.pose.orientation.z,
          msg.pose.orientation.w)
    p1 = (msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z)
    br.sendTransform(p1,
                    q1,
                    rospy.Time.now(),
                    "map",
                    "base_link")
    br.sendTransform((0, 0.1, 0.1),
                quaternion_from_euler(0, 1.5708, 0),
                rospy.Time.now(),
                "robot_camera_link",
                "base_link")

if __name__ == '__main__':
    rospy.init_node('mav_obj_tf_broadcaster')
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pub_mavros_tf)
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, pub_visp_tf)
    
    rospy.spin()