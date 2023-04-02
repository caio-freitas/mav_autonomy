#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
import apriltag
from cv_bridge import CvBridge
import cv2
import numpy as np

bridge = CvBridge()

detector = apriltag.Detector()

img_pub = rospy.Publisher("apritag_detector/debug/image_raw", Image, queue_size=1)

point_pub = rospy.Publisher("apritag_detector/detection", Quaternion, queue_size=10)


def img_cb(msg):
    global detector
    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    IM_SIZE = gray.shape[0]*gray.shape[1]
    results = detector.detect(gray)
    for r in results:
        (ptA, ptB, ptC, ptD) = r.corners
        ptB = np.array([int(ptB[0]), int(ptB[1])])
        ptC = np.array([int(ptC[0]), int(ptC[1])])
        ptD = np.array([int(ptD[0]), int(ptD[1])])
        ptA = np.array([int(ptA[0]), int(ptA[1])])

        # draw the bounding box of the AprilTag detection
        cv2.line(gray, ptA, ptB, (200, 200, 200), 4)
        cv2.line(gray, ptB, ptC, (200, 200, 200), 4)
        cv2.line(gray, ptC, ptD, (200, 200, 200), 4)
        cv2.line(gray, ptD, ptA, (200, 200, 200), 4)

        v1 = ptA - ptB
        v2 = ptB - ptC
        
        # TODO investigate why it's so diffeent from 
        A = np.cross(v1, v2)/(IM_SIZE)
        
        # sort points to determine angle
        pts = np.array([ptA, ptB, ptC, ptD])
        # pts = pts[pts[:,1].argsort()] #sort by Y coordinate
        phi = np.arctan2(*(pts[3][::-1] - pts[2][::-1]))

        

        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(r.center[0]), int(r.center[1]))
        cv2.circle(gray, (cX, cY), 5, (200, 200, 200), -1)


        # print("[INFO] center: {}, {}".format(cX, cY))
    # show the output image after AprilTag detection
    image_message = bridge.cv2_to_imgmsg(gray, encoding="mono8")
    img_pub.publish(image_message)
    if len(results)>0:
        point_pub.publish(Quaternion(cX, cY, A, phi))

if __name__ == '__main__':
    rospy.init_node('mav_obj_tf_broadcaster')
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, img_cb)
    # rospy.Subscriber('/usb_cam/image_raw', Image, img_cb)
    
    rospy.spin()