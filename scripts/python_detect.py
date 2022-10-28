#!/usr/bin/env python
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
import apriltag

LINE_LENGTH = 5
CENTER_COLOR = (0, 255, 0)
CORNER_COLOR = (255, 0, 255)

#Camera Constants
VIDEO_DEV = 0 #Video Device ID for the camera used. Probably 0 or 1 for Webcam, 2 or 3 for internal if on laptop and more than one device
FRAME_HEIGHT = 480 #Height of the camera being used
FRAME_WIDTH = 640 #Width of the camera being used
FRAME_RATE = 30 #Desired Frame Rate


TAG_SIZE = .3 #Tag size in meters

#Camera Information thats needed for solvePnp
camInfo = np.matrix([[692.978391,   0,         320.5],
 [  0,         692.978391, 240.5  ],
 [  0,           0,           1.0        ]]) #FIXME For when you use this you will need to run camera_calib unless you want stuff to be really wrong

#Distsortion Coefficients for the camera being used
distCoeff = np.matrix([[0,  0, 0, 0, 0]]) #FIXME For when you use this you will need to run camera_calib unless you want stuff to be really wrong

bridge = CvBridge()

img_pub = rospy.Publisher("apritag_detector/debug/image_raw", Image, queue_size=1)

point_pub = rospy.Publisher("apritag_detector/detection", Quaternion, queue_size=10)




#Plots points and draws lines for the corner of the tags
def plotPoint(image, center, color):
    center = (int(center[0]), int(center[1]))
    image = cv2.line(image,
                     (center[0] - LINE_LENGTH, center[1]),
                     (center[0] + LINE_LENGTH, center[1]),
                     color,
                     3)
    image = cv2.line(image,
                     (center[0], center[1] - LINE_LENGTH),
                     (center[0], center[1] + LINE_LENGTH),
                     color,
                     3)
    return image

#Plots the tag ID in the center of the tag
def plotText(image, center, color, text):
    center = (int(center[0]) + 4, int(center[1]) - 4)
    return cv2.putText(image, str(text), center, cv2.FONT_HERSHEY_SIMPLEX,
                       1, color, 3)

#Initializes the AprilTag detector and sets the Camera ID
detector = apriltag.Detector()
cam = cv2.VideoCapture(VIDEO_DEV) #ID of the camera being used

#Sets the resolution and frame rate settings
cam.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
cam.set(cv2.CAP_PROP_POS_FRAMES, FRAME_RATE)

looping = True #Starts looping the fun stuff

def img_cb(msg):
    global detector
    image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
    grayimg = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

	# look for tags
    detections = detector.detect(grayimg)
    if not detections:
        # NT.putString("tagfound", 0)
        print("No Tag found.  Looking for tags")
    else:
        for detect in detections:
            ##print("\ntag_id: %s, center-yx: %s" % (detect.tag_id, detect.center))
            #print("\ntag-id: %s center-x: %s \ntag-id: %s center-y: %s" % (detect.tag_id, detect.center[1], detect.tag_id, detect.center[0])) #Prints the tag ID and the center coordinates of the tag

            #Detects the center of the tag and outputs the X and Y coordinates induvidually
            centerX = detect.center[0]
            centerY = detect.center[1]

            #Makes the X and Y coordinates relative to the center of the frame
            centerOriginX = (centerX - (FRAME_WIDTH / 2))
            centerOriginY = ((FRAME_HEIGHT / 2) - centerY)

            #Debug stuff for outputting the center of the tag
            ##print("\nX-Axis:", centerOriginX, "\n") #Debug
            ##print("Y-Axis:", centerOriginY, "\n") #Debug

            ##print("\ntag-id:", detect.tag_id, "center-x:", centerX) #Debug
            ##print("tag-id:", detect.tag_id, "center-y:", centerY) #Debug

            #Plots the tag ID and a cross-hair in the center of the tag
            image = plotPoint(image, detect.center, CENTER_COLOR)
            image = plotText(image, detect.center, CENTER_COLOR, detect.tag_id)

            #Plots points in the corners of the tag
            for corner in detect.corners:
                image = plotPoint(image, corner, CORNER_COLOR)

        #Stuff needed for SolvePnP
        halfTagSize = TAG_SIZE/2
        objectPoints= np.array([ [-halfTagSize,halfTagSize, 0], [ halfTagSize, halfTagSize, 0], [ halfTagSize, -halfTagSize, 0], [-halfTagSize, -halfTagSize, 0] ])
        SOLVEPNP_IPPE_SQUARE =7 # (enumeration not working: 
        # https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga357634492a94efe8858d0ce1509da869)

        for d in detections:
                
            ##print(d['lb-rb-rt-lt']) #Debug
            imagePoints = np.array([detect.corners]) #Outputs corners of tag as array
            ##print(imagePoints) #Debug

            # solvePnP docs: https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d

            #solvePNP returns rotation and translation vectors
            retval, rvec, tvec  = cv2.solvePnP(objectPoints, imagePoints, camInfo, distCoeff, useExtrinsicGuess=False, flags=SOLVEPNP_IPPE_SQUARE)
            ##print(cv2.solvePnP(objectPoints, imagePoints, camInfo, distCoeff, useExtrinsicGuess=False, flags=SOLVEPNP_IPPE_SQUARE))
            #print("rvec:", rvec)
            #print("tvec:", tvec)
            R = cv2.Rodrigues(tvec)[0]
            
            # #print("R:", R)

            #Convert to yaw, pitch, roll
            # yaw = np.arctan2(R[0,2],R[2,2])*180/np.pi # 180//np.pi gets to integers?
            # pitch = np.arcsin(-R[1][2])*180/np.pi
            # roll = (np.arctan2(R[1,0],R[1,1])*180/np.pi) + 180
            roll = np.arctan2(R[1,0],R[1,1])
            q = Quaternion(tvec[0][0], tvec[1][0], tvec[2][0], roll)
            #print(q)
            point_pub.publish(q)

            #This stuff only outputs in euler angles and should probably be removed but am keeping for debugging purposes
            # rvec_matrix = cv2.Rodrigues(rvec)[0] #Debug
            # proj_matrix = np.hstack((rvec_matrix, tvec)) #Debug
            # eulerAngles = -cv2.decomposeProjectionMatrix(proj_matrix)[6] #Debug
            # yaw   = eulerAngles[1] #Debug
            # pitch = eulerAngles[0] #Debug
            # roll  = eulerAngles[2] #Debug

            #Output yaw, pitch, roll values to command line
            #print("\nYaw", yaw)
            #print("pitch", pitch)
            #print("roll", roll)

            #Output yaw, pitch, roll values to NetworkTables
            # NT.putString("yaw", yaw)
            # NT.putString("pitch", pitch)
            # NT.putString("roll", roll)

    #Output window with the live feed from the camera and overlays
    cv2.imshow('Vid-Stream', image) #Comment out when running in headless mode to not piss off python

    #Defines enter key and a 100ms delay
    key = cv2.waitKey(100)

    #If the enter key is pressed exit the program
    if key == 13:
        looping = False     

    #cameraServer() #Uncomment if you want things to break


if __name__ == '__main__':
    rospy.init_node('mav_obj_tf_broadcaster')
    rospy.Subscriber('/iris/usb_cam/image_raw', Image, img_cb)
    # rospy.Subscriber('/usb_cam/image_raw', Image, img_cb)
    
    rospy.spin()