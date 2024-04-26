#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
import cv2
from PIL import Image
import numpy as np

class ball:

    def __init__(self):
        
        rospy.init_node('ball_pose')
        self.pub = rospy.Publisher('/ball_pose/2D', PoseStamped, queue_size = 10)
        self.rate = rospy.Rate(30)


    def get_limits(self, color):

        c = np.uint8([[color]])
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        lowerLimit = hsvC[0][0][0] - 10, 100, 100
        upperLimit = hsvC[0][0][0] + 10, 255, 255
        lowerLimit = np.array(lowerLimit, dtype = np.uint8)
        upperLimit = np.array(upperLimit, dtype = np.uint8)

        return lowerLimit, upperLimit


    def translate(self, value, leftMin, leftMax, rightMin, rightMax):

        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)
        
        return rightMin + (valueScaled * rightSpan)


    def get_position(self):

        cap = cv2.VideoCapture("/dev/video2")
        yellow = [0, 255, 255]
        lowerLimit, upperLimit = self.get_limits(yellow)
        pose = PoseStamped()
        
        while not rospy.is_shutdown():

            ret, frame = cap.read()
            hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
            mask_ = Image.fromarray(mask)
            bbox = mask_.getbbox()

            if bbox is not None:
                
                x1, y1, x2, y2 = bbox
                frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
                x = self.translate(((x1 + x2) / 2), 0, 640, -20, 20)
                y = self.translate(((y1 + y2) / 2), 0, 480, -20, 20)

                pose.header.stamp = rospy.Time.now()
                pose.header.frame_id = "world"
                pose.pose.position.x = x
                pose.pose.position.y = y

                self.pub.publish(pose)

                self.rate.sleep()

            cv2.imshow('mask', mask)
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                
                rospy.signal_shutdown("Spacebar pressed --> Shutdown hook activated")
                break

        cap.release()
        cv2.destroyAllWindows()

if __name__ == "__main__":

    obj = ball()
    obj.get_position() 