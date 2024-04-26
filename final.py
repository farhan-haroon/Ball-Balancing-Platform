#!/usr/bin/env python3

import numpy as np
import cv2
from PIL import Image
import rospy
from geometry_msgs.msg import PoseStamped

class ball:

    def __init__(self):

        rospy.init_node('ball_tracker', anonymous = True)
        self.pub = rospy.Publisher('/ball_pose/2D', PoseStamped, queue_size = 10)


    def translate(self, value, leftMin, leftMax, rightMin, rightMax):

        leftSpan = leftMax - leftMin
        rightSpan = rightMax - rightMin
        valueScaled = float(value - leftMin) / float(leftSpan)

        return rightMin + (valueScaled * rightSpan)


    def get_location(self):

        position = PoseStamped()

        yellow = [0, 255, 255] # define the color to be tracked in BGR colorspace

        c = np.uint8([[yellow]])
        hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
        lowerLimit = hsvC[0][0][0] - 10, 100, 100
        upperLimit = hsvC[0][0][0] + 10, 255, 255
        lowerLimit = np.array(lowerLimit, dtype = np.uint8)
        upperLimit = np.array(upperLimit, dtype = np.uint8)

        cap = cv2.VideoCapture(0)

        while True:

            ret, frame = cap.read()

            hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
            mask_ = Image.fromarray(mask)
            bbox = mask_.getbbox()

            if bbox is not None:

                x1, y1, x2, y2 = bbox
                frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)

                position.header.stamp = rospy.Time.now()
                position.header.frame_id = "world"

                position.pose.position.x = self.translate(x1, 0, 640, -15, 15)
                position.pose.position.y = self.translate(y1, 0, 480, -15, 15)

                self.pub.publish(position)

            cv2.imshow('mask', mask)
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord(' '):
                
                break
            
            cap.release()
            cv2.destroyAllWindows()

if __name__ == "__main__":

    obj = ball()
    obj.get_location()