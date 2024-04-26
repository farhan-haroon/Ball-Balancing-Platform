import cv2
from PIL import Image
import numpy as np

def get_limits(color):

    c = np.uint8([[color]])
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)
    lowerLimit = hsvC[0][0][0] - 10, 100, 100
    upperLimit = hsvC[0][0][0] + 10, 255, 255
    lowerLimit = np.array(lowerLimit, dtype = np.uint8)
    upperLimit = np.array(upperLimit, dtype = np.uint8)

    return lowerLimit, upperLimit

yellow = [0, 255, 255] # yellow in BGR colorspace
cap = cv2.VideoCapture("/dev/video2")
lowerLimit, upperLimit = get_limits(color = yellow)

def translate(value, leftMin, leftMax, rightMin, rightMax):

    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    valueScaled = float(value - leftMin) / float(leftSpan)

    return rightMin + (valueScaled * rightSpan)


while True:

    ret, frame = cap.read()
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
    mask_ = Image.fromarray(mask)
    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
        x = translate(((x1 + x2) / 2), 0, 640, -20, 20)
        y = translate(((y1 + y2) / 2), 0, 480, -20, 20)

        print (x, y)

    cv2.imshow('mask', mask)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()