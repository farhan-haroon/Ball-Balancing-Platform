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

    # Crop the center portion of the frame to achieve a 1:1 aspect ratio
    height, width, _ = frame.shape
    min_dim = min(height, width)
    start_x = (width - min_dim) // 2
    start_y = (height - min_dim) // 2
    end_x = start_x + min_dim
    end_y = start_y + min_dim
    frame = frame[start_y:end_y, start_x:end_x]

    # Resize the cropped frame to 640x640
    frame = cv2.resize(frame, (640, 640))

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)
    mask_ = Image.fromarray(mask)
    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox
        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        
        x = translate(((x1 + x2) / 2), 0, min_dim, -20, 20)
        y = translate(((y1 + y2) / 2), 0, min_dim, -20, 20)

        print (x, y)

    cv2.imshow('mask', mask)
    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord(' '):
        break

cap.release()
cv2.destroyAllWindows()
