import cv2
from PIL import Image
from util import get_limits
import numpy as np

yellow = [0, 255, 255] # yellow in BGR colorspace

cap = cv2.VideoCapture(0)

# lowerLimit, upperLimit = get_limits(color = yellow)

# print(upperLimit.shape)
# print(lowerLimit.shape)

lowerLimit = np.ndarray(shape=(3,))
upperLimit = np.ndarray(shape=(3,))

# lowerLimit = [25, 100, 100]
# upperLimit = [35, 255, 255]

print(type(lowerLimit))
print(type(upperLimit))

lowerLimit[0], lowerLimit[1], lowerLimit[2] = 25, 100, 200
upperLimit[0], upperLimit[1], upperLimit[2] = 35, 255, 255

print(lowerLimit[0])

print("Lower Limit =", lowerLimit)
print("Upper Limit =", upperLimit)

while True:

    ret, frame = cap.read()

    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    mask = cv2.inRange(hsvImage, lowerLimit, upperLimit)

    mask_ = Image.fromarray(mask)

    bbox = mask_.getbbox()

    if bbox is not None:
        x1, y1, x2, y2 = bbox

        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 5)
        print(x1, y1, x2, y2)

    cv2.imshow('frame', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()