import cv2
import numpy as np

def detect_yellow_ball(frame):
    # Convert frame from BGR to HSV color space
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # Define range of yellow color in HSV
    lower_yellow = np.array([20, 100, 100])
    upper_yellow = np.array([30, 255, 255])
    
    # Threshold the HSV image to get only yellow colors
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    # Find contours in the mask
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    if contours:
        # Get the largest contour (assuming it's the ball)
        largest_contour = max(contours, key=cv2.contourArea)
        
        # Ensure that the contour has non-zero area
        if cv2.contourArea(largest_contour) > 0:
            # Get the centroid of the largest contour
            M = cv2.moments(largest_contour)
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            
            # Display the centroid on the frame
            cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
            
            # Convert centroid coordinates to the specified coordinate frame
            # Assuming the frame size is 640x480 pixels
            x = cx - 320  # Adjusting for center origin
            y = 240 - cy  # Adjusting for first quadrant at top right
            
            # Print the coordinates
            print("Ball Coordinates (x, y):", x, y)
    
    # Display the frame
    cv2.imshow("Frame", frame)
    
    # Press 'q' to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        return False
    
    return True

# Main function
def main():
    # Initialize webcam
    cap = cv2.VideoCapture(0)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Detect yellow ball and get its coordinates
        if not detect_yellow_ball(frame):
            break
    
    # Release webcam and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
