import cv2
import numpy as np

class LineTracker:
    def __init__(self):
        self._delta = 0.0
        self._delta2 = 0.0

    def process(self, img: np.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200]) 
        upper_white = np.array([180, 30, 255])
	
        mask = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape

        mask[0:h, 0:int(2 * w / 4)] = 0
        mask[0:h, int(2 * w / 4 + 25):w] = 0
        mask[int(h / 2 + 50):h,int(w / 2):int(w / 2+ 25)] = 0
        
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            err = cy - h / 2
            self._delta = err
        cv2.imshow("window", img)
        cv2.imshow("mask", mask)
        cv2.waitKey(3)
        
    def process_second(self, img: np.ndarray) -> None:
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200]) 
        upper_white = np.array([180, 30, 255])
	
        mask3 = cv2.inRange(hsv, lower_white, upper_white)

        h, w, d = img.shape
        mask3[0:int(h / 2 - 45),0:int(w)] = 0    
        mask3[0:h,int(w / 2 + 20):int(w)] = 0
        mask3[int(8 * h /10 ):h,0:w] = 0 

        M = cv2.moments(mask3)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            err = cy - h / 2
            self._delta = err
        cv2.imshow("window3", img)
        cv2.imshow("mask3", mask3)
        cv2.waitKey(3)
        
 
    @property
    def delta(self):
        return self._delta

def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('/home/ros2/Ros2Projects/oom_ws/src/py_follower/worlds/sample.png')
        tracker.process(img)
        tracker.detect_stop_line(img)
        time.sleep(0.1)

if __name__ == "__main__":
    main()
