import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
import cv_bridge
import time
import numpy
import cv2
import numpy as np
import threading
import datetime as dt
from enum import Enum

class StopLineTracker(Node):
    def __init__(self):
        self._delta = None
        self.start_time = time.time()  # 시작 시간 저장
        self.timer = None  # 타이머 변수 추가
        self.stop_line_mask = None  # 인스턴스 변수로 선언
               
    def stop_line_callback(self, img: np.ndarray) -> None:
        if time.time() - self.start_time < 10:  # 타임 시간 동안은 기능을 사용하지 않음
            return                    
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        lower_white = np.array([0, 0, 200]) 
        upper_white = np.array([180, 30, 255])
        self.stop_line_mask = cv2.inRange(hsv, lower_white, upper_white)
        
        h, w, d = img.shape
        search_top = int( h / 2 - 50 )
        search_bot = int(h)
        self.stop_line_mask[0:h,0:int(w / 2 -45)] = 0
        self.stop_line_mask[0:h,int(w / 2 - 35 ):int(w)] = 0
        self.stop_line_mask[0:int(h / 2 - 5),0:w] = 0
        self.stop_line_mask[int(h / 2 + 5):int(h),0:w] = 0 
        
        #self.stop_line_mask[0:search_top, 0:w] = 0
       # self.stop_line_mask[search_bot:h, 0:w] = 0
        
        M = cv2.moments(self.stop_line_mask)
        if M['m00'] > 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            cv2.circle(img, (cx, cy), 20, (0, 0, 255), -1)
            err = cy - h / 2
            self._delta = err
        cv2.imshow("window1", img)
        cv2.imshow("stop_line_mask", self.stop_line_mask)
        cv2.waitKey(3)
        
def main():
    tracker = LineTracker()
    import time
    for i in range(100):
        img = cv2.imread('/home/ros2/Ros2Projects/oom_ws/src/py_follower/worlds/sample.png')
        tracker.process(img)
        tracker.detect_stop_line(img)
        time.sleep(0.1)


def main():
    rclpy.init()
    tracker = StopLineTracker()
    import time
    for i in range(100):
        img = cv2.imread('/home/ros2/Ros2Projects/oom_ws/src/py_follower/worlds/sample.png')

        tracker.stop_line_callback(img)
        time.sleep(0.1)



if __name__ == "__main__":
    main()
