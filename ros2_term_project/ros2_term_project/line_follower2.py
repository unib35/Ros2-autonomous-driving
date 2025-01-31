import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
import cv_bridge
import time
import cv2
import numpy as np
import threading
import datetime as dt
from enum import Enum
from .stop_line_tracker import StopLineTracker
from .obstacle import Obstacle

class LineFollower2(Node):
    def __init__(self, line_tracker: LineTracker, stop_line_tracker:StopLineTracker, obstacle:Obstacle):
        super().__init__('line_follower2')
        self.line_tracker = line_tracker
        self.stop_line_tracker = stop_line_tracker
        self.obstacle = obstacle
        self.bridge = cv_bridge.CvBridge()
        self._subscription = self.create_subscription(Image, '/camera4/image_raw', self.image_callback, 10)
        self._subscription2 = self.create_subscription(Image, '/camera3/image_raw',self.stop_line_callback, 10)
        self._subscription3 = self.create_subscription(LaserScan, '/scan1', self.scan_callback, 10)          
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.twist = Twist()
        self.twist.linear.x = 3.3
        self.img = None
        #self.stopped = False
        self.count = 0
        self.start_time = time.time()  # 시작 시간 저장
        self.timer = None  # 타이머 변수 추가
        self.set_timer(32, self.first_decrease_speed)
        self.sensorFlag = False
                
    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if not self.obstacle.obstacle_found and min_distance < 7.0:
            self.stop()
            self.obstacle.obstacle_found = True
        if self.obstacle.obstacle_found and min_distance > 7.0 :
            self.go()
            self.obstacle.obstacle_found = False    
    def go(self):
        self.get_logger().info('obstacle has been removed...')
        self.twist.linear.x = 3.4
        
    def moving_callback(self, msg: LaserScan):
        self.twist.linear.x = 3.0
        self._publisher.publish(self.twist)   
                                                                
    def stop_line_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.stop_line_tracker.stop_line_callback(img)                                   
        if self.stop_line_tracker.stop_line_mask is not None and np.any(self.stop_line_tracker.stop_line_mask == 255):

            if self.sensorFlag == False:
                self.count += 1
            
            
            self.sensorFlag = True
            
            
            if self.count == 1:
                self.stop()
                time.sleep(3)
                self.twist.linear.x = 2.5
                self.get_logger().info('change linear.x = %f' % self.twist.linear.x) 
                self.get_logger().info('count = %f' % self.count)                           
                self._publisher.publish(self.twist)
                self.set_timer(25, self.increase_speed)  # 20초 후에 increase_speed 호출
                
                               
            if self.count == 2:
                self.stop()  
                time.sleep(3)  
                self.twist.linear.x = 3.2
                self.get_logger().info('change linear.x = %f' % self.twist.linear.x) 
                self.get_logger().info('count = %f' % self.count)                           
                self._publisher.publish(self.twist) 
                self.set_timer(16, self.second_decrease_speed)  
                   
                                                        
            if self.count == 3:
                self.get_logger().info('count = %f' % self.count)            
                self.stop()
                time.sleep(100)

        else:
            self.sensorFlag = False

  
    def image_callback(self, msg: Image):
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process_second(img)
        self.twist.angular.z = (1) * self.line_tracker._delta / 250
        self._publisher.publish(self.twist)

        
    def increase_speed(self):
        self.twist.linear.x = 3.4
        self.get_logger().info('change linear.x = %f' % self.twist.linear.x) 
        self._publisher.publish(self.twist)
        
    def first_decrease_speed(self):
        self.twist.linear.x = 2.2
        self.get_logger().info('change linear.x = %f' % self.twist.linear.x) 
        self._publisher.publish(self.twist)
                  
    def second_decrease_speed(self):
        self.twist.linear.x = 2.1
        self.get_logger().info('change linear.x = %f' % self.twist.linear.x) 
        self._publisher.publish(self.twist)
        
  
                          
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)

    def set_timer(self, duration, callback):
        if self.timer is not None:
            self.timer.cancel()
        self.timer = threading.Timer(duration, callback)
        self.timer.start()

        
    @property
    def publisher(self):
        return self._publisher


def main():
    rclpy.init()
    tracker = LineTracker()
    stop = StopLineTracker()
    obstacle = Obstacle()
    follower2 = LineFollower2(tracker,stop,obstacle)
    try:
        rclpy.spin(follower2)
    except KeyboardInterrupt:
        follower2.stop()
        follower2.stop()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
