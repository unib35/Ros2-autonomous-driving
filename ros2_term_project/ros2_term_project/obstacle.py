import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import cv_bridge
import time
import numpy
import cv2
import numpy as np
import threading
import datetime as dt
from enum import Enum


class Obstacle(Node):
    def __init__(self):
        self._delta = None
        self.sensorFlag = False
        self.obstacle_found = False
        self.waiting_start_time = None
        self.avoidance_move = False
        self.avoidance_start_time = None
        self.avoidance_sign = 1
        self.avoidance_start_delta = 0
        
    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if not self.obstacle_found and min_distance < 7.0:
            self.stop()
            self.obstacle_found = True
            self.get_logger().info('An obstacle found in %.2f m' % min(msg.ranges))
        if self.obstacle_found and min_distance > 7.0 :
            self.go()
            self.obstacle_found = False
    def go(self):
        self.get_logger().info('obstacle has been removed...')
        self.twist.linear.x = 3.0
        
    def moving_callback(self, msg: LaserScan):
        self.twist.linear.x = 3.0
        self._publisher.publish(self.twist)   
                                         
                   
    def stop(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)
        
    @property
    def publisher(self):
        return self._publisher

    class State(Enum):
        WAITING = 0
        STEP_ASIDE = 1
        GO_STRAIGHT = 2
        STEP_IN = 3
        
def main():
    rclpy.init()
    obstacle = Obstacle()



if __name__ == "__main__":
    main()
