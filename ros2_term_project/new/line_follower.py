import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from .line_tracker import LineTracker
import cv_bridge
import datetime as dt
from enum import Enum


class LineFollower(Node):
    def __init__(self, line_tracker: LineTracker):
        super().__init__('line_follower')
        self.line_tracker = line_tracker
        self.bridge = cv_bridge.CvBridge()
        self._subscription = self.create_subscription(Image, '/camera1/image_raw', self.image_callback, 10)

        #add a subscriber for /scan
        self.lider_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self._publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.twist = Twist()
        self.twist.linear.x = 1.0
        self.img = None
        self.obstacle_found = False
        self.waiting_start_time = None
        self.avoidance_move = False
        self.avoidance_start_time = None
        self.avoidance_state = LineFollower.State.WAITING
        self.avoidance_sign = 1
        self.avoidance_start_delta = 0

    def image_callback(self, msg: Image):
        if self.obstacle_found: return
        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.line_tracker.process(img)
        #
        self.twist.linear.x = 0.7  # 1.0 -> 0.7
        self.twist.angular.z = (-1) * self.line_tracker.delta / 450  # OK: 400
        self._publisher.publish(self.twist)

    def scan_callback(self, msg: LaserScan):
        min_distance = min(msg.ranges)
        if not self.obstacle_found and min_distance < 4.0:
            self.stop()
            self.obstacle_found = True
            if self.waiting_start_time is None:
                # set the start time
                self.waiting_start_time = dt.datetime.now()
            self.get_logger().info('An obstacle found in %.2f m' % min(msg.ranges))

            if self.obstacle_found:
                if self.waiting_start_time is None: return
                # 4.0 + 1.0 deliberately
                if self.avoidance_state == LineFollower.State.WAITING and min_distance > 5.0:
                    self.obstacle_found = False
                    self.waiting_start_time = None
                    return
                elapsed_time = (dt.datetime.now() - self.waiting_start_time).total_seconds()
                self.get_logger().debug('elapsed time = %f' % elapsed_time)
                if self.avoidance_move is False and elapsed_time > 5.0:
                    # self.obstacle_found should be set False after the end of avoidance move
                    self.avoidance_move = True
                    self.avoidance_state = LineFollower.State.STEP_ASIDE
                    self.avoidance_start_time = dt.datetime.now()  # cf. self.get_clock().now()
                    self.avoidance_start_delta = self.line_tracker.delta
                    self.avoidance_sign = -1 if self.line_tracker.delta > 0.0 else 1
                    self.get_logger().info('delta = %.2f' % self.line_tracker.delta)

                    if self.avoidance_move:
                        if self.avoidance_state == LineFollower.State.STEP_ASIDE:
                            self.step_aside()
                            elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                            if elapsed_time > 3.5:
                                print('State changed...')
                                self.avoidance_state = LineFollower.State.GO_STRAIGHT
                                self.avoidance_start_time = dt.datetime.now()
                        elif self.avoidance_state == LineFollower.State.GO_STRAIGHT:
                            self.go_straight()
                            elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                            max_time = 2.5 if abs(self.avoidance_start_delta) > 25 else 4.0

                            if elapsed_time > max_time:
                                self.avoidance_state = LineFollower.State.STEP_IN
                                self.avoidance_start_time = dt.datetime.now()
                                # self.obstacle_found = False  # to activate image_callback()
                    elif self.avoidance_state == LineFollower.State.STEP_IN:
                        self.step_in()
                        elapsed_time = (dt.datetime.now() - self.avoidance_start_time).total_seconds()
                        if elapsed_time > 2.0:
                            self.avoidance_state = LineFollower.State.WAITING
                            self.avoidance_start_time = None
                            self.avoidance_move = False
                            self.obstacle_found = False
                            self.waiting_start_time = None

    def stop(self):
        self.get_logger().info('stop() called...')
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
        self._publisher.publish(self.twist)

    def step_aside(self):
        self.twist.linear.x = 1.0
        self.twist.angular.z = self.avoidance_sign * \
                               (0.475 if abs(self.avoidance_start_delta) > 25 else 0.3)
        self._publisher.publish(self.twist)

    def step_in(self):
        self.twist.linear.x = 0.6
        self.twist.angular.z = (-1) * self.avoidance_sign * \
                               (0.2 if abs(self.avoidance_start_delta) > 25 else 0.25)
        self._publisher.publish(self.twist)

    def go_straight(self):
        self.twist.linear.x = 0.8
        self.twist.angular.z = (-1) * self.avoidance_sign * 0.15 \
                if abs(self.avoidance_start_delta) > 25 else (-1) * self.avoidance_sign * 0.3
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
    tracker = LineTracker()
    follower = LineFollower(tracker)
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        follower.stop()
        follower.stop()
    rclpy.shutdown()


if __name__== '__main__':
    main()