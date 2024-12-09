from enum import Enum
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision_msgs.msg import Detection2DArray
import numpy as np
import time

IMAGE_WIDTH = 1400

# TODO: Add your new constants here

TIMEOUT = None #TODO threshold in timer_callback
SEARCH_YAW_VEL = None #TODO searching constant
TRACK_FORWARD_VEL = None #TODO tracking constant
KP = -2. #TODO proportional gain for tracking

class State(Enum):
    SEARCH = 0
    TRACK = 1

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine_node')

        self.detection_subscription = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

        self.command_publisher = self.create_publisher(
            Twist,
            'cmd_vel',
            10
        )

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.state = State.TRACK

        # TODO: Add your new member variables here
        self.kp = KP # TODO
        self.last_position = 0.
        self.lost_target_timer = None

    def detection_callback(self, msg):
        """
        Determine which of the HAILO detections is the most central detected object
        """
        x_min = None
        detections = msg.detections
        for i, detection in enumerate(detections):
            x = detection.bbox.center.position.x
            x = (x - 662) / 319
            if x_min is None or abs(x - self.last_position) < abs(x_min - self.last_position):
                x_min = x
        if x_min is None:
            if self.lost_target_timer is None:
                self.lost_target_timer = time.time()
            return
        self.last_position = x_min
        self.lost_target_timer = None
        print(f"Bounding box's x coordinate: {x_min}")

    def timer_callback(self):
        """
        Implement a timer callback that sets the moves through the state machine based on if the time since the last detection is above a threshold TIMEOUT
        """
        
        if self.lost_target_timer  is not None and time.time() - self.lost_target_timer > 2: # TODO: Part 3.2
            self.state = State.SEARCH
        else:
            self.state = State.TRACK

        yaw_command = 0.0
        forward_vel_command = 0.0

        if self.state == State.SEARCH:
            yaw_command = 0.8 # TODO: Part 3.1
            forward_vel_command = 0.
        elif self.state == State.TRACK:
            yaw_command = self.kp * self.last_position
            forward_vel_command = 0.6 # TODO: Part 2 / 3.4

        cmd = Twist()
        cmd.angular.z = yaw_command
        cmd.linear.x = forward_vel_command
        self.command_publisher.publish(cmd)

def main():
    rclpy.init()
    state_machine_node = StateMachineNode()

    try:
        rclpy.spin(state_machine_node)
    except KeyboardInterrupt:
        print("Program terminated by user")
    finally:
        zero_cmd = Twist()
        state_machine_node.command_publisher.publish(zero_cmd)

        state_machine_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
