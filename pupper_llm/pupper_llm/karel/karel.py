# karel.py
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import simpleaudio as sa
import pygame

class KarelPupper:
    def start():
        rclpy.init()
        '''
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)
        '''

    def __init__(self):
        # rclpy.init()
        self.node = Node('karel_node')
        self.publisher = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def move(self):
        move_cmd = Twist()
        move_cmd.linear.x = 1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move forward...')
        time.sleep(1)
        self.stop()
    
    def move_back(self):
        move_cmd = Twist()
        move_cmd.linear.x = -1.0
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('Move backward...')
        time.sleep(1)
        self.stop()

    def crab_walk_left(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.7
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('crab leftward...')
        time.sleep(1)
        self.stop()
    
    def crab_walk_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = -0.7
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('crab rightward...')
        time.sleep(1)
        self.stop()
    
    def run(self):
        move_cmd = Twist()
        move_cmd.linear.x = 2.
        move_cmd.angular.z = 0.0 
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.get_logger().info('run forward...')
        time.sleep(1)
        self.stop()

    def turn_left(self):
        pass
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 1.
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=0.8)
        self.node.get_logger().info('Turn left...')
        time.sleep(0.5)
        self.stop()

    def turn_right(self):
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = -1.
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=0.8)
        self.node.get_logger().info('Turn right...')
        time.sleep(0.5)
        self.stop()


    def bark(self):
        self.node.get_logger().info('Bark...')
        pygame.mixer.init()
        bark_sound = pygame.mixer.Sound('/home/pi/pupper_llm/sounds/dog_bark.wav')
        bark_sound.play()
        
        time.sleep(0.5)
        self.stop()
    
    def dance(self):
        def tf():
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.75
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.5)
            time.sleep(0.1)
            self.stop()
        def tr():
            move_cmd = Twist()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.75
            self.publisher.publish(move_cmd)
            rclpy.spin_once(self.node, timeout_sec=0.5)
            time.sleep(0.1)
            self.stop()
        self.node.get_logger().info('Rick Rolling...')
        pygame.mixer.init()
        dance_sound = pygame.mixer.Sound('/home/pi/pupper_llm/sounds/rickroll.wav')
        dance_sound.play()
        tf()
        tr()
        tr()
        tf()
        self.move()
        self.move_back()
        self.crab_walk_left()
        self.crab_walk_right()
        self.stop()


    def stop(self):
        self.node.get_logger().info('Stopping...')
        move_cmd = Twist()
        move_cmd.linear.x = 0.0
        move_cmd.linear.y = 0.0
        move_cmd.linear.z = 0.0
        move_cmd.angular.x = 0.0
        move_cmd.angular.y = 0.0
        move_cmd.angular.z = 0.0
        self.publisher.publish(move_cmd)
        rclpy.spin_once(self.node, timeout_sec=1.0)
    
    def __del__(self):
        self.node.get_logger().info('Tearing down...')
        self.node.destroy_node()
        rclpy.shutdown()

