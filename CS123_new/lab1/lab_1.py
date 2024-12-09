import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import time
from collections import deque
from numpy import mean
import matplotlib.pyplot as plt
import seaborn as sns
import pandas as pd
import math
import random


JOINT_NAME = 'leg_front_r_1'
####
####
KP = 1.2 # YOUR KP VALUE
KD = 0.2 # YOUR KD VALUE
####
####
LOOP_RATE = 200  # Hz
MAX_TORQUE = 3.0
DELAY = 1 # min is 1 for no delay


random.seed(1) # for pos - delay vs. no delay comparison


class JointStateSubscriber(Node):


   def __init__(self):
       super().__init__('joint_state_subscriber')
       # Create a subscriber to the /joint_states topic
       self.subscription = self.create_subscription(
           JointState,
           '/joint_states',
           self.get_joint_info,
           10  # QoS profile history depth
       )
       self.subscription  # prevent unused variable warning


       # Publisher to the /forward_command_controller/commands topic
       self.command_publisher = self.create_publisher(
           Float64MultiArray,
           '/forward_command_controller/commands',
           10
       )
       self.print_counter = 0
       self.pd_torque = 0
       self.joint_pos = 0
       self.joint_vel = 0
       self.target_joint_pos = 0
       self.target_joint_vel = 0
       self.timer = -1 # offset by -1 to begin at 0 once code runs
       self.cur_loss = []
       self.loss_map = {}
       self.control_arg = "PD" # {"BB", "P", "PD"}
       self.torque_history = deque(maxlen=DELAY)


       # Create a timer to run pd_loop at the specified frequency
       self.create_timer(1.0 / LOOP_RATE, self.pd_loop)


   def get_target_joint_info(self):
       # target_joint_pos, target_joint_vel
       return self.target_joint_pos, self.target_joint_vel


   def calculate_pd_torque(self, joint_pos, joint_vel, target_joint_pos, target_joint_vel):
       self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info()
       if self.control_arg == "BB":
           return MAX_TORQUE * (1 if joint_pos < target_joint_pos else -1)
       elif self.control_arg == "P":
           return KP * (self.target_joint_pos - joint_pos)
       elif self.control_arg == "PD":
           torque =  KP * (self.target_joint_pos - joint_pos) + KD * (self.target_joint_vel - joint_vel)
           self.torque_history.append(torque)
           return self.torque_history[0]
       else:
           raise ValueError("You're bad")




   def print_info(self):
       if self.print_counter == 0:
           print(f"timer: {self.timer}, KP: {KP}, KD: {KD}, Pos: {self.joint_pos:.2f}, Target Pos: {self.target_joint_pos:.2f}, Vel: {self.joint_vel:.2f}, Target Vel: {self.target_joint_vel:.2f}, Tor: {self.pd_torque:.2f}")
       self.print_counter += 1
       self.print_counter %= 50


   def get_joint_info(self, msg):
       joint_index = msg.name.index(JOINT_NAME)
       joint_pos = msg.position[joint_index]
       joint_vel = msg.velocity[joint_index]


       self.joint_pos = joint_pos
       self.joint_vel = joint_vel
  
       return joint_pos, joint_vel


   def get_PD_min_throttle(self):
       global KP, KD
       self.timer += 1
       if KP > 5:
           print(self.loss_map)
           rows = []
           for k, v in self.loss_map.items():
               kp, kd = map(lambda x: round(float(x), 2), k.split('_'))
               rows.append([kp, kd, round(v, 2)])
           loss_df = pd.DataFrame(rows, columns=['KP', 'KD', 'Loss'])
           hm_loss = loss_df.pivot(index='KP', columns='KD', values='Loss')
           print(hm_loss)
           plt.figure(figsize=(8, 6))
           plt.title("Heatmap: KP and KD on RMSE loss")
           sns.heatmap(hm_loss, annot=True, cmap="viridis")
           plt.show()
           raise KeyboardInterrupt()
       if KD > 1:
           KP += 0.5
           KD = 0
       if self.timer % 250 == 0:
           KD += 0.1
           self.target_joint_pos = random.uniform(-math.pi, math.pi)
           self.loss_map[f"{KP}_{KD}"] = mean(self.cur_loss)
           self.cur_loss = []
       else:
           if self.timer % 250 > 100:
               self.cur_loss.append((self.joint_pos - self.target_joint_pos) ** 2)




   def pd_loop(self):
       # uncomment for tuning/getting heatmap of KP and KD vs RMSE loss
       # self.get_PD_min_throttle()
       current_time = time.time()
       # Step 8 - sin motion
       # self.target_joint_pos = math.pi * 1.05 * math.sin(current_time)
       self.pd_torque = self.calculate_pd_torque(self.joint_pos, self.joint_vel, self.target_joint_pos, self.target_joint_vel)
       self.print_info()
       self.publish_torque(self.pd_torque)


   def publish_torque(self, torque=0.0):
       # Create a Float64MultiArray message with zero kp and kd values
       command_msg = Float64MultiArray()
       torque = np.clip(torque, -MAX_TORQUE, MAX_TORQUE)
       command_msg.data = [torque, 0.0, 0.0]  # Zero kp and kd values


       # Publish the message
       self.command_publisher.publish(command_msg)


def main(args=None):
   rclpy.init(args=args)


   # Create the node
   joint_state_subscriber = JointStateSubscriber()


   # Keep the node running until interrupted
   try:
       rclpy.spin(joint_state_subscriber)
   except KeyboardInterrupt:
       print("Ctrl-C detected")
       joint_state_subscriber.publish_torque(0.0)
#    finally:
#        joint_state_subscriber.publish_torque(0.0)


   # Clean up and shutdown
   joint_state_subscriber.destroy_node()
   rclpy.shutdown()


if __name__ == '__main__':
   main()
else:
   main()


