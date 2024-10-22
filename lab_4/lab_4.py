import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
np.set_printoptions(precision=3, suppress=True)


def rotation_x(angle):
   ################################################################################################
   # TODO: [already done] paste lab 2 forward kinematics here
   ################################################################################################
   return np.array([
              [1, 0, 0, 0],
              [0, np.cos(angle), -np.sin(angle), 0],
              [0, np.sin(angle), np.cos(angle), 0],
              [0, 0, 0, 1]
          ])


def rotation_y(angle):
   ################################################################################################
   # TODO: [already done] paste lab 2 forward kinematics here
   ################################################################################################
   return np.array([
              [np.cos(angle), 0, np.sin(angle), 0],
              [0, 1, 0, 0],
              [-np.sin(angle), 0, np.cos(angle), 0],
              [0, 0, 0, 1]
          ])


def rotation_z(angle):
   ################################################################################################
   # TODO: [already done] paste lab 2 forward kinematics here
   ################################################################################################
   return np.array([
              [np.cos(angle), -np.sin(angle), 0, 0],
              [np.sin(angle), np.cos(angle), 0, 0],
              [0, 0, 1, 0],
              [0, 0, 0, 1]
          ])


def translation(x, y, z):
   ################################################################################################
   # TODO: [already done] paste lab 2 forward kinematics here
   ################################################################################################
   return np.array([
              [1, 0, 0, x],
              [0, 1, 0, y],
              [0, 0, 1, z],
              [0, 0, 0, 1]
          ])


class InverseKinematics(Node):


   def __init__(self):
       super().__init__('inverse_kinematics')
       self.joint_subscription = self.create_subscription(
           JointState,
           'joint_states',
           self.listener_callback,
           10)
       self.joint_subscription  # prevent unused variable warning


       self.command_publisher = self.create_publisher(
           Float64MultiArray,
           '/forward_command_controller/commands',
           10
       )


       self.joint_positions = None
       self.joint_velocities = None
       self.target_joint_positions = None
       self.counter = 0


       ################################################################################################
       # TODO: Implement the trotting gait
       # position_1 = ...
       # position_2 = ...
       # ...
       ################################################################################################
       triangle_positions = np.array([
           [-0.05, 0.0, -0.12], # Liftoff
           [0.0, 0.0, -0.06],    # Mid-swing
           [0.05, 0.0, -0.12],  # Touchdown
       ])
       '''
       triangle_positions = np.array([
           [-0.06, 0.0, -0.12], # Liftoff
           [0.0, 0.0, -0.07],    # Mid-swing
           [0.06, 0.0, -0.12],  # Touchdown
       ])
       '''


       center_to_rf_hip = np.array([0.07500, -0.08350, 0])
       center_to_lf_hip = np.array([0.07500, 0.08350, 0])
       center_to_rb_hip = np.array([0.07500, -0.07250, 0])
       center_to_lb_hip = np.array([0.07500, 0.07250, 0])


       ## trotting
       rf_ee_triangle_positions = np.array([
           ################################################################################################
           triangle_positions
           ################################################################################################
       ]) + np.array([0.06, -0.09, 0])
       lf_ee_triangle_positions = np.array([
           ################################################################################################
           triangle_positions
           ################################################################################################
       ]) + np.array([0.06, 0.09, 0])
       rb_ee_triangle_positions = np.array([
           ################################################################################################
           triangle_positions * 0.8
           ################################################################################################
       ]) + np.array([-0.11, -0.09, 0])
       lb_ee_triangle_positions = np.array([
           ################################################################################################
           triangle_positions * 0.8
           ################################################################################################
       ]) + np.array([-0.11, 0.09, 0])


       self.ee_triangle_positions = [rf_ee_triangle_positions, lf_ee_triangle_positions, rb_ee_triangle_positions, lb_ee_triangle_positions]
       self.fk_functions = [self.fr_leg_fk, self.fl_leg_fk, self.br_leg_fk, self.lb_leg_fk]


       self.target_joint_positions_cache, self.target_ee_cache = self.cache_target_joint_positions()
       print(f'shape of target_joint_positions_cache: {self.target_joint_positions_cache.shape}')
       print(f'shape of target_ee_cache: {self.target_ee_cache.shape}')




       self.pd_timer_period = 1.0 / 200  # 200 Hz
       self.ik_timer_period = 1.0 / 100   # 10 Hz 
       self.pd_timer = self.create_timer(self.pd_timer_period, self.pd_timer_callback)
       self.ik_timer = self.create_timer(self.ik_timer_period, self.ik_timer_callback)




   def fr_leg_fk(self, theta):
       ################################################################################################
       # TODO: [already done] paste lab 2 forward kinematics here
       ################################################################################################
       T_0_1 = translation(0.07500, -0.0445, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_1_2 = translation(0, 0, 0.039) @ rotation_y(np.pi / 2) @ rotation_z(theta[1])
       T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(theta[2])
       T_3_ee = translation(0.06231, -0.06216, 0.018)
       T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
       joint_pos = np.array([0, 0, 0, 1])
       #joint_pos[:3] = self.joint_positions
       end_effector_position = T_0_ee @ joint_pos.T
       end_effector_position = (end_effector_position[:3] / end_effector_position[-1]).T
       return end_effector_position


   def fl_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_0_1 = translation(0.07500, 0.0445, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
       T_1_2 = translation(0, 0, -0.039) @ rotation_y(np.pi / 2) @ rotation_z(theta[1])
       T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(-theta[2])
       T_3_ee = translation(0.06231, -0.06216, -0.018)
       T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
       joint_pos = np.array([0, 0, 0, 1])
       #joint_pos[:3] = self.joint_positions
       end_effector_position = T_0_ee @ joint_pos.T
       end_effector_position = (end_effector_position[:3] / end_effector_position[-1]).T
       return end_effector_position


   def br_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_0_1 = translation(-0.07500, -0.0335, 0) @ rotation_x(1.57080) @ rotation_z(theta[0])
       T_1_2 = translation(0, 0, 0.039) @ rotation_y(-np.pi / 2) @ rotation_z(theta[1])
       T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(theta[2])
       T_3_ee = translation(0.06231, -0.06216, 0.018)
       T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
       joint_pos = np.array([0, 0, 0, 1])
       #joint_pos[:3] = self.joint_positions
       end_effector_position = T_0_ee @ joint_pos.T
       end_effector_position = (end_effector_position[:3] / end_effector_position[-1]).T
       return end_effector_position


   def lb_leg_fk(self, theta):
       ################################################################################################
       # TODO: implement forward kinematics here
       ################################################################################################
       T_0_1 = translation(-0.07500, 0.0335, 0) @ rotation_x(1.57080) @ rotation_z(-theta[0])
       T_1_2 = translation(0, 0, -0.039) @ rotation_y(-np.pi / 2) @ rotation_z(theta[1])
       T_2_3 = translation(0, -0.0494, 0.0685) @ rotation_y(np.pi / 2) @ rotation_z(-theta[2])
       T_3_ee = translation(0.06231, -0.06216, -0.018)
       T_0_ee = T_0_1 @ T_1_2 @ T_2_3 @ T_3_ee
       joint_pos = np.array([0, 0, 0, 1])
       #joint_pos[:3] = self.joint_positions
       end_effector_position = T_0_ee @ joint_pos.T
       end_effector_position = (end_effector_position[:3] / end_effector_position[-1]).T
       return end_effector_position


   def forward_kinematics(self, theta):
       return np.concatenate([self.fk_functions[i](theta[3*i: 3*i+3]) for i in range(4)])


   def listener_callback(self, msg):
       joints_of_interest = [
           'leg_front_r_1', 'leg_front_r_2', 'leg_front_r_3',
           'leg_front_l_1', 'leg_front_l_2', 'leg_front_l_3',
           'leg_back_r_1', 'leg_back_r_2', 'leg_back_r_3',
           'leg_back_l_1', 'leg_back_l_2', 'leg_back_l_3'
       ]
       self.joint_positions = np.array([msg.position[msg.name.index(joint)] for joint in joints_of_interest])
       self.joint_velocities = np.array([msg.velocity[msg.name.index(joint)] for joint in joints_of_interest])


   def inverse_kinematics_single_leg(self, target_ee, leg_index, initial_guess=[0., 0., 0.]):
       leg_forward_kinematics = self.fk_functions[leg_index]


       def cost_function(theta):
           gt_ee = leg_forward_kinematics(theta)
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################
           return np.sum((gt_ee - target_ee)**2), np.linalg.norm(gt_ee - target_ee)


       def gradient(theta, epsilon=1e-3):
           grad = np.zeros(3)
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################
           gradient = np.zeros([3,])
           for i, t in enumerate(theta):
               theta_min = [theta[0] - (i == 0)* epsilon, theta[1] - (i == 1)* epsilon, theta[2] - (i == 2) * epsilon]
               theta_max = [theta[0] + (i == 0)* epsilon, theta[1] + (i == 1)* epsilon, theta[2] + (i == 2) * epsilon]
               gradient[i] = (cost_function(theta_max)[0] - cost_function(theta_min)[0]) / (2 * epsilon)
           return gradient


       theta = np.array(initial_guess)
       learning_rate = 5. # TODO:[already done] paste lab 3 inverse kinematics here
       max_iterations = 50 # TODO: [already done] paste lab 3 inverse kinematics here
       tolerance = 1e-2 # TODO: [already done] paste lab 3 inverse kinematics here


       cost_l = []
       for _ in range(max_iterations):
           ################################################################################################
           # TODO: [already done] paste lab 3 inverse kinematics here
           ################################################################################################
           theta = theta - learning_rate * gradient(theta)
           cost, l1 = cost_function(theta)
           cost_l.append(cost)
           if l1.mean() < tolerance:
               break


       return theta


   def interpolate_triangle(self, t, leg_index):
       ################################################################################################
       # TODO: implement interpolation for all 4 legs here
       ################################################################################################
       vertex1, vertex2, vertex3 = self.ee_triangle_positions[leg_index][0]
       t_res = t % 4 if leg_index in [0, 3] else (t+2) % 4
       if 0 <= t_res <= 1:
           return vertex1 + t_res * (vertex2 - vertex1)
       elif 1 < t_res <= 2:
           return vertex2 + (t_res-1) * (vertex3 - vertex2)
       else:
           return vertex3 + (t_res-2) * (vertex1 - vertex3)/2


   def cache_target_joint_positions(self):
       # Calculate and store the target joint positions for a cycle and all 4 legs
       target_joint_positions_cache = []
       target_ee_cache = []
       for leg_index in range(4):
           target_joint_positions_cache.append([])
           target_ee_cache.append([])
           target_joint_positions = [0] * 3
           for t in np.arange(0, 4, 0.08):
               print(t)
               target_ee = self.interpolate_triangle(t, leg_index)
               target_joint_positions = self.inverse_kinematics_single_leg(target_ee, leg_index, initial_guess=target_joint_positions)


               target_joint_positions_cache[leg_index].append(target_joint_positions)
               target_ee_cache[leg_index].append(target_ee)


       # (4, 50, 3) -> (50, 12)
       target_joint_positions_cache = np.concatenate(target_joint_positions_cache, axis=1)
       target_ee_cache = np.concatenate(target_ee_cache, axis=1)
      
       return target_joint_positions_cache, target_ee_cache


   def get_target_joint_positions(self):
       target_joint_positions = self.target_joint_positions_cache[self.counter]
       target_ee = self.target_ee_cache[self.counter]
       self.counter += 1
       if self.counter >= self.target_joint_positions_cache.shape[0]:
           self.counter = 0
       return target_ee, target_joint_positions


   def ik_timer_callback(self):
       if self.joint_positions is not None:
           target_ee, self.target_joint_positions = self.get_target_joint_positions()
           current_ee = self.forward_kinematics(self.joint_positions)


           self.get_logger().info(
               f'Target EE: {target_ee}, \
               Current EE: {current_ee}, \
               Target Angles: {self.target_joint_positions}, \
               Target Angles to EE: {self.forward_kinematics(self.target_joint_positions)}, \
               Current Angles: {self.joint_positions}')


   def pd_timer_callback(self):
       if self.target_joint_positions is not None:
           command_msg = Float64MultiArray()
           command_msg.data = self.target_joint_positions.tolist()
           self.command_publisher.publish(command_msg)


def main():
   rclpy.init()
   inverse_kinematics = InverseKinematics()
  
   try:
       rclpy.spin(inverse_kinematics)
   except KeyboardInterrupt:
       print("Program terminated by user")
   finally:
       # Send zero torques
       zero_torques = Float64MultiArray()
       zero_torques.data = [0.0] * 12
       inverse_kinematics.command_publisher.publish(zero_torques)
      
       inverse_kinematics.destroy_node()
       rclpy.shutdown()


if __name__ == '__main__':
   main()



