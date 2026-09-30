import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import time
from collections import deque
import signal


JOINT_NAME = "leg_front_l_3"
####
####
KP = 2.0  # YOUR KP VALUE
KD = 0.3  # YOUR KD VALUE
FEEDFORWARD = 0.0
####
####
LOOP_RATE = 200  # Hz
MAX_TORQUE = 3.0


DELAY_SECONDS = 0
DELAY_STEPS = int(DELAY_SECONDS * LOOP_RATE)


PERIODIC_AMPLITUDE = 20  # radians
PERIODIC_FREQUENCY = 1.0  # Hz






class JointStateSubscriber(Node):


   def __init__(self):
       super().__init__("joint_state_subscriber")
       # Create a subscriber to the /joint_states topic
       self.subscription = self.create_subscription(
           JointState, "/joint_states", self.get_joint_info, 10  # QoS profile history depth
       )
       self.subscription  # prevent unused variable warning


       # Publisher to the /forward_command_controller/commands topic
       self.command_publisher = self.create_publisher(Float64MultiArray, "/forward_command_controller/commands", 10)
       self.print_counter = 0
       self.calculated_torque = 0
       self.joint_pos = 0
       self.joint_vel = 0
       self.target_joint_pos = 0
       self.target_joint_vel = 0
       # self.torque_history = deque(maxlen=DELAY)
      
       self.angle_buffer = deque(maxlen=DELAY_STEPS + 1)
       self.velocity_buffer = deque(maxlen=DELAY_STEPS + 1)
       self.start_time = time.time()


       # Create a timer to run control_loop at the specified frequency
       self.create_timer(1.0 / LOOP_RATE, self.control_loop)


   def get_target_joint_info(self):
       elapsed_time = time.time() - self.start_time
       angular_frequency = 2.0 * math.pi * PERIODIC_FREQUENCY
       target_joint_pos = PERIODIC_AMPLITUDE * math.sin(angular_frequency * elapsed_time)
       target_joint_vel = PERIODIC_AMPLITUDE * angular_frequency * math.cos(angular_frequency * elapsed_time)


       # target_joint_pos, target_joint_vel
       return target_joint_pos, target_joint_vel


   def calculate_torque(self, joint_pos, joint_vel, target_joint_pos, target_joint_vel):
       ####
       torque = KP * (target_joint_pos - joint_pos) + KD * (target_joint_vel - joint_vel) + FEEDFORWARD
       ####
      
       return torque


   def print_info(self):
       """Print joint information every 2 control loops"""
       if self.print_counter == 0:
           self.get_logger().info(
               f"Pos: {self.joint_pos:.2f}, Target Pos: {self.target_joint_pos:.2f}, Vel: {self.joint_vel:.2f}, Target Vel: {self.target_joint_vel:.2f}, Tor: {self.calculated_torque:.2f}"
           )
       self.print_counter += 1
       self.print_counter %= 2


   def get_joint_info(self, msg):
       """Callback function to process incoming JointState messages"""
       joint_index = msg.name.index(JOINT_NAME)
       joint_pos = msg.position[joint_index]
       joint_vel = msg.velocity[joint_index]


       self.joint_pos = joint_pos
       self.joint_vel = joint_vel


       return joint_pos, joint_vel


   def control_loop(self):
       """Control control loop to calculate and publish torque commands"""


       self.angle_buffer.append(self.joint_pos)
       self.velocity_buffer.append(self.joint_vel)
       delayed_pos = self.angle_buffer[0]
       delayed_vel = self.velocity_buffer[0]


       self.target_joint_pos, self.target_joint_vel = self.get_target_joint_info()
       self.calculated_torque = self.calculate_torque(
           delayed_pos, delayed_vel, self.target_joint_pos, self.target_joint_vel
       )
       self.print_info()
       self.publish_torque(self.calculated_torque)


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


   # Install a SIGINT handler that sends zero torque BEFORE shutting down the context
   def _handle_sigint(sig, frame):
       joint_state_subscriber.get_logger().info("SIGINT received: sending zero torque and shutting down...")
       joint_state_subscriber.publish_torque(0.0)
       time.sleep(0.1)
       joint_state_subscriber.publish_torque(0.0)
       rclpy.shutdown()


   signal.signal(signal.SIGINT, _handle_sigint)


   rclpy.spin(joint_state_subscriber)


if __name__ == "__main__":
   main()



