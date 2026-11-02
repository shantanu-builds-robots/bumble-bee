#!/usr/bin/python3


import rclpy
from rclpy.node import Node
import numpy as np

from drone_plugin.msg import MotorSpeed                            # CHANGE
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64
import tf_transformations

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        
        # Subscribe to the motor_speed topic
        self.motor_speed_subscription = self.create_subscription(MotorSpeed, '/motor_speed', self.motor_speed_callback, 10)  # CHANGE
        
        # Subscribe to drone velocity
        self.drone_velocity_subscription = self.create_subscription(Twist, '/drone_velocity', self.drone_velocity_callback, 10)
        
        # Subscribe to drone pose
        self.drone_pose_subscription = self.create_subscription(PoseStamped, '/drone_pose', self.drone_pose_callback, 10)
        
        
        timer_period = 0.5
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        
        
        # Publish Estimated State (Height)
        self.est_publisher_ = self.create_publisher(Float64, '/estimated_height', 10)  # CHANGE
        
        # Publish Observer State (Height)
        self.obs_publisher_ = self.create_publisher(Float64, '/observer_height', 10)
        
        
        # Drone Parameters
        self.m = 1.405 # mass of the drone
        self.g = 9.8 # gravity
        self.k_th = 0.00025 # thrust coefficient
        self.k_to = 0.0000074 # torque coefficient
        self.l = 0.12 # arm length
        
        distance = 0.167
        self.Ixx = 0.0833333 * self.m * (distance * distance + distance/4 * distance/4)
        self.Iyy = 0.0833333 * self.m * (distance * distance + distance/4 * distance/4)
        self.Izz = 0.0833333 * self.m * (distance * distance + distance * distance)
        self.a = 0.707 # angle between the arm and the x-axis (45 degrees)
        
        # State Variables
        self.U = np.zeros((4,1)) # control input
        self.X = np.zeros((12,1)) # state vector
        self.Y = np.zeros((12,1))
        self.X[0] = 0.99 # initial height
        
        self.X_obs = np.zeros((12,1))
        
        self.observed_height = 0.0
        self.estimated_height = 0.0
        
        
        self.current_time = 0.0
        self.previous_time = 0.0
        self.dt = 0.0
        

    def timer_callback(self):
        if self.U[0] == 0.0:
            return
        print(f"Current Time: {self.current_time}")
        A,B,C,D = self.drone_model(m=self.m, g=self.g, Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz)
        G= np.array([[0,self.g,0,0,0,0,0,0,0,0,0,0]]).T
        # self.get_logger().info('X[1] before: "%f"' % self.X[1])
        X_dot = np.dot(A, self.X) + np.dot(B, self.U)-G
        print("Prediction")
        print(f"Acceleration: {X_dot[1]}")
        print(f"Velocity: {X_dot[0]}")
        self.dt = self.current_time - self.previous_time
        if self.dt > 1.0:
            self.dt = 0.5
        # print(f"X_dot (Acceleration): {X_dot[1]} X (Velocity): {self.X[1]}")
        # self.get_logger().info("X_dot (Acceleration): %f X (Velocity): %f" % (X_dot[1], self.X[1]))
        self.X = self.X + self.dt*X_dot
        self.Y = np.dot(C, self.X) + np.dot(D, self.U)
        
        self.previous_time = self.current_time
        
        self.estimated_height = float(self.Y[0])
        
        print(f"Time difference: {self.dt}")
        # self.get_logger().info('Estimated Height: "%f"' % self.estimated_height)
        # self.get_logger().info('Estimated Velocity: "%f"' % (self.Y[1]))
        # self.get_logger().info('Observed Height: "%f"' % self.observed_height)
        self.get_logger().info('Control Input: "%f %f %f %f"' % (self.U[0], self.U[1], self.U[2], self.U[3]))
        
        # msg = Float64()
        # msg.data = self.estimated_height
        
        # self.est_publisher_.publish(msg)
        
        # msg.data = self.observed_height
        # self.obs_publisher_.publish(msg)
        
        print("Observed State")
        print(f"Velocity: {self.X_obs[1]}")
        # self.get_logger().info('')
        
        
        
        
        
    def drone_velocity_callback(self, msg):
        self.X_obs[1] = msg.linear.z
        self.X_obs[5] = msg.linear.x
        self.X_obs[9] = msg.linear.y
        self.X_obs[3] = msg.angular.x
        self.X_obs[7] = msg.angular.y
        self.X_obs[11] = msg.angular.z
        
    def drone_pose_callback(self, msg):
        quaternion = (msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w)
        roll, pitch, yaw = roll, pitch, yaw = tf_transformations.euler_from_quaternion(quaternion)
        self.X_obs[0] = msg.pose.position.z
        self.X_obs[4] = msg.pose.position.x
        self.X_obs[8] = msg.pose.position.y
        self.X_obs[2] = roll
        self.X_obs[6] = pitch
        self.X_obs[10] = yaw
        # self.observed_height = msg.pose.position.z
        
        



    def motor_speed_callback(self, msg):
        prop1_speed = msg.velocity[0]
        prop2_speed = msg.velocity[1]
        prop3_speed = msg.velocity[2]
        prop4_speed = msg.velocity[3]
        # self.get_logger().info('Motor Speeds: "%f %f %f %f"' % (prop1_speed, prop2_speed, prop3_speed, prop4_speed))
        Ui = np.array([[prop1_speed**2], [prop2_speed**2], [prop3_speed**2], [prop4_speed**2]])
        T = np.array([[self.k_th, self.k_th, self.k_th, self.k_th],
                     [self.k_th*self.l*np.cos(self.a), self.k_th*self.l*np.cos(self.a), -self.k_th*self.l*np.cos(self.a), -self.k_th*self.l*np.cos(self.a)],
                     [self.k_th*self.l*np.sin(self.a), -self.k_th*self.l*np.sin(self.a), -self.k_th*self.l*np.sin(self.a), self.k_th*self.l*np.sin(self.a)],
                     [self.k_to, -self.k_to, self.k_to, -self.k_to]])
        self.U = np.dot(T, Ui)
        
        self.current_time = rclpy.clock.Clock().now().nanoseconds / 1e9
        self.dt = self.current_time - self.previous_time
        if self.dt > 1.0:
            self.dt = 0.1
        print(f"Time difference: {self.dt}")
        self.previous_time = self.current_time
        
        A,B,C,D = self.drone_model(m=self.m, g=self.g, Ixx=self.Ixx, Iyy=self.Iyy, Izz=self.Izz)
        G= np.array([[0,self.g,0,0,0,0,0,0,0,0,0,0]]).T

        X_dot = np.dot(A, self.X) + np.dot(B, self.U)-G
        print("Prediction")
        print(f"Acceleration: {X_dot[1]}")
        print(f"Velocity: {X_dot[0]}")
        
        
        
        self.X = self.X + self.dt*X_dot
        self.Y = np.dot(C, self.X) + np.dot(D, self.U)
        
        
        
        self.estimated_height = float(self.Y[0])
        
        self.get_logger().info('Control Input: "%f %f %f %f"' % (self.U[0], self.U[1], self.U[2], self.U[3]))
        
        print("Observed State")
        print(f"Velocity: {self.X_obs[1]}")
        
        
        
    
    def drone_model(self,g=9.81,m=0.03,Ixx=1e-5,Iyy=1e-5,Izz=1e-5):
        A = np.array([[0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, g, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],
                    [0 ,0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -g, 0],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1],
                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0] ])
        B = np.array([[0, 0, 0, 0],
                    [1/m, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 1/Ixx, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 1/Iyy, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 0],
                    [0, 0, 0, 1/Izz]])    
        C = np.eye(12)
        D = np.zeros((12,4))
        return A, B, C, D
    
    
        
        
        
            
        
    
def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


