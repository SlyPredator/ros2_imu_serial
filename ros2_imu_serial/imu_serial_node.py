#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped
import serial
import threading
import time
import math
import numpy as np
import re

class IMUSerialNode(Node):
    def __init__(self):
        super().__init__('imu_serial_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('use_madgwick', True)
        self.declare_parameter('madgwick_beta', 0.1)
        self.declare_parameter('publish_euler', True)  

        self.serial_port = self.get_parameter('serial_port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_madgwick = self.get_parameter('use_madgwick').value
        self.beta = self.get_parameter('madgwick_beta').value
        self.publish_euler = self.get_parameter('publish_euler').value
        
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)

        if self.publish_euler:
            self.euler_pub = self.create_publisher(Vector3Stamped, 'imu/euler', 10)
        
        # Variables for Madgwick filter
        self.q = np.array([1.0, 0.0, 0.0, 0.0])  # Quaternion [w, x, y, z]
        self.last_time = None
        
        # Data buffer and latest data
        self.data_buffer = ""
        self.latest_imu_data = None
        self.latest_euler = None

        self.serial_conn = None
        self.running = True
        
        # Start serial thread
        self.serial_thread = threading.Thread(target=self.serial_read_thread)
        self.serial_thread.daemon = True
        self.serial_thread.start()

        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
    def connect_serial(self):
        """Connect to serial port"""
        while self.running and (self.serial_conn is None or not self.serial_conn.is_open):
            try:
                self.get_logger().info(f'Attempting to connect to {self.serial_port}...')
                self.serial_conn = serial.Serial(
                    port=self.serial_port,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    rtscts=False,
                    dsrdtr=False,
                    xonxoff=False
                )
                time.sleep(2)
                self.get_logger().info(f'Successfully connected to {self.serial_port}')
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.data_buffer = ""
                    
                self.get_logger().info(f'IMU Serial Node started on {self.serial_port} at {self.baudrate} baud')
                self.get_logger().info(f'Publishing to /imu/data_raw at {self.publish_rate} Hz')
                if self.publish_euler:
                    self.get_logger().info(f'Also publishing Euler angles to /imu/euler')
            except serial.SerialException as e:
                self.get_logger().warn(f'Failed to connect: {e}. Retrying in 2 seconds...')
                time.sleep(2)
            except Exception as e:
                self.get_logger().error(f'Unexpected error: {e}')
                time.sleep(2)
    
    def serial_read_thread(self):
        """Reading serial data"""
        self.connect_serial()
        
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    if self.serial_conn.in_waiting > 0:
                        try:
                            raw_data = self.serial_conn.read(self.serial_conn.in_waiting).decode('utf-8', errors='ignore')
                            self.data_buffer += raw_data

                            while '\n' in self.data_buffer:
                                line_end = self.data_buffer.find('\n')
                                line = self.data_buffer[:line_end].strip()
                                self.data_buffer = self.data_buffer[line_end + 1:]
                                
                                if line:
                                    self.process_serial_line(line)
                        except UnicodeDecodeError as e:
                            self.get_logger().warn(f'Unicode decode error: {e}')
                            self.data_buffer = ""
                    else:
                        time.sleep(0.001)
                else:
                    time.sleep(0.1)
                    
            except serial.SerialException as e:
                self.get_logger().error(f'Serial error: {e}. Reconnecting...')
                try:
                    if self.serial_conn:
                        self.serial_conn.close()
                except:
                    pass
                self.serial_conn = None
                self.connect_serial()
            except Exception as e:
                self.get_logger().error(f'Error in serial thread: {e}', throttle_duration_sec=5)
                time.sleep(0.1)
    
    def process_serial_line(self, line):
        """Process a single line of serial data"""
        if line.startswith('$'):
            try:
                # Use regex to find all complete messages
                pattern = r'\$([^$]+)'
                matches = re.findall(pattern, '$' + line)
                
                for match in matches:
                    values_str = match.split(',')
                    
                    if len(values_str) >= 6:
                        accel_x = float(values_str[0])
                        accel_y = float(values_str[1])
                        accel_z = float(values_str[2])
                        gyro_x = float(values_str[3])
                        gyro_y = float(values_str[4])
                        gyro_z = float(values_str[5])
                        
                        if (abs(accel_x) < 100 and abs(accel_y) < 100 and abs(accel_z) < 100 and
                            abs(gyro_x) < 20 and abs(gyro_y) < 20 and abs(gyro_z) < 20):

                            imu_data = {
                                'linear_acceleration': [accel_x, accel_y, accel_z],
                                'angular_velocity': [gyro_x, gyro_y, gyro_z],
                                'timestamp': time.time()
                            }
                            
                            self.latest_imu_data = imu_data
                            
                            # Apply Madgwick filter
                            if self.use_madgwick:
                                self.update_madgwick_filter([gyro_x, gyro_y, gyro_z], 
                                                           [accel_x, accel_y, accel_z])
                                self.calculate_euler_angles()
                        else:
                            self.get_logger().warn(f'Invalid IMU values: {values_str}', 
                                                  throttle_duration_sec=5)
                    else:
                        self.get_logger().warn(f'Incomplete data: {values_str}', 
                                              throttle_duration_sec=5)
                        
            except ValueError as e:
                self.get_logger().warn(f'Failed to parse IMU data: {line}. Error: {e}', 
                                      throttle_duration_sec=5)
            except Exception as e:
                self.get_logger().error(f'Unexpected parsing error: {e}', 
                                       throttle_duration_sec=5)
    
    def update_madgwick_filter(self, gyro, accel):
        """Madgwick filter implementation for orientation estimation"""
        current_time = time.time()
        
        if self.last_time is None:
            self.last_time = current_time
            return
        
        dt = current_time - self.last_time
        self.last_time = current_time
        
        if dt <= 0:
            return

        norm = math.sqrt(accel[0]**2 + accel[1]**2 + accel[2]**2)
        if norm > 0:
            accel = [a/norm for a in accel]
        q = self.q

        f = [
            2*(q[1]*q[3] - q[0]*q[2]) - accel[0],
            2*(q[0]*q[1] + q[2]*q[3]) - accel[1],
            2*(0.5 - q[1]**2 - q[2]**2) - accel[2]
        ]
        
        j = [
            [-2*q[2], 2*q[3], -2*q[0], 2*q[1]],
            [2*q[1], 2*q[0], 2*q[3], 2*q[2]],
            [0, -4*q[1], -4*q[2], 0]
        ]

        gradient = np.zeros(4)
        for i in range(3):
            for k in range(4):
                gradient[k] += j[i][k] * f[i]

        norm_grad = math.sqrt(sum([g**2 for g in gradient]))
        if norm_grad > 0:
            gradient = [g/norm_grad for g in gradient]

        q_dot = 0.5 * np.array([
            -q[1]*gyro[0] - q[2]*gyro[1] - q[3]*gyro[2],
            q[0]*gyro[0] + q[2]*gyro[2] - q[3]*gyro[1],
            q[0]*gyro[1] - q[1]*gyro[2] + q[3]*gyro[0],
            q[0]*gyro[2] + q[1]*gyro[1] - q[2]*gyro[0]
        ])
        
        q_dot -= self.beta * np.array(gradient)

        q = q + q_dot * dt

        norm = math.sqrt(sum([qi**2 for qi in q]))
        self.q = [qi/norm for qi in q]
    
    def calculate_euler_angles(self):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        w, x, y, z = self.q
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Convert to degrees
        roll_deg = math.degrees(roll)
        pitch_deg = math.degrees(pitch)
        yaw_deg = math.degrees(yaw)

        self.latest_euler = [roll_deg, pitch_deg, yaw_deg]
    
    def timer_callback(self):
        """Publish IMU data at fixed rate"""
        if self.latest_imu_data:
            try:
                current_time = self.get_clock().now()

                imu_msg = Imu()
                imu_msg.header.stamp = current_time.to_msg()
                imu_msg.header.frame_id = self.frame_id
                
                # Set covariance matrices
                imu_msg.linear_acceleration_covariance[0] = 0.01
                imu_msg.linear_acceleration_covariance[4] = 0.01
                imu_msg.linear_acceleration_covariance[8] = 0.01
                
                imu_msg.angular_velocity_covariance[0] = 0.01
                imu_msg.angular_velocity_covariance[4] = 0.01
                imu_msg.angular_velocity_covariance[8] = 0.01
                
                imu_msg.orientation_covariance[0] = -1

                imu_msg.linear_acceleration.x = float(self.latest_imu_data['linear_acceleration'][0])
                imu_msg.linear_acceleration.y = float(self.latest_imu_data['linear_acceleration'][1])
                imu_msg.linear_acceleration.z = float(self.latest_imu_data['linear_acceleration'][2])
                
                imu_msg.angular_velocity.x = float(self.latest_imu_data['angular_velocity'][0])
                imu_msg.angular_velocity.y = float(self.latest_imu_data['angular_velocity'][1])
                imu_msg.angular_velocity.z = float(self.latest_imu_data['angular_velocity'][2])

                if self.use_madgwick:
                    imu_msg.orientation.w = float(self.q[0])
                    imu_msg.orientation.x = float(self.q[1])
                    imu_msg.orientation.y = float(self.q[2])
                    imu_msg.orientation.z = float(self.q[3])
                    imu_msg.orientation_covariance[0] = 0.05
                    imu_msg.orientation_covariance[4] = 0.05
                    imu_msg.orientation_covariance[8] = 0.05
                
                self.imu_pub.publish(imu_msg)
                
                # Publish Euler angles if enabled
                if self.publish_euler and self.use_madgwick and self.latest_euler:
                    euler_msg = Vector3Stamped()
                    euler_msg.header.stamp = current_time.to_msg()
                    euler_msg.header.frame_id = self.frame_id
                    euler_msg.vector.x = self.latest_euler[0] 
                    euler_msg.vector.y = self.latest_euler[1] 
                    euler_msg.vector.z = self.latest_euler[2] 
                    
                    self.euler_pub.publish(euler_msg)

                    self.get_logger().debug(
                        f"Roll: {self.latest_euler[0]:.2f}°, "
                        f"Pitch: {self.latest_euler[1]:.2f}°, "
                        f"Yaw: {self.latest_euler[2]:.2f}°",
                        throttle_duration_sec=1.0
                    )
                
            except Exception as e:
                self.get_logger().error(f'Error publishing IMU data: {e}', 
                                       throttle_duration_sec=5)
    
    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down IMU Serial Node...')
        self.running = False
        if self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1.0)
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = IMUSerialNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()