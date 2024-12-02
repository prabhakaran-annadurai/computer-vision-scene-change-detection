import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_srvs.srv import Empty
from gazebo_msgs.srv import DeleteEntity
import cv2
from cv_bridge import CvBridge
from math import atan2
import time
import random
import subprocess
from pathlib import Path

class Record(Node):
    def __init__(self):
        super().__init__('scene_record')
        self.recordings_root_path="/home/prabha/Desktop/recordings/"
        self.training_path=self.recordings_root_path+"training/scene/"
        self.validation_path=self.recordings_root_path+"validation/scene/"
        self.testing_path=self.recordings_root_path+"testing/scene/"
        
        self.target_x = 6
        self.testing=True
        self.y_right_limit=random.gauss(0.0, 0.2)
        self.straight_path_angle=0.0

        self.bridge = CvBridge()
        
        self.run=1
        self.max_runs=20

    def random_initialize(self):
        self.current_x = 0.0
        self.current_y = 0.0
        self.initial_x_velocity=0.5
        self.max_angle=random.gauss(0.65, 0.1)
        self.initial_z_deviation=-0.6
        self.video_writer = None
        if self.testing: #testing
            self.recordings_path=self.testing_path
            Path(self.testing_path).mkdir(parents=True, exist_ok=True)
        else: #training
            if self.run <=10:
                self.recordings_path=self.training_path
                Path(self.training_path).mkdir(parents=True, exist_ok=True)
            else:
                self.recordings_path=self.validation_path
                Path(self.validation_path).mkdir(parents=True, exist_ok=True)

    def start_loop(self):
        if(self.spawn_entity()):
            time.sleep(2)
            self.random_initialize()
            self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)    
            self.odmsubscription_ = self.create_subscription(
                Odometry,
                '/odom',
                self.odom_callback,
                10
            )
            self.videosubscription_ = self.create_subscription(
                Image,
                '/camera_sensor/image_raw',
                self.image_callback,
                10) 
            self.get_logger().info(f"self.max_angle {self.max_angle}")
        else:
            self.get_logger().info(f"not able to spawn the robot")

    def spawn_entity(self):
        # Construct the command to spawn the entity
        command = [
            'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
            '-entity', "my_robot",
            '-topic', '/robot_description',
            '-y','3.0'
        ]

        try:
            # Run the command to spawn the entity
            self.get_logger().info(f"Spawning entity")
            subprocess.run(command, check=True)
            self.get_logger().info(f"Entity spawned successfully.")
            return True
        except subprocess.CalledProcessError as e:
            self.get_logger().error(f"Failed to spawn entity: {e}")
            return False
        
    def image_callback(self, msg):
        #self.get_logger().info('Video recording...')
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        if self.video_writer is None:
            height, width, _ = cv_image.shape
            self.video_writer = cv2.VideoWriter(self.recordings_path+str(self.run)+".avi", cv2.VideoWriter_fourcc(*'XVID'), 20, (width, height))
        self.video_writer.write(cv_image)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)
        deviation_angle = yaw - self.straight_path_angle
        deviation_angle = (deviation_angle + 3.14159) % (2 * 3.14159) - 3.14159
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        #self.get_logger().info(f'Odometry: x = {self.current_x:.2f}, y = {self.current_y:.2f}')

        # Check if the robot has reached the target distance
        if self.current_x >= self.target_x:            
            self.get_logger().info('Reached target distance. Stopping robot.')
            self.terminate_loop()
            #rclpy.shutdown()
            #return

        # Correct the path if deviating from the center
        self.correct_path(deviation_angle)

    def quaternion_to_yaw(self, quaternion):
        qx, qy, qz, qw = (
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        )

        # Yaw calculation
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def correct_path(self,deviation_angle):
        cmd = Twist()
        cmd.linear.x=self.initial_x_velocity
        gaussian_noise=random.gauss(0, 0.2)
        if self.current_y < self.y_right_limit:
            cmd.angular.z= abs(self.initial_z_deviation)/5 + gaussian_noise
        elif  abs(deviation_angle) <= self.max_angle :
            cmd.angular.z= -abs(self.initial_z_deviation)/5 + gaussian_noise
        else:
            cmd.angular.z=0.0

        self.publisher_.publish(cmd)

    def remove_robot(self):
        self.get_logger().info(f"Deleting my_robot from gazebo")
        client = self.create_client(DeleteEntity, '/delete_entity')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /delete_entity service...')
        request = DeleteEntity.Request()
        request.name = "my_robot"
        future = client.call_async(request)
        time.sleep(2)

    def stop_robot(self):
        # Publish a stop command
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = 0.0
        self.publisher_.publish(cmd)

        #complete video recording
        time.sleep(2)
        self.video_writer.release()
        self.remove_robot()

    def terminate_loop(self):
        self.destroy_subscription(self.odmsubscription_)
        self.destroy_subscription(self.videosubscription_)
        self.stop_robot()
        if self.run == self.max_runs:
            self.get_logger().info(f"--- completed caputring videos ---")
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.run+=1
            self.get_logger().info(f" start next loop {self.run}")
            self.start_loop()

def main(args=None):
    try:
        rclpy.init(args=args)
        scene_recorder = Record()
        scene_recorder.start_loop()
        rclpy.spin(scene_recorder)

    except KeyboardInterrupt:
        pass
    finally:
        scene_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()