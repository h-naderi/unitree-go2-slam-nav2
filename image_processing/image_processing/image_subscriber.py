#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',  # Adjust the topic name as needed
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Initialize CvBridge
        self.bridge = CvBridge()

        # Define the directory path
        self.save_dir = os.path.expanduser('~/robot_videos')

        # Create the directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)

        # Initialize variables for VideoWriter
        self.out = None
        self.fps = 30.0  # Adjust as needed
        self.output_path = os.path.join(self.save_dir, 'output_video.avi')

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        current_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Initialize VideoWriter if not already done
        if self.out is None:
            height, width, channels = current_frame.shape
            fourcc = cv2.VideoWriter_fourcc(*'XVID')  # Codec for .avi files
            self.out = cv2.VideoWriter(self.output_path, fourcc, self.fps, (width, height))

        # Write the frame to the video file
        self.out.write(current_frame)

        # Display the frame (optional)
        cv2.imshow('Camera', current_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        # Release the VideoWriter and close OpenCV windows
        if self.out:
            self.out.release()
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)
    image_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

