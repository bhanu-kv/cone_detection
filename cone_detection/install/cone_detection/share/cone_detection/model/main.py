#!/usr/bin/python3

import rclpy
from PIL import Image
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import cv2 as cv
import torch
import pathlib
from ultralytics import YOLO
import math
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry

class ConeDetectionNode(Node):
    def __init__(self):
        super().__init__('cone_detection')

        self.camera1_orientation = -2.355
        self.camera2_orientation = -0.785

        self.rgb_camera1 = None
        self.rgb_camera2 = None

        self.new_centroids = {}
       
        self.model = YOLO("/home/bhanu/abhiyaan/cone_detection/src/cone_detection/best.pt")

        # self.classNames = ["Barrel", "No-Turns", "One-Way", "Person", "Road-Closed", "Stop-Sign", "Tire"]
        self.classNames = ["cone"]
        
        self.subscription_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        
        # self.subscription_rgb = self.create_subscription(
        #     Image,
        #     # '/zed/zed_node/depth/depth_registered',
        #     # '/zed/zed_node/rgb/image_rect_color',
        #     '/camera/image_raw',
        #     self.image_callback,
        #     10)
            
        # self.subscription_caminfo = self.create_subscription(
        #     CameraInfo,
        #     # '/zed/zed_node/depth/depth_registered',
        #     # '/zed/zed_node/depth/camera_info',
        #     '/depth_camera/camera_info',
        #     self.caminfo_callback,
        #     10)
            
        # self.subscription_depth = self.create_subscription(
        #     Image,
        #     # '/zed/zed_node/depth/depth_registered',
        #     '/depth_camera/depth/image_raw',
        #     self.depth_callback,
        #     10)

        self.subscription_cam1info = self.create_subscription(
            CameraInfo,
            # '/zed/zed_node/depth/depth_registered',
            # '/zed/zed_node/depth/camera_info',
            '/depth_camera1/camera_info',
            self.caminfo_callback,
            10)
        
        self.subscription_camera1_depth = self.create_subscription(
            Image,
            # '/zed/zed_node/depth/depth_registered',
            '/depth_camera1/depth/image_raw',
            self.depth_callback,
            10)
        
        
        self.subscription_camera1 = self.create_subscription(
            Image,
            # '/zed/zed_node/depth/depth_registered',
            # '/zed/zed_node/rgb/image_rect_color',
            '/camera1/image_raw',
            lambda msg: self.image_callback(msg, camera_name = "camera1"),
            10)
        
        self.subscription_cam1info = self.create_subscription(
            CameraInfo,
            # '/zed/zed_node/depth/depth_registered',
            # '/zed/zed_node/depth/camera_info',
            '/depth_short_1_camera/camera_info',
            self.caminfo_callback,
            10)
        
        self.subscription_camera1_depth = self.create_subscription(
            Image,
            # '/zed/zed_node/depth/depth_registered',
            '/depth_short_1_camera/depth/image_raw',
            self.depth_callback,
            10)
        
        self.subscription_camera1 = self.create_subscription(
            Image,
            # '/zed/zed_node/depth/depth_registered',
            # '/zed/zed_node/rgb/image_rect_color',
            '/short_1_camera/image_raw',
            lambda msg: self.image_callback(msg, camera_name = "camera2"),
            10)

        self.publisher_ = self.create_publisher(
            Image,
            '/cones',
            10)
            
        self.detect_pub = self.create_publisher(Pose, 'destination_pose',10)
        self.caminfo = None
        self.depth_img = None
        self.bridge = CvBridge()
        
    def odom_callback(self, msg):
        self.x_pos = msg.pose.pose.position.x
        self.y_pos = msg.pose.pose.position.y
        self.z_pos = msg.pose.pose.position.z
        self.orientation = msg.pose.pose.orientation.z
        
    def depth_callback(self, msg):
        self.depth_img = self.bridge.imgmsg_to_cv2(msg)
        
    def caminfo_callback(self, msg):
        self.caminfo = msg

    def publish_coordinates(self, camera_name):
        l = list(self.new_centroids.keys())
        if len(l) < 2 or self.dual_camera == False:
            return
        
        l.sort()
        
        # print(self.new_centroids)
        # print(l)

        centroids = [self.new_centroids[l[-1]], self.new_centroids[l[-2]]]
        mean_centroid = (0.50*(centroids[0][0]+centroids[1][0]), 0.50*(centroids[0][1]+centroids[1][1]))
        #self.get_logger().info(f"Mean Centroid: {mean_centroid}")
        #self.publish_trajectory(mean_centroid,centroids)

        msg = Pose()
        magnitude = math.sqrt(mean_centroid[0]**2 + mean_centroid[1]**2)
        angle = math.atan(mean_centroid[0]/mean_centroid[1])

        print(magnitude)

        if camera_name == "camera1":
            if(self.orientation>=0 and self.orientation <=3.1416):
                msg.position.x = self.x_pos #+ magnitude*math.cos(self.orientation + (self.camera1_orientation+angle))
                msg.position.y = self.y_pos #+ magnitude*math.sin(self.orientation + (self.camera1_orientation+angle)) 
            else:
                msg.position.x = self.x_pos #+ magnitude*math.cos(-self.orientation + (self.camera1_orientation+angle))
                msg.position.y = self.y_pos #+ magnitude*math.sin(-self.orientation + (self.camera1_orientation+angle))
        
        if camera_name == "camera2":
            if(self.orientation>=0 and self.orientation <=3.1416):
                msg.position.x = self.x_pos #+ magnitude*math.cos(-self.orientation + (self.camera1_orientation+angle))
                msg.position.y = self.y_pos #+ magnitude*math.sin(-self.orientation + (self.camera1_orientation+angle))
            else:
                msg.position.x = self.x_pos #+ magnitude*math.cos(self.orientation + (self.camera1_orientation+angle))
                msg.position.y = self.y_pos #+ magnitude*math.sin(self.orientation + (self.camera1_orientation+angle)) 

        msg.position.z = 0.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0

        self.detect_pub.publish(msg)
        self.get_logger().info(f'Publishing: Position=({msg.position.x}, {msg.position.y}')
        
        return

    def display_image(self):
        if(self.dual_camera == True):
            numpy_horizontal_concat = np.concatenate((self.rgb_camera1, self.rgb_camera2), axis=1)
            cv.imshow("IMAGE", numpy_horizontal_concat)
            cv.waitKey(1)
        
        return

    def cone_detection(self, rgb, camera_name):
        # Add parameter settings or modifications as needed
        # rgb = cv.flip(rgb, 0)
        
        results = self.model(rgb, verbose = False)
        # img_rgb = cv.cvtColor(rgb, cv.COLOR_BGR2RGB)
        # img_rgb = cv.resize(rgb, (200, 200))
        # img_rgb = rgb

        cl_box = []
        camera_factor = self.caminfo.k[-1]
        camera_cx = self.caminfo.k[2]
        camera_cy = self.caminfo.k[5]
        camera_fx = self.caminfo.k[0]
        camera_fy = self.caminfo.k[4]

        if camera_name == "camera1":
            self.new_centroids = {}
            self.dual_camera = False
        else:
            self.dual_camera = True
        
        for r in results:
            boxes = r.boxes
    
            for box in boxes:
                # bounding box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2) # convert to int values

                cl_box.append(((x1+x2)/2, (y1+y2)/2))
                confidence = math.ceil((box.conf[0]*100))/100
                
                if(confidence < 0.6):
                    continue
                    
                cv.rectangle(rgb, (x1, y1), (x2, y2), (255, 0, 255), 3)
                u = int((x1+x2)/2)
                v = int((y1+y2)/2)
                                
                z = self.depth_img[v][u] / camera_factor
                x = (u - camera_cx) * z / camera_fx
                y = (v - camera_cy) * z / camera_fy

                # confidence
                confidence = math.ceil((box.conf[0]*100))/100
                # print("Confidence --->",confidence)
                # print(f'point is at:({x} , {y}, {z})')
                # class name
                cls = int(box.cls[0])
                # print("Class name -->", self.classNames[cls])
                coordinates=[]
                coordinates.append([confidence,x,y,z])

                # cone details
                org = [x1, y1]
                font = cv.FONT_HERSHEY_SIMPLEX
                fontScale = 1
                color = (255, 0, 0)
                thickness = 2
    
                cv.putText(rgb, self.classNames[cls], org, font, fontScale, color, thickness)
        
                if coordinates:
                    if not coordinates[0]:
                        continue
                    
                    for i in coordinates:
                        confidence, x, z = i[0], i[1], i[3]
                        self.new_centroids[confidence] = [i[1], i[3]]

        self.publish_coordinates(camera_name)

        if camera_name == "camera1":
            self.rgb_camera1 = rgb
        else:
            self.rgb_camera2 = rgb

        self.display_image()
        
        return rgb

    def image_callback(self, data, camera_name):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return
        if self.depth_img is not None and self.caminfo is not None:
            final = self.cone_detection(cv_image, camera_name = camera_name)

            try:
                self.publisher_.publish(self.bridge.cv2_to_imgmsg(final))
            except CvBridgeError as e:
                self.get_logger().error(str(e))

def main(args=None):
    rclpy.init(args=args)
    node = ConeDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
