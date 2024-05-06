#!/usr/bin/env python3

###############################################
##      Open CV and Numpy integration        ##
###############################################
# get number of humans
# publish when human detected
# distance of human from robot/camera

import os
import cv2
import numpy as np
from pathlib import Path
import pyrealsense2 as rs
from skimage.segmentation import flood

import torch
import rclpy
import onnxruntime
import matplotlib.pyplot as plt

#ROS2 Imports
from rclpy.node import Node
from sensor_msgs.msg import Image
from navthon_msgs.msg import HumanDetection as HDMsg

from det_utils import non_max_suppression, process_image, scale_boxes

class HumanDetection:
    def __init__(self, model_path):
        self.model_path = Path(model_path)
        assert self.model_path.suffix=='.onnx' and self.model_path.exists()
        
        self.detecton_threshold = 0.3
        self.segmentation_threshold = 0.01
        
        self.color = np.random.randint(0, 255, 3, dtype=np.uint8)
        
        print("Loading Onnx Model")
        self.loadOnnxModel()
        print("Done")
        
        print("Setting RealSense Camera")
        self.setupRealSense()
        print("Done")
        
        self.count = 0
        self.dists_to_humans = []
        
    def loadOnnxModel(self):
        # Load Onnx model
        cuda = False
        providers = ["CUDAExecutionProvider", "CPUExecutionProvider"] if cuda else ["CPUExecutionProvider"]
        self.session = onnxruntime.InferenceSession(str(self.model_path), providers=providers)
        self.output_names = [x.name for x in self.session.get_outputs()]

    def setupRealSense(self):
        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.pipeline)
        pipeline_profile = config.resolve(pipeline_wrapper)
        device = pipeline_profile.get_device()
        device_product_line = str(device.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in device.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)

        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

        if device_product_line == 'L500':
            config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
        else:
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

        # Start streaming
        profile = self.pipeline.start(config)

        depth_sensor = profile.get_device().first_depth_sensor()
        self.depth_scale = depth_sensor.get_depth_scale()
        
        dstream = profile.get_stream(rs.stream.depth)
        intrinsic = dstream.as_video_stream_profile().get_intrinsics()
        self.width = intrinsic.width
        self.height = intrinsic.height
        self.fx = intrinsic.fx
        self.fy = intrinsic.fy
        self.cx = intrinsic.ppx
        self.cy = intrinsic.ppy    
        align_to = rs.stream.color
        self.align = rs.align(align_to)
        
    def processScan(self)->bool:
        frames = self.pipeline.wait_for_frames()
        # Align the depth frame to color frame
        aligned_frames = self.align.process(frames)

        # Get aligned frames
        aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
        color_frame = aligned_frames.get_color_frame()

        # Validate that both frames are valid
        if not aligned_depth_frame or not color_frame:
            return False

        # Convert images to numpy arrays
        self.depth_image = np.asanyarray(aligned_depth_frame.get_data())*self.depth_scale
        self.color_image = np.asanyarray(color_frame.get_data())[:,:,:3]
        
        return True

    def performHumanDetection(self):
        # Person Detection
        img = process_image(self.color_image[:,:,::-1])
        pred = self.session.run(self.output_names, {self.session.get_inputs()[0].name: img})
        pred = torch.from_numpy(pred[0])
        pred_nms = non_max_suppression(prediction=pred, conf_thres=0.3, iou_thres=0.45, classes=0, agnostic=False, max_det=50,nm=0) #32 is for seg
        for i, det in enumerate(pred_nms):
            det[:, :4] = scale_boxes([480, 640], det[:, :4], [480, 640]).round()

        if len(det):
            num_humans = 0
            self.dists_to_humans = []
            for d in det:
                if d[-2]>=self.detecton_threshold and ((d[2]-d[0]) * (d[3]-d[1]))>50:
                    num_humans += 1
                    self.count = num_humans
                    d = d[:4].to(int).tolist()
                    depth_det = self.depth_image[d[1]:d[3], d[0]:d[2]]
                    indexes = np.argwhere(depth_det==np.median(depth_det))
                    #print(len(indexes))
                    mask = np.zeros_like(self.depth_image, dtype=bool)
                    mask[d[1]:d[3], d[0]:d[2]] = flood(depth_det, seed_point=(indexes[0,0],indexes[0,1]), connectivity=1, tolerance=self.segmentation_threshold)
                    # mask[d[1]:d[3], d[0]:d[2]] = flood(self.depth_image, seed_point=(indexes[0,0],indexes[0,1]), connectivity=1, tolerance=self.segmentation_threshold)
                    
                    index = np.argwhere(mask).mean(axis=0).astype(int)
                    z = self.depth_image[mask].mean()
                    x = (float(index[1]) - self.cx)*z/self.fx
                    y = (float(index[0]) - self.cy)*z/self.fy
                    
                    #Segmentation Mask
                    masked_img = np.where(mask[...,None], self.color, self.color_image[:,:,:3])
                    self.color_image = cv2.addWeighted(self.color_image[:,:,:3], 0.8, masked_img, 0.2, 0)
                    
                    #Bounding Box
                    box_org = (d[0], d[1])
                    box_end = (d[2], d[3])
                    self.color_image = cv2.rectangle(self.color_image, box_org, box_end, self.color.tolist(), 2)
                    
                    #Distance Text
                    text_org = (d[0], d[1])
                    # text = f'X:{x:.2f}, Y:{y:.2f}, Z:{z:.2f}'
                    text = f'Z:{z:.2f}'
                    self.dists_to_humans.append(z)
                    self.color_image = cv2.putText(self.color_image, text, text_org, cv2.FONT_HERSHEY_SIMPLEX, 1., self.color.tolist(), 2, cv2.LINE_AA) 
        else:
            self.dists_to_humans = []
            self.count = 0
    # def run(self):
    #     try:
    #         while True:
    #             is_new = self.processScan()
    #             if is_new:
    #                 self.performHumanDetection()
    #             cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    #             cv2.imshow('RealSense', self.color_image)
    #             cv2.waitKey(1)
    #     finally:
    #         # Stop streaming
    #         self.pipeline.stop()

    def run(self):
        is_new = self.processScan()
        if is_new:
            self.performHumanDetection()
        #cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        #cv2.imshow('RealSense', self.color_image)
        #cv2.waitKey(1)


class HumanDetectionRos(Node):
    def __init__(self):
        super().__init__('human_detection_node')
        #Declare ROS2 parameters
        self.declare_parameter("namespace", "")
        self.declare_parameter("topic_name", "human_detection")
        self.declare_parameter("module_path", "src/naviathon/sensing/navthon_human_detection/src/detector/humandet.onnx")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.topic_name = self.get_parameter("topic_name").get_parameter_value().string_value
        self.module_path = self.get_parameter("module_path").get_parameter_value().string_value
        self.publisher_ = self.create_publisher(HDMsg, self.topic_name, 10)
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.hd_callback)
        self.humnadet = HumanDetection(self.module_path)
       

    def hd_callback(self):
        self.humnadet.run()
        msg = HDMsg()
        humans = self.humnadet.count
        msg.human_detected = False
        if (humans > 0):
            msg.human_detected = True
        msg.distance_to_human = self.humnadet.dists_to_humans
        msg.human_number = humans
        self.publisher_.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    hd_node = HumanDetectionRos()
    rclpy.spin(hd_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()



# self.count += 1
# png.from_array(self.depth_image, 'L').save("depth/{:4d}.png".format(self.count))
# plt.imsave("color/{:4d}.png".format(self.count), self.color_image[:,:,::-1])
# Image.fromarray(self.color_image[:,:,::-1]).save("color/{:4d}.png".format(self.count))

# We will be removing the background of objects more than
#  clipping_distance_in_meters meters away
# clipping_distance_in_meters = 1 #1 meter
# clipping_distance = clipping_distance_in_meters / depth_scale

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.

# pc = rs.pointcloud()

# try:
#     while True:

#         # Wait for a coherent pair of frames: depth and color
#         frames = pipeline.wait_for_frames()
#         # Align the depth frame to color frame
#         aligned_frames = align.process(frames)

#         # Get aligned frames
#         aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
#         color_frame = aligned_frames.get_color_frame()

#         # Validate that both frames are valid
#         if not aligned_depth_frame or not color_frame:
#             continue

#         # Convert images to numpy arrays
#         depth_image = np.asanyarray(aligned_depth_frame.get_data())
#         color_image = np.asanyarray(color_frame.get_data())
        
        # Pointcloud data to arrays
        # points = pc.calculate(aligned_depth_frame)
        # v= points.get_vertices()
        # t = points.get_texture_coordinates()
        # verts = np.asanyarray(v).view(np.float32).reshape(-1, 3)  # xyz
        # texcoords = np.asanyarray(t).view(np.float32).reshape(-1, 2)  # uv
        # print(f"Number of points: {verts.shape}, {texcoords.shape}")
        
        

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        # depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        # depth_colormap_dim = depth_colormap.shape
        # color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        # if depth_colormap_dim != color_colormap_dim:
        #     resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
        #     images = np.hstack((resized_color_image, depth_colormap))
        # else:
        #     images = np.hstack((color_image, depth_colormap))

        # # Show images
        # cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
        # cv2.imshow('RealSense', images)
        # cv2.waitKey(1)

# finally:

#     # Stop streaming
#     pipeline.stop()
