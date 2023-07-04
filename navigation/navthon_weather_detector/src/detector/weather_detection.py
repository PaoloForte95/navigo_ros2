#!/usr/bin/env python3

import os
import cv2
import onnxruntime
import numpy as np
import rclpy

from cv_bridge import CvBridge, CvBridgeError
from rclpy.node import Node
from sensor_msgs.msg import Image
#Import srv and msg
from navthon_msgs.srv import GetWeatherCondition
from navthon_msgs.msg import WeatherState

class WeatherDetection:
    def __init__(self, model_path:str="image_based_weather_detection.onnx"):
        self._image_size = 456
        self._ort_session = onnxruntime.InferenceSession(model_path)
        self._labels = {
                            0: "Clear",
                            1: "Fog",
                            2: "Rain",
                            3: "Snow",
                       }
        self._mean = np.array([0.485, 0.456, 0.406])
        self._std = np.array([0.229, 0.224, 0.225])
        
        # you can insert the ROS based variables here (subscriber, publishers, or service related variables)
        #
        
    def preprocess_image(self, image):
        '''
        This will be the class function for the ROS2 service
        input:
            img: numpy array (h, w, 3) in bgr format
        '''
        
        h, w, _ = image.shape
        if h<=w:
            h_new = self._image_size
            w_new = int(w * (h_new / h))
        else:
            w_new = self._image_size
            h_new = int(h * (w_new / w))

        image = cv2.resize(image, (w_new, h_new))
        
        x = int(w_new/2 - self._image_size/2)
        y = int(h_new/2 - self._image_size/2)

        crop_img = image[int(y):int(y+self._image_size), int(x):int(x+self._image_size)] # center crop of image
        crop_img = cv2.cvtColor(crop_img, cv2.COLOR_BGR2RGB)
        norm_img = (crop_img/255.0 - self._mean)/self._std
        img_array = (norm_img.transpose((2, 0, 1)))[None,:,:,:]
        return img_array
    
    def get_inference(self, image):
        ort_inputs = {self._ort_session.get_inputs()[0].name: image.astype(np.float32)}
        ort_outs = self._ort_session.run(None, ort_inputs)
        img_out_y = ort_outs[0]

        prob = np.exp(img_out_y[0])/(np.exp(img_out_y[0]).sum())

        preds = {
                    "Clear": prob[0],
                    "Fog": prob[1],
                    "Rain": prob[2],
                    "Snow": prob[3]
                }

        return preds
    
    def get_weather_condition(self, preds):
        state = max(preds, key=preds.get)
        condition = WeatherState()
        match state:
            case "Clear":
                condition.condition = WeatherState.CLEAR
            case "Fog":
                condition.condition = WeatherState.FOG
            case "Rain":
                condition.condition = WeatherState.RAIN
            case "Snow":
                condition.condition = WeatherState.SNOW      
        return condition




class WeahterDetectionService(Node):

    def __init__(self):
        super().__init__('weather_detection_service')
        #Declare ROS2 parameters
        self.declare_parameter("namespace", "")
        self.declare_parameter("service_name", "get_weather_condition")
        self.namespace = self.get_parameter("namespace").get_parameter_value().string_value
        self.service_name = self.get_parameter("service_name").get_parameter_value().string_value
        self.srv = self.create_service(GetWeatherCondition, self.namespace+'/'+ self.service_name, self.get_weather_condition_callback)
        self.module_path = os.getcwd() + "/src/navigation2_oru/navthon_weather_detector"
        self.bridge = CvBridge()

    def get_weather_condition_callback(self, request, response):
        self.get_logger().info('Received image! Computing wheather condition... ')
        #Get the NN
        WD = WeatherDetection(self.module_path + "/src/detector/image_based_weather_detection.onnx")
        #Convert the image
        try:
            img = self.bridge.imgmsg_to_cv2(request.image, desired_encoding='bgr8')
        except CvBridgeError as e:
             print(e)
        #Process the image and get the weather condition
        pimg = WD.preprocess_image(img)
        preds = WD.get_inference(pimg)
        self.get_logger().info("Predictions: %s" % preds)
        cond = WD.get_weather_condition(preds)

        print(cond)
        response.condition = cond
        return response




def main(args=None):
    rclpy.init(args=args)
    wd_service = WeahterDetectionService()
    rclpy.spin(wd_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

