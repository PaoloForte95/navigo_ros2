#!/usr/bin/env python3

import os
import sys
import cv2


def test_weather_detection():
    """Test for detect weather from an image."""
    module_path = os.getcwd() 
    sys.path.insert(0,  module_path)
    from src.detector.weather_detection import WeatherDetection
    ### Python based example
    WD = WeatherDetection(module_path + "/src/detector/image_based_weather_detection.onnx")
    img = cv2.imread(module_path + "/test/images/im1.jpg")
    pimg = WD.preprocess_image(img)
    pred = WD.get_inference(pimg)
    print(pred)
    return pred

test_weather_detection()