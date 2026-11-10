## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 RealSense, Inc. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

def change_thresh(i, isLow):
    if isLow:
        def do_input(x):
            low_thresh[i] = x
    else:
        def do_input(x):
            high_thresh[i] = x
    return do_input

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

low_thresh = np.array([0, 59, 83])
high_thresh = np.array([26, 230, 230])

depth_scale = device.first_depth_sensor().get_depth_scale()

found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("The demo requires Depth camera with Color sensor")
    exit(0)

config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

align = rs.align(rs.stream.color)

# Start streaming
pipeline.start(config)

cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
cv2.namedWindow('Threshold', cv2.WINDOW_AUTOSIZE)
cv2.createTrackbar('B min','Threshold',low_thresh[0],255,change_thresh(0, True))
cv2.createTrackbar('B max','Threshold',high_thresh[0],255,change_thresh(0, False))
cv2.createTrackbar('G min','Threshold',low_thresh[1],255,change_thresh(1, True))
cv2.createTrackbar('G max','Threshold',high_thresh[1],255,change_thresh(1, False))
cv2.createTrackbar('R min','Threshold',low_thresh[2],255,change_thresh(2, True))
cv2.createTrackbar('R max','Threshold',high_thresh[2],255,change_thresh(2, False))

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = align.process(pipeline.wait_for_frames())
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape        
        
        cropped_depth_colormap = depth_colormap[100:300, :, :]
        threshold = cv2.inRange(color_image, low_thresh, high_thresh)

        kernel = np.ones((5,5),np.uint8)
        erosion = cv2.morphologyEx(threshold, cv2.MORPH_ERODE, kernel)
        contours, hierarchy = cv2.findContours(erosion, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = sorted(contours, key=lambda contour: cv2.contourArea(contour), reverse=True)
        cv2.drawContours(color_image, contours, 0, (0,255,0), -1)
        cv2.drawContours(depth_colormap, contours, 0, (0,255,0), 1)

        weights = np.zeros((depth_image.shape[0], depth_image.shape[1], 3), np.uint8)
        if len(contours) > 0 and cv2.contourArea(contours[0]) > 100:
            cv2.drawContours(weights, contours, 0, (0,255,0), -1)
            weights = weights[:, :, 1]
            filtered = depth_image[weights > 0]
            distance = np.median(filtered)
            print(distance * depth_scale)

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.imshow('RealSense', images)
        cv2.imshow('Threshold', threshold)
        cv2.imshow('OpenClose', erosion)
        cv2.imshow('Weights', weights)
        k = cv2.waitKey(1)
        if k == ord('q'):
            break
        if k == ord('w'):
            low_thresh[2] += 1
        if k == ord('s'):
            low_thresh[2] -= 1

        

finally:

    # Stop streaming
    pipeline.stop()