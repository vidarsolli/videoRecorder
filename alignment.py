import pyrealsense2 as rs
import cv2 as cv
import json
import sys, getopt
from datetime import datetime
import numpy as np

rotation = np.array([0.99998, -0.00585515, -0.00218272, 0.00584836, 0.999978, -0.00310369, 0.00220084, 0.00309087, 0.999993])
rotation = rotation.reshape(3,3)
translation = np.array([0.0147237, 8.24167e-05, 0.000220023])
M = np.array([[0.99998, -0.00585515, -0.00218272, 0.0147237],
              [0.00584836, 0.999978, -0.00310369, 8.24167e-05],
              [0.00220084, 0.00309087, 0.999993, 0.000220023],
              [0, 0, 0, 1.0]])
print(M)
print ("M.shape: ", M.shape)

# Retreive transformation data
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 15)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 15)

pipe_profile = pipeline.start(config)
frames = pipeline.wait_for_frames()
depth_frame = frames.get_depth_frame()
color_frame = frames.get_color_frame()

# Intrinsics & Extrinsics
depth_intrin = depth_frame.profile.as_video_stream_profile().intrinsics
color_intrin = color_frame.profile.as_video_stream_profile().intrinsics
depth_to_color_extrin = depth_frame.profile.get_extrinsics_to(color_frame.profile)
color_to_depth_extrin = color_frame.profile.get_extrinsics_to(depth_frame.profile)

print("\n Depth intrinsics: " + str(depth_intrin))
print("\n Color intrinsics: " + str(color_intrin))
print("\n Depth to color extrinsics: " + str(depth_to_color_extrin))
print("\n Color_to_depth_extrinsics: " + str(color_to_depth_extrin))

# Depth scale - units of the values inside a depth frame, i.e how to convert the value to units of 1 meter
depth_sensor = pipe_profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print("Depth scale: ", depth_scale)
# Map depth to color
depth_pixel = [200, 200]   # Random pixel
depth_image = np.asanyarray(depth_frame.get_data())
print("Depth image shape: ", depth_image.shape)
depth = depth_image[200][200]*depth_scale
print("Depth: ", depth)
print("Depth_pixel: " + str(depth_pixel))
depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, depth_pixel, depth)
print("Depth point:", depth_point)
color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
print("Color point:", color_point)
color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
print("Color_pixel: " + str(color_pixel))
pipeline.stop()

cap = cv.VideoCapture("./records/vtest.avi")
while cap.isOpened():
    ret, frame = cap.read()
    print("Frame shape:", frame.shape)
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    cv.imshow('original', frame)
    aligned_frame = np.zeros((480, 640, 3), dtype="uint8")
    for x in range(frame.shape[0]):
        for y in range(frame.shape[1]):
            #print("Aligned frame shape:", aligned_frame.shape)
            depth = (frame[x][y][0]*256+frame[x][y][1])* depth_scale
            #print("Depth_pixel: " + str(depth_pixel))
            depth_point = rs.rs2_deproject_pixel_to_point(depth_intrin, [x,y], depth)
            #print("Depth point:", depth_point)
            color_point = rs.rs2_transform_point_to_point(depth_to_color_extrin, depth_point)
            #print("Color point:", color_point)
            color_pixel = rs.rs2_project_point_to_pixel(color_intrin, color_point)
            #print("Color_pixel: " + str(color_pixel))
            newX = int(np.minimum(frame.shape[0]-1, int(color_pixel[0])))
            newY = int(np.minimum(frame.shape[1]-1, int(color_pixel[1])))
            newX = int(np.maximum(0, newX))
            newY = int(np.maximum(0, newY))
            #print(newX, newY)
            aligned_frame[newX][newY][0] = int((frame[x][y][0]*256+frame[x][y][1])/32)
            aligned_frame[newX][newY][1] = aligned_frame[newX][newY][0] #frame[x][y][1]
            aligned_frame[newX][newY][2] = aligned_frame[newX][newY][0] #frame[x][y][2]
            print(x, y, newX, newY)
    cv.imshow('translated', aligned_frame)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()