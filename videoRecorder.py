import pyrealsense2 as rs
import cv2
import json
import sys, getopt
from datetime import datetime
import numpy as np
import time
import re



#default configuration parameters
recColor = False
recDepth = False
width = 0
height = 1
frameRate = 30
showImage = True
colorRes = [640, 480]
depthRes = [640, 480]

filenameExt = ""
jsonInFile = ""
recording = False

pipeline = rs.pipeline()
config = rs.config()
# Print an explanoritory text
print("videoRecorder records the selected video channels to .wav files")
print("Default parameters can be changed by a json file.")
print("Usage: %s -i json_file" % sys.argv[0])
print("Hit 's' to start recording, press 'q' to stop and exit program")
print(" ")

# Get the input json file
try:
    myOpts, args = getopt.getopt(sys.argv[1:], "i:")
except getopt.GetoptError as e:
    print(str(e))
    print("Usage: %s -i json_file" % sys.argv[0])
    sys.exit(2)

for o, a in myOpts:
    if o == '-i':
        jsonInFile = a

with open(jsonInFile) as jsonFile:
    recordConfig = json.load(jsonFile)
# Update  configuration parameters
for a in recordConfig:
    if a == "Color":
        if (recordConfig["Color"] == "Yes"):
            recColor = True
    if a == "Depth":
        if (recordConfig["Depth"] == "Yes"):
            recDepth = True
    if a == "ShowImage":
        if (recordConfig["ShowImage"] == "No"):
            showImage = False
    if a == "ColorWidth":
        colorRes[width] = recordConfig["ColorWidth"]
    if a == "ColorHeight":
        colorRes[height] = recordConfig["ColorHeight"]
    if a == "DepthWidth":
        depthRes[width] = recordConfig["DepthWidth"]
    if a == "DepthHeight":
        depthRes[height] = recordConfig["DepthHeight"]
    if a == "FrameRate":
        frameRate = recordConfig["FrameRate"]

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()
ts = time.time()
timestep = re.split('[ .]', str(ts))
t = datetime.now()
filenameExt = str(t.year) + str(t.month) + str(t.day) + str(t.hour) + str(t.minute) + timestep[0]

if recColor:
    config.enable_stream(rs.stream.color, colorRes[width], colorRes[height], rs.format.bgr8, frameRate)
if recDepth:
    config.enable_stream(rs.stream.depth, depthRes[width], depthRes[height], rs.format.z16, frameRate)
#config.enable_record_to_file("Testfile.bag")
# Start streaming
pipeline.start(config)
# Define the codec and create VideoWriter object.The output is stored in 'outpy.avi' file.
if recColor:
    outColor = cv2.VideoWriter("./records/" + "color" + filenameExt + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),frameRate, (colorRes[width], colorRes[height]))
if recDepth:
    outDepth = cv2.VideoWriter("./records/" + "depth" + filenameExt + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),frameRate, (depthRes[width], depthRes[height]))

frameCount = 0
ts = time.time()
cv2.namedWindow('Video recorder', cv2.WINDOW_AUTOSIZE)

try:
    key = chr(cv2.waitKey(1) & 0xFF)

    while key != 'q' and key != 'Q':
        # Press esc or 'q' to close the image window
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        if recColor:
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            if recording:
                outColor.write(color_image)
        if recDepth:
            depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            if recording:
                outDepth.write(depth_colormap)
        if recording:
            frameCount +=1
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(color_image, 'RECORDING', (10, 450), font, 3, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(color_image, 'PAUSE', (10, 450), font, 3, (0, 255, 0), 2, cv2.LINE_AA)

        # Stack both images horizontally
        if recDepth and recColor:
            images = np.hstack((color_image, depth_colormap))
        else:
            if recDepth:
                images = depth_colormap
            else:
                images = color_image

        # Show images
        if showImage:
            cv2.imshow('Video Recorder', images)

        key = chr(cv2.waitKey(1) & 0xFF)
        if key == "s":
            print("Recording started")
            recording = True
        if key == "q":
            print("Recording stopped")
            recording = False
        if key == "h":
            print("Recording halted")
            recording = False

finally:
    print("Frame count: ",frameCount)
    # Stop streaming
    pipeline.stop()

    if recDepth:
        outDepth.release()
    if recColor:
        outColor.release()

    # Closes all the frames
    cv2.destroyAllWindows()