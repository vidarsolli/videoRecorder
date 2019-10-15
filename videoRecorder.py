import pyrealsense2 as rs
import cv2
import json
import sys, getopt
from datetime import datetime
import numpy as np
import time
import re
import zmq


#default configuration parameters
recColor = False
recDepth = False
width = 0
height = 1
alignOn = False

# Possible frame rate: 6, 15, 30, 60
frameRate = 30
showImage = True
# Possible resolutions: 320x180, 320x240, 424x240, 640x360, 640x480, 848x480, 960x540, 1280x720, 1920x1080
colorRes = [640, 480]
# Possibe resolutions: 424x240, 480x270, 640x360, 640x400, 640x480, 848x100, 848x480, 1280x720, 1280x800
depthRes = [640, 480]

filenameExt = ""
jsonInFile = ""
recording = False

port = "5556"
context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.setsockopt_string(zmq.SUBSCRIBE, "")
socket.connect("tcp://localhost:%s" % port)


pipeline = rs.pipeline()
config = rs.config()
# Print an explanoritory text
print("videoRecorder records the selected video channels to .wav files")
print("Default parameters can be changed by a json file.")
print("Usage: %s -i json_file" % sys.argv[0])
print("Commands:")
print("'s' start recording")
print("'h' halt recording")
print("'v' toggle image view")
print("'q' to stop and exit program")
print("----------------------------------------------------")

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
    if a == "Align":
        if (recordConfig["Align"] == "Yes"):
            alignOn = True
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

# Create an align object
# rs.align allows us to perform alignment of depth frames to others frames
# The "align_to" is the stream type to which we plan to align depth frames.
if alignOn:
    align_to = rs.stream.color
    align = rs.align(align_to)

ts = time.time()
timestep = re.split('[ .]', str(ts))
t = datetime.now()
filenameExt = str(t.year) + "-" + str(t.month) + "-" + str(t.day) + "-" + str(t.hour) + "-" + str(t.minute) + "-" + str(t.second)

if recColor:
    config.enable_stream(rs.stream.color, colorRes[width], colorRes[height], rs.format.bgr8, frameRate)
if recDepth:
    config.enable_stream(rs.stream.depth, depthRes[width], depthRes[height], rs.format.z16, frameRate)
# Start streaming
pipeline.start(config)
# Define the codec and create VideoWriter object.
if recColor:
    outColor = cv2.VideoWriter("./records/" + "color" + filenameExt + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'),frameRate, (colorRes[width], colorRes[height]))
if recDepth:
    outDepth = cv2.VideoWriter("./records/" + "depth" + filenameExt + ".avi", cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'), frameRate, (depthRes[width], depthRes[height]))

frameCount = 0
recordStartTime = 0
ts = time.time()
key = 0


try:
    key = chr(cv2.waitKey(1) & 0xFF)
    while key != 'q' and key != 'Q':
        # Press esc or 'q' to close the image window
        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        if alignOn:
            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

        if recColor:
            if alignOn:
                color_frame = aligned_frames.get_color_frame()
            else:
                color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            # Convert images to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            if recording:
                outColor.write(color_image)
        if recDepth:
            if alignOn:
                depth_frame = aligned_frames.get_depth_frame()
            else:
                depth_frame = frames.get_depth_frame()
            if not depth_frame:
                continue
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())

            # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
            depth_GrayscaleImage = np.zeros((depthRes[height], depthRes[width], 3), dtype=np.uint8)
            depth_GrayscaleImage[:,:,0]=depth_image/256
            depth_GrayscaleImage[:,:,1]=depth_image%256

            if recording:
                outDepth.write(depth_GrayscaleImage)
        if recording:
            frameCount +=1
            frameRate = frameCount/(time.time()-recordStartTime)
            if showImage:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(color_image, "FrameRate"+str(frameRate), (10, 350), font, 1, (0, 255, 0), 2, cv2.LINE_AA)
                cv2.putText(color_image, 'RECORDING', (10, 450), font, 3, (0, 0, 255), 2, cv2.LINE_AA)
        else:
            if showImage:
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(color_image, 'PAUSE', (10, 450), font, 3, (0, 255, 0), 2, cv2.LINE_AA)


        # Show images
        key = chr(cv2.waitKey(1) & 0xFF)
        try:
            msg = socket.recv_json(flags=zmq.NOBLOCK)
            jsonMsg = json.loads(msg)
            print(jsonMsg)
            for a in jsonMsg:
                print("clien received message, a = ", a)
                if a == "Cmd":
                    key = jsonMsg[a]
        except zmq.ZMQError as e:
            pass

        if showImage:
            # Stack both images horizontally
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_HOT)
            if recDepth and recColor:
                images = np.hstack((color_image, depth_colormap))
            else:
                if recDepth:
                    images = depth_colormap
                else:
                    images = color_image

            cv2.imshow('Video Recorder', images)

        if key == "s":
            print("Recording started")
            recordStartTime = time.time()
            frameCount = 0
            recording = True
        if key == "q":
            print("Recording stopped")
            recording = False
            frameRate = frameCount / (time.time() - recordStartTime)
        if key == "h":
            print("Recording halted")
            recording = False
        if key == "v":
            cv2.namedWindow('Video recorder', cv2.WINDOW_AUTOSIZE)
            if showImage:
                print("Viewing halted")
                showImage = False
            else:
                print("Viewing started")
                showImage = True

finally:
    print("Frame rate: ",frameRate)
    # Stop streaming
    pipeline.stop()

    if recDepth:
        outDepth.release()
    if recColor:
        outColor.release()

    # Closes all the frames
    cv2.destroyAllWindows()

    # Close the ZeroMQ socket
    socket.close()

    sys.exit(0)
