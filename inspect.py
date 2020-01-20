import cv2 as cv
import numpy as np
import sys, getopt
import json
from time import sleep


def mouse_click(event, x, y, flags, param):
    # grab references to the global variables
    global depth_image

    print(depth_image[x, y])
# Get the input json file
json_file = "inspect.json"
try:
    myOpts, args = getopt.getopt(sys.argv[1:], "i:")
except getopt.GetoptError as e:
    print(str(e))
    print("Usage: %s -i <json_file>" % sys.argv[0])
    sys.exit(2)

for o, a in myOpts:
    if o == '-i':
        json_file = a

with open(json_file) as file:
    cp = json.load(file)

# Open the files
color_file = open("/home/vidar/recordings/color"+cp["file_id"]+".npy", "rb")
depth_file = open("/home/vidar/recordings/depth"+cp["file_id"]+".npy", "rb")

recordLen = 10 # number of int64's per record
depth_image_size = cp["width"]*cp["height"]*2 # size of a record in bytes
color_image_size = cp["width"]*cp["height"]*3 # size of a record in bytes
depth_image = np.zeros(cp["width"]*cp["height"], dtype=np.uint16) # a buffer for 1 record
color_image = np.zeros((cp["width"]*cp["height"], 3), dtype=np.uint8) # a buffer for 1 record

def mouse_click(event, x, y, flags, param):
    # grab references to the global variables
    global depth_image
    print("Depth: ", y, x, depth_image[y, x])


# Reading a record recordNo from file into the memArray
#file.seek(recordSize * recordNo)
cbytes = color_file.read(color_image_size)
dbytes = depth_file.read(depth_image_size)
frames = 0
cv.namedWindow('Color', cv.WINDOW_AUTOSIZE)
cv.namedWindow('Depth', cv.WINDOW_AUTOSIZE)
cv.setMouseCallback("Color", mouse_click)

while len(cbytes) == color_image_size:
    color_image = np.frombuffer(cbytes, dtype=np.uint8).copy()
    depth_image = np.frombuffer(dbytes, dtype=np.uint16).copy()
    #cbytes = color_file.read(color_image_size)
    frames += 1
    color_image = np.reshape(color_image, (540, 960, 3))
    depth_image = np.reshape(depth_image, (540, 960))
    depth_colormap = cv.applyColorMap(cv.convertScaleAbs(depth_image, alpha=0.03), cv.COLORMAP_HOT)

    cv.imshow('Color', color_image)
    cv.imshow('Depth', depth_colormap)
    key = cv.waitKey(1) & 0xFF
    if key == ord("q"):
        break
    if key == ord("n"):
        cbytes = color_file.read(color_image_size)
        dbytes = depth_file.read(depth_image_size)

# Note copy() added to make the memArray mutable
print("Frames: ", frames)
color_file.close()
depth_file.close()

# close all open windows
cv.destroyAllWindows()