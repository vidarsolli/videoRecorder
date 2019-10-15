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

#Build the 3D transformation matrix

cap = cv.VideoCapture("./records/vtest.avi")
while cap.isOpened():
    ret, frame = cap.read()
    print("Frame shape:", frame.shape)
    # if frame is read correctly ret is True
    if not ret:
        print("Can't receive frame (stream end?). Exiting ...")
        break
    cv.imshow('original', frame)
    #aligned_frame = frame.copy()
    aligned_frame = np.full(frame.shape, 255, dtype = "uint8")
    diff = frame.copy()
    vIn = np.zeros(4)
    vOut = np.zeros(4)
    for x in range(frame.shape[0]):
        for y in range(frame.shape[1]):
            vIn = [x, y, (frame[x][y][1]+256*frame[x][y][0]), 1]
            vOut= np.matmul(M, vIn)
            #print("Vout shape", vOut.shape)
            #print(vIn, vOut)
            newX = int(np.minimum(frame.shape[0]-1, int(vOut[0])))
            newY = int(np.minimum(frame.shape[1]-1, int(vOut[1])))
            #print(newX, newY)
            aligned_frame[newX][newY][0] =  frame[x][y][0]*10
            aligned_frame[newX][newY][1] = frame[x][y][1]
            aligned_frame[newX][newY][2] = frame[x][y][2]
            if abs(aligned_frame[newX][newY][0] -frame[x][y][0]) > 0:
                diff[x][y][0] = 255
            if abs(aligned_frame[newX][newY][1] -frame[x][y][1]) > 0:
                diff[x][y][1] = 255
            if abs(aligned_frame[newX][newY][2] -frame[x][y][2]) > 0:
                diff[x][y][2] = 255
            #aligned_frame[x][y][0] = frame[x][y][0]
            #aligned_frame[x][y][1] = frame[x][y][1]
            #aligned_frame[x][y][2] = frame[x][y][2]
    #gray = cv.cvtColor(aligned_frame, cv.COLOR_BGR2GRAY)
    cv.imshow('translated', aligned_frame)
    cv.imshow('diff', diff)
    if cv.waitKey(1) == ord('q'):
        break
cap.release()
cv.destroyAllWindows()