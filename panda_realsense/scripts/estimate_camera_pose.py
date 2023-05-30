#!/usr/bin/env python

import pyrealsense2 as rs
import numpy as np
import cv2 as cv
import math

from subprocess import PIPE, Popen
from scipy.spatial.transform import Rotation


# function to draw in image
def draw(img, corners, imgpts):
    corner = tuple(corners[0].ravel())
    img = cv.line(img, corner, tuple(imgpts[0].ravel()), (0, 0, 255), 2)
    img = cv.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 2)
    img = cv.line(img, corner, tuple(imgpts[2].ravel()), (255, 0, 0), 2)
    return img

# disable scientific notation in output
np.set_printoptions(suppress=True)

# configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()
device_product_line = str(device.get_info(rs.camera_info.product_line))

# set supported resolution
if device_product_line == 'L500':
    config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
else:
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

pipe_profile = pipeline.start(config)

# read color frame
color_frame = None
while not color_frame:
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()

# get intrinsics
intrinsics = color_frame.profile.as_video_stream_profile().intrinsics

# convert intrinsics
mtx = np.float32([[intrinsics.fx, 0.0, intrinsics.ppx],
                  [0.0, intrinsics.fy, intrinsics.ppy],
                  [0.0, 0.0, 1.0]]);
dist = np.float32(intrinsics.coeffs)

# create criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((6 * 7, 3), np.float32)
objp[:, :2] = np.mgrid[0:7, 0:6].T.reshape(-1, 2)
axis = np.float32([[3, 0, 0], [0, -3, 0], [0, 0, -3]]).reshape(-1, 3)

# chessboard square length [m]
UNIT_SIZE = 0.01

T_CS = None
cv.namedWindow('Camera Pose Estimation', cv.WINDOW_AUTOSIZE)
while True:
    # read next color frame
    frames = pipeline.wait_for_frames()
    color_frame = frames.get_color_frame()
    if not color_frame:
        continue

    # convert image to np array
    img = np.asanyarray(color_frame.get_data())
    image_dim = img.shape

    # detect chessboard
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    ret, corners = cv.findChessboardCorners(gray, (7, 6), None)
    if ret:
        # detect corners
        corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        # find the rotation and translation vectors
        ret, rvecs, tvecs = cv.solvePnP(objp, corners2, mtx, dist)

        # project 3D points to image and draw
        imgpts, jac = cv.projectPoints(axis, rvecs, tvecs, mtx, dist)
        img = draw(img, corners2.astype(int), imgpts.astype(int))

        # transformation from color to board
        T_CS = np.r_['0,2', np.c_[cv.Rodrigues(rvecs)[0], tvecs.flatten() * UNIT_SIZE], [0, 0, 0, 1]]

    # show image
    cv.imshow('Camera Pose Estimation', img)

    # press space to capture
    k = cv.waitKey(500)
    if k == ord(' ') and T_CS is not None:
        break;

# transformation from world to board
T_BS =  np.float32(
    [[1, 0, 0, 0.7/2 + (0.115-0.04) - 7*UNIT_SIZE],
    [0, math.cos(math.pi), -math.sin(math.pi), 1.25/2 - 0.365 - 1*UNIT_SIZE],
    [0, math.sin(math.pi), math.cos(math.pi), -0.013],
    [0, 0, 0, 1]]
)

# transformation from board to color
T_SC = np.linalg.inv(T_CS)

# transformation from world to color
T_BC = np.matmul(T_BS, T_SC)

# transformation from color to camera
T_CK = np.float32(
    [[0, -1, 0, 0],
    [0, 0, -1, 0.045],
    [1, 0, 0, -0.008],
    [0, 0, 0, 1]]
)

# transformation from world to camera
T_BK = np.matmul(T_BC, T_CK)

print("camera pose in world frame: ")
print(T_BK)

# save camera pose
T_BK_ZYX = np.r_['0,2,0', T_BK[:3,3], Rotation.from_dcm(T_BK[:3,:3]).as_euler('ZYX', degrees=False)]
np.savetxt("camera_pose.txt", T_BK_ZYX)