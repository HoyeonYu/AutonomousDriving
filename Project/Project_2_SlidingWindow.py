#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

Width = 640
Height = 480

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None

CAM_FPS = 30
WIDTH, HEIGHT = 640, 480
ROI_ROW = 250  # ROI row 
ROI_HEIGHT = HEIGHT - ROI_ROW  
L_ROW = ROI_HEIGHT - 120  # Row for position detection

# camera image topic callback
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")

warp_img_w = 320
warp_img_h = 240

warpx_margin = 20
warpy_margin = 3

nwindows = 15
margin = 60
minpix = 5

lane_bin_th = 145

warp_src  = np.array([
    [230-warpx_margin, 300-warpy_margin],  
    [45-warpx_margin,  450+warpy_margin],
    [445+warpx_margin, 300-warpy_margin],
    [610+warpx_margin, 450+warpy_margin]
], dtype=np.float32)

warp_dist = np.array([
    [0,0],
    [0,warp_img_h],
    [warp_img_w,0],
    [warp_img_w, warp_img_h]
], dtype=np.float32)

GRAY_COLOR = (150, 150, 150)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 255, 0)
BLUE_COLOR = (255, 0, 0)
YELLOW_COLOR = (0, 255, 255)

calibrated = True

if calibrated:
    mtx = np.array([
        [422.037858, 0.0, 245.895397], 
        [0.0, 435.589734, 163.625535], 
        [0.0, 0.0, 1.0]
    ])
    dist = np.array([-0.289296, 0.061035, 0.001786, 0.015238, 0.0])
    cal_mtx, cal_roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (Width, Height), 1, (Width, Height))

def calibrate_image(frame):
    global Width, Height
    global mtx, dist
    global cal_mtx, cal_roi
    
    tf_image = cv2.undistort(frame, mtx, dist, None, cal_mtx)
    x, y, w, h = cal_roi
    tf_image = tf_image[y:y+h, x:x+w]

    return cv2.resize(tf_image, (Width, Height))

def warp_image(img, src, dst, size):
    # Set Points of Source Image 
    top_y_offset = HEIGHT * 0.65
    below_y_offset = HEIGHT * 0.8
    tl_offset = WIDTH * 0.27
    tr_offset = WIDTH - tl_offset
    bl_offset = WIDTH * 0
    br_offset = WIDTH - bl_offset

    src_tl = [tl_offset, top_y_offset]
    src_tr = [tr_offset, top_y_offset]
    src_bl = [bl_offset, below_y_offset]
    src_br = [br_offset, below_y_offset]

    src_pt = np.float32([src_tl, src_tr, src_bl, src_br])

    # Draw Lines of Perspective Area
    point_img = img.copy()

    tup_src_tl = tuple(map(int, src_tl))
    tup_src_tr = tuple(map(int, src_tr))
    tup_src_bl = tuple(map(int, src_bl))
    tup_src_br = tuple(map(int, src_br))

    cv2.line(point_img, tup_src_tl, tup_src_tr, RED_COLOR, 2)
    cv2.line(point_img, tup_src_tr, tup_src_br, RED_COLOR, 2)
    cv2.line(point_img, tup_src_br, tup_src_bl, RED_COLOR, 2)
    cv2.line(point_img, tup_src_bl, tup_src_tl, RED_COLOR, 2)

    cv2.imshow('point_img', point_img)

    # Set Points of Destination Image
    dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])

    # Get Perspective Transform Matrix
    M = cv2.getPerspectiveTransform(src_pt, dst_pt)
    Minv = cv2.getPerspectiveTransform(dst_pt, src_pt)

    # Get Translated Image
    dst_img = cv2.warpPerspective(img, M, (WIDTH, HEIGHT))

    return dst_img, M, Minv

def warp_process_image(img):
    global nwindows
    global margin
    global minpix
    global lane_bin_th

    blur = cv2.GaussianBlur(img,(5, 5), 0)
    _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
    _, lane = cv2.threshold(L, lane_bin_th, 255, cv2.THRESH_BINARY)
    cv2.imshow('threshed lane', lane)

    histogram = np.sum(lane[lane.shape[0]//4 * 3:,:], axis=0)    
    # print('histogram', histogram) 
    # Change? : midPoint of Width: shape[1] / 2 
    # midpoint = np.int(histogram.shape[1]/2)
    midpoint = np.int(histogram.shape[0]/2)
    leftx_current = np.argmax(histogram[:midpoint])
    rightx_current = np.argmax(histogram[midpoint:]) + midpoint

    window_height = np.int(lane.shape[0]/nwindows)
    nz = lane.nonzero()
    # print('nz: ', nz)

    left_lane_inds = []
    right_lane_inds = []
    
    lx, ly, rx, ry = [], [], [], []

    out_img = np.dstack((lane, lane, lane))*255

    prev_left_inds = np.int(0)
    prev_right_inds = np.int(0)

    for window in range(nwindows):
        win_yl = int(lane.shape[0] - (window+1)*window_height)
        win_yh = int(lane.shape[0] - window*window_height)

        win_xll = int(leftx_current - margin)
        win_xlh = int(leftx_current + margin)
        win_xrl = int(rightx_current - margin)
        win_xrh = int(rightx_current + margin)

        cv2.rectangle(out_img,(win_xll,win_yl),(win_xlh,win_yh),(0,255,0), 2) 
        cv2.rectangle(out_img,(win_xrl,win_yl),(win_xrh,win_yh),(0,255,0), 2) 
        # print('win_xll', win_xll, 'win_yl', win_yl, 'win_xlh', win_xlh,
        #         'win_yh', win_yh)

        good_left_inds = np.array(((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero())
        print(good_left_inds)
        good_right_inds = np.array(((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero())
        print(good_right_inds)

        good_left_x = prev_left_inds
        good_right_x = prev_right_inds

        if good_left_inds.size > 0:
            good_left_x = np.int(np.mean(nz[1][good_left_inds]))

        if good_right_inds.size > 0:
            good_right_x = np.int(np.mean(nz[1][good_right_inds]))

        print('good_left_x', good_left_x)
        print('good_right_x', good_right_x)

        left_lane_inds.append(good_left_x)
        right_lane_inds.append(good_right_x)
        # print('good_left_inds', len(good_left_inds))
        # print('right_lane_inds', len(right_lane_inds))
        # print('good_left_inds', (good_left_inds))

        lx.append(good_left_x)
        ly.append((win_yl + win_yh)/2)

        rx.append(good_right_x)
        ry.append((win_yl + win_yh)/2)

    left_lane_inds = np.concatenate(left_lane_inds)
    right_lane_inds = np.concatenate(right_lane_inds)


    #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
    #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
    
    lfit = np.polyfit(np.array(ly),np.array(lx),2)
    rfit = np.polyfit(np.array(ry),np.array(rx),2)

    lane = cv2.cvtColor(lane, cv2.COLOR_GRAY2BGR)
    out_img = cv2.bitwise_or(out_img, lane)
    out_img[nz[0][left_lane_inds], nz[1][left_lane_inds]] = [255, 0, 0]
    out_img[nz[0][right_lane_inds] , nz[1][right_lane_inds]] = [0, 0, 255]
    cv2.imshow("viewer", out_img)
    
    #return left_fit, right_fit
    return lfit, rfit

def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global Width, Height
    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
    left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
    right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
    pts = np.hstack((pts_left, pts_right))
    
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

def start():
    global Width, Height, cap

    global image
    global motor
    prev_x_left, prev_x_right = 0, WIDTH

    rospy.init_node('h_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    while not image.size == (WIDTH * HEIGHT * 3):
        continue
 
    while not rospy.is_shutdown():
        # image = calibrate_image(image)
        warp_img, M, Minv = warp_image(image, warp_src, warp_dist, (warp_img_w, warp_img_h))
        cv2.imshow("warp_img", warp_img)

        left_fit, right_fit = warp_process_image(warp_img)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)

        cv2.imshow('lane_img', lane_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    start()


