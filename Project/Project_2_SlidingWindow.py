#!/usr/bin/env python
# -*- coding: utf-8 -*-

from pickle import FALSE
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


GRAY_COLOR = (150, 150, 150)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 255, 0)
BLUE_COLOR = (255, 0, 0)
YELLOW_COLOR = (0, 255, 255)
MAGENTA_COLOR = (255, 0, 255)

IMG_BORDER = 150
N_WINDOWS = 15
MARGIN = 60
MIN_PIX = 5


def warp_image(img):
    # Set Points of Source Image 
    top_y_offset = HEIGHT * 0.65
    below_y_offset = HEIGHT * 0.8
    tl_offset = WIDTH * 0.28
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
    warp_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
    warp_inverse_mat = cv2.getPerspectiveTransform(dst_pt, src_pt)

    # Get Translated Image
    dst_img = cv2.warpPerspective(img, warp_mat, (WIDTH, HEIGHT))

    return dst_img, warp_mat, warp_inverse_mat


def binarize_image(img):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
    hls_yellow_binary = cv2.inRange(hls, (20, 145, 100), (70, 255, 255))
    hls_all_binary = cv2.inRange(hls, (0, 145, 0), (255, 255, 255))
    lane = cv2.bitwise_xor(hls_yellow_binary, hls_all_binary)

    return lane


def sliding_window(img):
    histogram = np.sum(img[int((img.shape[0] / 5 * 3)):, :], axis=0)
    midpoint = np.int(histogram.shape[0] / 2)
    left_x_current = np.argmax(histogram[:midpoint])
    right_x_current = np.argmax(histogram[midpoint:]) + midpoint

    window_height = np.int(img.shape[0] / N_WINDOWS)
    nz = img.nonzero()

    left_color_lane_inds, right_color_lane_inds = [], []
    lx, ly, rx, ry = [], [], [], []

    FOUND_COLOR = GREEN_COLOR
    PREDICT_COLOR = YELLOW_COLOR

    sliding_img = np.dstack((img, img, img)) * 255
    sliding_img = cv2.copyMakeBorder(sliding_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT,
                                     cv2.BORDER_CONSTANT)

    prev_left_x = left_x_current
    prev_left_diff = 0
    left_found_flag = True

    prev_right_x = right_x_current
    prev_right_diff = 0
    right_found_flag = True

    for window in range(N_WINDOWS):
        win_yl = img.shape[0] - (window + 1) * window_height
        win_yl_part = win_yl + 0.2 * window_height
        win_yh = img.shape[0] - window * window_height

        win_xll = left_x_current - MARGIN
        win_xlh = left_x_current + MARGIN
        win_xrl = right_x_current - MARGIN
        win_xrh = right_x_current + MARGIN

        left_color_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        right_color_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]
        left_color_lane_inds.append(left_color_inds)
        right_color_lane_inds.append(right_color_inds)

        left_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        right_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]

        if len(left_found_upper_part) > MIN_PIX:
            left_x_current = np.int(np.mean(nz[1][left_found_upper_part]))
            prev_left_diff = left_x_current - prev_left_x
            left_found_flag = True

        else:
            left_x_current = prev_left_x + prev_right_diff
            left_found_flag = False

        if len(right_found_upper_part) > MIN_PIX:
            right_x_current = np.int(np.mean(nz[1][right_found_upper_part]))
            prev_right_diff = right_x_current - prev_right_x
            right_found_flag = True

        else:
            right_x_current = prev_right_x + prev_left_diff
            right_found_flag = False

        lx.append(left_x_current)
        ly.append((win_yl + win_yh) / 2)

        rx.append(right_x_current)
        ry.append((win_yl + win_yh) / 2)

        prev_center_x = (prev_left_x + prev_right_x) / 2
        current_center_x = (left_x_current + right_x_current) / 2

        cv2.line(sliding_img, (int(IMG_BORDER + prev_center_x), int(win_yh)),
                 (int(IMG_BORDER + current_center_x), int(win_yl)), MAGENTA_COLOR, 2)

        prev_left_x = left_x_current
        prev_right_x = right_x_current

        if not left_found_flag:
            win_xll = left_x_current - MARGIN
            win_xlh = left_x_current + MARGIN

        if not right_found_flag:
            win_xrl = right_x_current - MARGIN
            win_xrh = right_x_current + MARGIN

        left_window_color = FOUND_COLOR if left_found_flag else PREDICT_COLOR
        right_window_color = FOUND_COLOR if right_found_flag else PREDICT_COLOR

        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xll), int(win_yl)), (int(IMG_BORDER + win_xlh), int(win_yh)),
                      left_window_color, 2)
        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xrl), int(win_yl)), (int(IMG_BORDER + win_xrh), int(win_yh)),
                      right_window_color, 2)

    left_color_lane_inds = np.concatenate(left_color_lane_inds)
    right_color_lane_inds = np.concatenate(right_color_lane_inds)

    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    img = cv2.copyMakeBorder(img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

    sliding_img = cv2.bitwise_or(sliding_img, img)

    sliding_img[nz[0][left_color_lane_inds], IMG_BORDER + nz[1][left_color_lane_inds]] = BLUE_COLOR
    sliding_img[nz[0][right_color_lane_inds], IMG_BORDER + nz[1][right_color_lane_inds]] = RED_COLOR

    return lx, ly, rx, ry, sliding_img


def warp_process_image(img):
    lane = binarize_image(img)
    cv2.imshow('lane', lane)

    lx, ly, rx, ry, sliding_img = sliding_window(lane)
    cv2.imshow("sliding_img", sliding_img)

    lfit = np.polyfit(np.array(ly), np.array(lx), 2)
    rfit = np.polyfit(np.array(ry), np.array(rx), 2)

    return lfit, rfit


def draw_lane(image, warp_img, Minv, left_fit, right_fit):
    global Width, Height
    # border_y = HEIGHT*(WIDTH+IMG_BORDER)/WIDTH
    border_y = 0

    yMax = warp_img.shape[0]
    ploty = np.linspace(0, yMax - 1, yMax)
    color_warp = np.zeros_like(warp_img).astype(np.uint8)

    left_fitx = left_fit[0] * ploty ** 2 + left_fit[1] * ploty + left_fit[2]
    right_fitx = right_fit[0] * ploty ** 2 + right_fit[1] * ploty + right_fit[2]

    # left_fitx += IMG_BORDER
    # right_fitx += IMG_BORDER

    pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
    pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))])
    pts = np.hstack((pts_left, pts_right))

    color_warp = cv2.copyMakeBorder(color_warp, border_y, border_y, 0, 0, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)
    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    cv2.imshow('color_warp', color_warp)
    newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

    return cv2.addWeighted(image, 1, newwarp, 0.3, 0)


def start():
    global Width, Height
    global image
    global motor

    rospy.init_node('h_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)

    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    while not rospy.is_shutdown():
        warp_img, M, Minv = warp_image(image)
        cv2.imshow("warp_img", warp_img)

        left_fit, right_fit = warp_process_image(warp_img)
        lane_img = draw_lane(image, warp_img, Minv, left_fit, right_fit)

        cv2.imshow('lane_img', lane_img)
        cv2.waitKey(1)


if __name__ == '__main__':
    start()
