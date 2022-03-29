#!/usr/bin/env python
# -*- coding: utf-8 -*-

import numpy as np
import cv2, math
import rospy, rospkg, time
from sensor_msgs.msg import Image, Imu
from xycar_msgs.msg import xycar_motor
from cv_bridge import CvBridge
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

GRAY_COLOR = (150, 150, 150)
RED_COLOR = (0, 0, 255)
GREEN_COLOR = (0, 255, 0)
BLUE_COLOR = (255, 0, 0)
YELLOW_COLOR = (0, 255, 255)
MAGENTA_COLOR = (255, 0, 255)
CYAN_COLOR = (255, 255, 0)

IMG_BORDER = 150
N_WINDOWS = 15
MARGIN = 60

MAX_ANGLE = 3
CAR_PIXEL = 73
METER_PER_PIXEL = 0.055

prev_hist_left_x, prev_hist_right_x = 0, 0
CENTER_X1_IDX, CENTER_X2_IDX = 2, 3
x_left_list = np.full(N_WINDOWS, 0)
x_right_list = np.full(N_WINDOWS, WIDTH)

imu_callback_time = None

velX, velY, veclocity, prev_vel = 0, 0, 0, 0


class MovingAverage:
    def __init__(self, n):
        self.samples = n
        self.data = []
        self.weights = list(range(1, n + 1))

    def add_sample(self, new_sample):
        if len(self.data) < self.samples:
            self.data.append(new_sample)
        else:
            self.data = self.data[1:] + [new_sample]

    def get_mm(self):
        return float(sum(self.data)) / len(self.data)

    def get_wmm(self):
        s = 0
        for i, x in enumerate(self.data):
            s += x * self.weights[i]
        return float(s) / sum(self.weights[:len(self.data)])


MV_AVG_SIZE = 20
current_left_x_mv = MovingAverage(MV_AVG_SIZE)
current_right_x_mv = MovingAverage(MV_AVG_SIZE)
# current_left_x_mv.add_sample(WIDTH * 0.1)
# current_right_x_mv.add_sample(WIDTH * 0.9)


# camera image topic callback
def img_callback(data):
    global image
    image = bridge.imgmsg_to_cv2(data, "bgr8")


def imu_callback(data):
    global imu_callback_time, velX, velY, veclocity, prev_vel

    if imu_callback_time is None:
        dt = 0
        imu_callback_time = time.time()
        return
    else:
        dt = time.time() - imu_callback_time

    velX += (data.linear_acceleration.x * dt)
    velY += (data.linear_acceleration.y * dt)

    veclocity = math.sqrt(math.pow(velX, 2) + math.pow(velY, 2))

    if veclocity == prev_vel:
        velX, velY = 0, 0
        veclocity = 0

    prev_vel = veclocity
    imu_callback_time = time.time()


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

    return dst_img, warp_inverse_mat


def binarize_image(img):
    blur = cv2.GaussianBlur(img, (5, 5), 0)
    hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
    hls_yellow_binary = cv2.inRange(hls, (20, 145, 100), (70, 255, 255))
    hls_all_binary = cv2.inRange(hls, (0, 145, 0), (255, 255, 255))
    lane = cv2.bitwise_xor(hls_yellow_binary, hls_all_binary)

    return lane


def draw_straight_line(x1, x2, y1, y2, sliding_img):
    under = x1 - x2
    if under == 0:
        under = 0.001

    grad = float(y1 - y2) / under

    x_below = float(HEIGHT - y2) / grad + x2
    x_upper = float(0 - y2) / grad + x2

    angle = math.degrees(math.atan(grad))
    angle = np.sign(angle) * -90 + angle

    control_grad = float(y1 - y2 + CAR_PIXEL) / under
    control_angle = math.degrees(math.atan(control_grad))
    control_angle = np.sign(control_angle) * -90 + control_angle

    cv2.line(sliding_img, (int(IMG_BORDER + x_below), HEIGHT), (int(IMG_BORDER + x_upper), 0), CYAN_COLOR, 2)

    return sliding_img, control_angle


def sliding_window(img):
    global CENTER_X1_IDX, CENTER_X2_IDX, x_left_list, x_right_list
    global current_left_x_mv, current_right_x_mv

    histogram = np.sum(img[int((HEIGHT / 5 * 3)):, :], axis=0)
    midpoint = int(WIDTH / 2)

    center_target_x = (x_left_list[CENTER_X1_IDX] + x_right_list[CENTER_X1_IDX]) // 2
    # print('center_target_x', center_target_x)
    # HIST_MARGIN = 10
    # center_target_x = int(min(center_target_x, (WIDTH / 2) - 1 - HIST_MARGIN))
    # center_target_x = int(max(center_target_x, HIST_MARGIN))

    current_left_x_mv.add_sample(np.argmax(histogram[:midpoint]))
    current_right_x_mv.add_sample(np.argmax(histogram[midpoint:]) + midpoint)

    current_left_x = current_left_x_mv.get_wmm()
    current_right_x = current_right_x_mv.get_wmm()

    window_height = np.int(HEIGHT / N_WINDOWS)
    nz = img.nonzero()

    left_color_lane_inds, right_color_lane_inds = [], []
    lx, ly, rx, ry = [], [], [], []

    found_color = GREEN_COLOR
    predict_color = YELLOW_COLOR

    sliding_img = np.dstack((img, img, img)) * 255
    sliding_img = cv2.copyMakeBorder(sliding_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT,
                                     cv2.BORDER_CONSTANT)

    prev_left_x = current_left_x
    prev_left_diff = 0
    left_found_flag = []

    prev_right_x = current_right_x
    prev_right_diff = 0
    right_found_flag = []

    total_left_found_flag = False
    total_right_found_flag = False

    x_left_list, x_right_list = [], []
    MIN_PIX = window_height * MARGIN * 0.2 * 0.3

    for window_idx in range(N_WINDOWS):
        win_yl = HEIGHT - (window_idx + 1) * window_height
        win_yl_part = win_yl + 0.2 * window_height
        win_yh = HEIGHT - window_idx * window_height

        win_xll = current_left_x - MARGIN
        win_xlh = current_left_x + MARGIN
        win_xrl = current_right_x - MARGIN
        win_xrh = current_right_x + MARGIN

        left_color_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        right_color_inds = ((nz[0] >= win_yl) & (nz[0] < win_yh) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]
        left_color_lane_inds.append(left_color_inds)
        right_color_lane_inds.append(right_color_inds)

        left_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
        right_found_upper_part = \
        ((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]

        if len(left_found_upper_part) > MIN_PIX:
            current_left_x = np.int(np.mean(nz[1][left_found_upper_part]))
            prev_left_diff = current_left_x - prev_left_x
            left_found_flag.append(True)
            total_left_found_flag = True

        else:
            current_left_x = prev_left_x + prev_right_diff
            if window_idx == 0:
                current_left_x = current_right_x - WIDTH * 0.7
            left_found_flag.append(False)

        if len(right_found_upper_part) > MIN_PIX:
            current_right_x = np.int(np.mean(nz[1][right_found_upper_part]))
            prev_right_diff = current_right_x - prev_right_x
            right_found_flag.append(True)
            total_right_found_flag = True

        else:
            current_right_x = prev_right_x + prev_left_diff
            if window_idx == 0:
                current_right_x = current_left_x + WIDTH * 0.7
            right_found_flag.append(False)

        if window_idx == 0:
            current_left_x_mv.add_sample(current_left_x) 
            current_right_x_mv.add_sample(current_right_x) 

        lx.append(current_left_x)
        ly.append((win_yl + win_yh) / 2)

        rx.append(current_right_x)
        ry.append((win_yl + win_yh) / 2)

        x_left_list.append(prev_left_x)
        x_right_list.append(prev_right_x)

        prev_left_x = current_left_x
        prev_right_x = current_right_x

    left_color_lane_inds = np.concatenate(left_color_lane_inds)
    right_color_lane_inds = np.concatenate(right_color_lane_inds)

    if not total_left_found_flag:
        dx = x_right_list[N_WINDOWS - 1] - x_right_list[0]

        if dx > 0:
            x_left_list = []
            print('right to left')
            for idx in range(N_WINDOWS):
                x_left_list.append(x_right_list[idx])
                x_right_list[idx] = x_left_list[idx] + (WIDTH * 0.8)

    if not total_right_found_flag:
        dx = x_left_list[N_WINDOWS - 1] - x_left_list[0]
        
        if dx < 0:
            x_right_list = []
            # print(340)
            for idx in range(N_WINDOWS):
                x_right_list.append(x_left_list[idx])
                x_left_list[idx] = x_right_list[idx] - (WIDTH * 0.8)
                # print(344, x_right_list[idx])

    # print('x_left_list', x_left_list)
    # print('x_right_list', x_right_list)
    # print('total_left_found_flag', total_left_found_flag)
    # print('total_right_found_flag', total_right_found_flag)

    # print('x_left_list', x_left_list)
    # print('x_right_list', x_right_list)

    for window_idx in range(N_WINDOWS):
        win_xll = x_left_list[window_idx] - MARGIN
        win_xlh = x_left_list[window_idx] + MARGIN
        win_xrl = x_right_list[window_idx] - MARGIN
        win_xrh = x_right_list[window_idx] + MARGIN

        win_yl = HEIGHT - (window_idx + 1) * window_height
        win_yh = HEIGHT - window_idx * window_height

        # if not left_found_flag[window_idx]:
        #     win_xll = x_left_list[window_idx] - MARGIN
        #     win_xlh = x_left_list[window_idx] + MARGIN

        # if not right_found_flag[window_idx]:
        #     win_xrl = x_right_list[window_idx] - MARGIN
        #     win_xrh = x_right_list[window_idx] + MARGIN

        left_window_color = found_color if left_found_flag[window_idx] else predict_color
        right_window_color = found_color if right_found_flag[window_idx] else predict_color

        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xll), int(win_yl)), (int(IMG_BORDER + win_xlh), int(win_yh)),
                      left_window_color, 2)
        cv2.rectangle(sliding_img, (int(IMG_BORDER + win_xrl), int(win_yl)), (int(IMG_BORDER + win_xrh), int(win_yh)),
                      right_window_color, 2)

        if window_idx > 0:
            prev_center_x = (x_left_list[window_idx - 1] + x_right_list[window_idx - 1]) / 2
            current_center_x = (x_left_list[window_idx] + x_right_list[window_idx]) / 2

            cv2.line(sliding_img, (int(IMG_BORDER + prev_center_x), int(win_yh)),
                    (int(IMG_BORDER + current_center_x), int(win_yl)), MAGENTA_COLOR, 2)

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
    # cv2.imshow("new_sliding_img", sliding_img)

    lfit = np.polyfit(np.array(ly), np.array(lx), 2)
    rfit = np.polyfit(np.array(ry), np.array(rx), 2)

    return lfit, rfit, sliding_img


def draw_lane(image, warp_img, warp_inverse_mat, left_fit, right_fit):
    global Width, Height

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

    color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
    new_warp = cv2.warpPerspective(color_warp, warp_inverse_mat, (Width, Height))

    return cv2.addWeighted(image, 1, new_warp, 0.3, 0)


# #########################################################
# #################    Previous Code    ###################
# #########################################################

# prev_warp_img_w = 320
# prev_warp_img_h = 240

# prev_warpx_margin = 20
# prev_warpy_margin = 3


# prev_nwindows = 9
# prev_margin = 12
# prev_minpix = 5

# ######################
# '''
# prev_nwindows = 15
# prev_margin = 60
# prev_minpix = 5
# '''
# ######################

# prev_lane_bin_th = 145

# def prev_warp_process_image(img):
#     global prev_nwindows
#     global prev_margin
#     global prev_minpix
#     global prev_lane_bin_th

#     CENTER_X1_IDX, CENTER_X2_IDX = 1, 2
#     center_x1, center_x2 = 0, 0
#     center_y1, center_y2 = 0, 0

#     blur = cv2.GaussianBlur(img,(5, 5), 0)
#     _, L, _ = cv2.split(cv2.cvtColor(blur, cv2.COLOR_BGR2HLS))
#     _, lane = cv2.threshold(L, prev_lane_bin_th, 255, cv2.THRESH_BINARY)

#     histogram = np.sum(lane[lane.shape[0]//2:,:], axis=0)      
#     midpoint = np.int(histogram.shape[0]/2)
#     leftx_current = np.argmax(histogram[:midpoint])
#     rightx_current = np.argmax(histogram[midpoint:]) + midpoint

#     window_height = np.int(lane.shape[0]/prev_nwindows)
#     nz = lane.nonzero()

#     left_lane_inds = []
#     right_lane_inds = []
    
#     lx, ly, rx, ry = [], [], [], []

#     out_img = np.dstack((lane, lane, lane))*255
#     out_img = cv2.copyMakeBorder(out_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT,
#                                      cv2.BORDER_CONSTANT)

#     prev_left_x = leftx_current
#     prev_right_x = rightx_current

#     for window in range(prev_nwindows):
#         win_yl = lane.shape[0] - (window+1)*window_height
#         win_yh = lane.shape[0] - window*window_height

#         win_xll = leftx_current - prev_margin
#         win_xlh = leftx_current + prev_margin
#         win_xrl = rightx_current - prev_margin
#         win_xrh = rightx_current + prev_margin

#         cv2.rectangle(out_img, (int(IMG_BORDER + win_xll), int(win_yl)), (int(IMG_BORDER + win_xlh), int(win_yh)),
#                       (0,255,0), 2) 
#         cv2.rectangle(out_img, (int(IMG_BORDER + win_xrl), int(win_yl)), (int(IMG_BORDER + win_xrh), int(win_yh)),
#                       (0,255,0), 2) 

#         good_left_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xll)&(nz[1] < win_xlh)).nonzero()[0]
#         good_right_inds = ((nz[0] >= win_yl)&(nz[0] < win_yh)&(nz[1] >= win_xrl)&(nz[1] < win_xrh)).nonzero()[0]

#         left_lane_inds.append(good_left_inds)
#         right_lane_inds.append(good_right_inds)

#         if len(good_left_inds) > prev_minpix:
#             leftx_current = np.int(np.mean(nz[1][good_left_inds]))
#         if len(good_right_inds) > prev_minpix:        
#             rightx_current = np.int(np.mean(nz[1][good_right_inds]))

#         lx.append(leftx_current)
#         ly.append((win_yl + win_yh)/2)

#         rx.append(rightx_current)
#         ry.append((win_yl + win_yh)/2)

#         prev_center_x = (prev_left_x + prev_right_x) / 2
#         current_center_x = (leftx_current + rightx_current) / 2

#         if window == CENTER_X1_IDX:
#             center_x1 = current_center_x
#             center_y1 = win_yh

#         elif window == CENTER_X2_IDX:
#             center_x2 = current_center_x
#             center_y2 = win_yh


#         cv2.line(out_img, (int(IMG_BORDER + prev_center_x), int(win_yh)),
#                  (int(IMG_BORDER + current_center_x), int(win_yl)), MAGENTA_COLOR, 2)

#         prev_left_x = leftx_current
#         prev_right_x = rightx_current

#     left_lane_inds = np.concatenate(left_lane_inds)
#     right_lane_inds = np.concatenate(right_lane_inds)

#     out_img, angle = draw_straight_line(center_x1, center_x2, center_y1, center_y2, out_img)

#     #left_fit = np.polyfit(nz[0][left_lane_inds], nz[1][left_lane_inds], 2)
#     #right_fit = np.polyfit(nz[0][right_lane_inds] , nz[1][right_lane_inds], 2)
    
#     lfit = np.polyfit(np.array(ly),np.array(lx),2)
#     rfit = np.polyfit(np.array(ry),np.array(rx),2)

#     lane = cv2.cvtColor(lane, cv2.COLOR_GRAY2BGR)
#     lane = cv2.copyMakeBorder(lane, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

#     out_img = cv2.bitwise_or(out_img, lane)

#     out_img[nz[0][left_lane_inds], IMG_BORDER + nz[1][left_lane_inds]] = BLUE_COLOR
#     out_img[nz[0][right_lane_inds], IMG_BORDER + nz[1][right_lane_inds]] = RED_COLOR

#     cv2.putText(out_img, 'angle: ' + str(angle)[:5], (int(IMG_BORDER*2 + WIDTH * 0.8), 
#                 int(HEIGHT * 0.1)), 1, 1, GRAY_COLOR, 1)

#     cv2.imshow("prev_sliding_img", out_img)
    
#     return lfit, rfit, angle

# def prev_draw_lane(image, warp_img, Minv, left_fit, right_fit):
#     global Width, Height
#     yMax = warp_img.shape[0]
#     ploty = np.linspace(0, yMax - 1, yMax)
#     color_warp = np.zeros_like(warp_img).astype(np.uint8)
    
#     left_fitx = left_fit[0]*ploty**2 + left_fit[1]*ploty + left_fit[2]
#     right_fitx = right_fit[0]*ploty**2 + right_fit[1]*ploty + right_fit[2]
    
#     pts_left = np.array([np.transpose(np.vstack([left_fitx, ploty]))])
#     pts_right = np.array([np.flipud(np.transpose(np.vstack([right_fitx, ploty])))]) 
#     pts = np.hstack((pts_left, pts_right))
    
#     color_warp = cv2.fillPoly(color_warp, np.int_([pts]), (0, 255, 0))
#     newwarp = cv2.warpPerspective(color_warp, Minv, (Width, Height))

#     return cv2.addWeighted(image, 1, newwarp, 0.3, 0)

# #########################################################

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)


def get_drive_angle(dist, img):
    global METER_PER_PIXEL, CENTER_X1_IDX, x_left_list, x_right_list

    translate_dist = dist * (1 / METER_PER_PIXEL)
    window_height = HEIGHT / N_WINDOWS

    target_y = HEIGHT - (CENTER_X1_IDX * window_height)
    target_y -= translate_dist
    cv2.line(img, (0, int(target_y)), (int(IMG_BORDER * 2 + WIDTH), int(target_y)), GRAY_COLOR, 2)

    y_below_idx = int((float(HEIGHT - target_y) / HEIGHT) * N_WINDOWS)

    if y_below_idx < 0:
        y_below_idx = 0
    elif y_below_idx >= N_WINDOWS - 1:
        y_below_idx = N_WINDOWS - 2
        
    # print('y_below_idx', y_below_idx)
    # print('center_x_list[y_below_idx]', center_x_list[y_below_idx])
    
    center_x_below = (x_left_list[y_below_idx] + x_right_list[y_below_idx]) / 2
    center_x_upper = (x_left_list[y_below_idx + 1] + x_right_list[y_below_idx + 1]) / 2

    img, drive_angle = draw_straight_line(center_x_below, center_x_upper,
                HEIGHT - (y_below_idx * window_height), HEIGHT - ((y_below_idx + 1) * window_height), img)

    cv2.putText(img, 'angle: ' + str(drive_angle)[:5], (int(IMG_BORDER*2 + WIDTH * 0.7), 
                int(HEIGHT * 0.1)), 1, 1, GRAY_COLOR, 1)
    cv2.imshow('new_sliding_window', img)

    return drive_angle


def start():
    global Width, Height
    global image, motor

    rospy.init_node('h_drive')

    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    motor_msg = xycar_motor()
    image_sub = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
    imu_sub = rospy.Subscriber('imu', Imu, imu_callback)

    while not image.size == (WIDTH * HEIGHT * 3):
        continue

    prev_time = time.time()
    while not rospy.is_shutdown():
        
        warp_img, warp_inverse_mat = warp_image(image)
        left_fit, right_fit, sliding_img = warp_process_image(warp_img)
        new_lane_img = draw_lane(image, warp_img, warp_inverse_mat, left_fit, right_fit)

        # #########################################################
        # #################    Previous Code    ###################
        # #########################################################

        # prev_left_fit, prev_right_fit, prev_angle = prev_warp_process_image(warp_img)
        # prev_lane_img = draw_lane(image, warp_img, warp_inverse_mat, prev_left_fit, prev_right_fit)

        # #########################################################

        cv2.imshow('new_lane_img', new_lane_img)
        # cv2.imshow('prev_lane_img', prev_lane_img)
        cv2.waitKey(1)

        ds = veclocity * (time.time()-prev_time)
        drive_angle = get_drive_angle(ds, sliding_img)

        drive(drive_angle, 5)

        prev_time = time.time()


if __name__ == '__main__':
    start()
