#!/usr/bin/env python
# -*- coding: utf-8 -*-

from turtle import left
from matplotlib.pyplot import draw
import rospy, time
import numpy as np
import cv2, math

# from sensor_msgs.msg import CompressedImage
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from xycar_msgs.msg import xycar_motor
from math import *
import signal
import sys
import os

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

def signal_handler(sig, frame):
    import time
    time.sleep(3)
    os.system('killall -9 python rosout')
    sys.exit(0)

signal.signal(signal.SIGINT, signal_handler)

image = np.empty(shape=[0])
bridge = CvBridge()
motor = None
WIDTH = 320
HEIGHT = 240
RESIZE = 4

HLS_THRESH_VAL = 90
BRIGHTNESS_VAL = 10

cam = False
cam_debug = True

sub_f = 0
time_c = 0

GAP = 40

line_below_left = 0
line_upper_left = 0
line_below_right = WIDTH
line_upper_right = WIDTH

MV_AVG_SIZE = 50

line_below_left_mv = MovingAverage(MV_AVG_SIZE)
line_upper_left_mv = MovingAverage(MV_AVG_SIZE)
line_below_right_mv = MovingAverage(MV_AVG_SIZE)
line_upper_right_mv = MovingAverage(MV_AVG_SIZE)

line_below_left_mv.add_sample(0)
line_upper_left_mv.add_sample(0)
line_below_right_mv.add_sample(0)
line_upper_right_mv.add_sample(0)

motor = None

def img_callback(data):
    global image   
    global sub_f 
    global time_c

    sub_f += 1

    if time.time() - time_c > 1:
        time_c = time.time()
        sub_f = 0

    # np_arr = np.fromstring(data.data, np.uint8)
    # image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    image = bridge.imgmsg_to_cv2(data, "bgr8")

# publish xycar_motor msg
def drive(Angle, Speed): 
    global motor

    motor_msg = xycar_motor()
    motor_msg.angle = Angle
    motor_msg.speed = Speed

    motor.publish(motor_msg)

def brightness_control(image, brightness):
    brightness = abs(brightness)
    array = np.full(image.shape, (brightness, brightness, brightness), dtype = np.uint8)

    if brightness > 0:
        bright_img = cv2.add(image, array)
    else:
        bright_img = cv2.subtract(image, array)

    return bright_img

def sharpness_control(image):
    kernel = np.array([[0, -1, 0],
                    [-1, 5, -1],
                    [0, -1, 0]])

    sharp_img = cv2.filter2D(image, -1, kernel)

    return sharp_img

def perspective(frame):
    global WIDTH, HEIGHT
    global src_pt, dst_pt
    global cnt

    # Set Points of Source Image 
    top_y_offset = HEIGHT * 0.55
    below_y_offset = HEIGHT * 0.75
    tl_offset = WIDTH * 0.1
    tr_offset = WIDTH - tl_offset
    bl_offset = WIDTH * 0.01
    br_offset = WIDTH - bl_offset

    src_tl = [tl_offset, top_y_offset]
    src_tr = [tr_offset, top_y_offset]
    src_bl = [bl_offset, below_y_offset]
    src_br = [br_offset, below_y_offset]

    src_pt = np.float32([src_tl, src_tr, src_bl, src_br])

    # Draw Lines of Perspective Area
    point_img = frame.copy()

    tup_src_tl = tuple(map(int, src_tl))
    tup_src_tr = tuple(map(int, src_tr))
    tup_src_bl = tuple(map(int, src_bl))
    tup_src_br = tuple(map(int, src_br))

    cv2.line(point_img, tup_src_tl, tup_src_tr, (0, 0, 255), 2)
    cv2.line(point_img, tup_src_tr, tup_src_br, (0, 255, 0), 2)
    cv2.line(point_img, tup_src_br, tup_src_bl, (255, 0, 0), 2)
    cv2.line(point_img, tup_src_bl, tup_src_tl, (0, 255, 255), 2)

    cv2.imshow('point_img', point_img)

    # Set Points of Destination Image
    dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])

    # Get Perspective Transform Matrix
    pers_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)

    # Get Translated Image
    dst_img = cv2.warpPerspective(frame, pers_mat, (WIDTH, HEIGHT))

    return dst_img

def houghLine(img):
    # Binarize with HLS Value
    blur_img = cv2.GaussianBlur(img,(5, 5), 0)
    hls = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HLS)
    h, l, s = cv2.split(hls)

    _, th = cv2.threshold(l, HLS_THRESH_VAL, 255, cv2.THRESH_BINARY)
    cv2.imshow('threshold_img', th)
    
    # Get Edge by Canny Edge Algorithm
    edge_img = cv2.Canny(np.uint8(th), 200, 75)
    # cv2.imshow('edge_img', edge_img)

    # Get Line by Hough Transformation
    all_lines = cv2.HoughLinesP(edge_img, 1, math.pi / 180, 30, 20, 10)
    line_img = img.copy()

    # Draw Hough Line in Image
    if not all_lines is None:
        for line in all_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # cv2.imshow('line_img', line_img)

    return all_lines

# Filtering Slope and Divide Left and Right Lines
def divide_left_right(lines):
    global WIDTH

    # Calculate Slope and Filtering with Threshold
    new_lines = []

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if (x2 - x1) == 0:
            slope = 0
        else:
            slope = float(y2-y1) / float(x2-x1)
        
        if abs(slope) > 0.5:
            new_lines.append(line[0])

    # Divide Left and Right Lines by Distance from Center
    left_lines = []
    right_lines = []
    th = WIDTH * 0.03

    for j in range(len(new_lines)):
        line = new_lines[j]
        x1, y1, x2, y2 = line

        if (x1 < WIDTH / 2 - th) and (x2 < WIDTH / 2 - th):
            left_lines.append([line.tolist()])
        elif (x1 > WIDTH / 2 + th) and (x2 > WIDTH / 2 + th):
            right_lines.append([line.tolist()])

    return left_lines, right_lines

# Get Line Position
def get_line_pos(img, lines, left=False, right=False):
    global WIDTH, HEIGHT
    global cam_debug
    global line_upper_left, line_upper_right, line_below_right, line_below_left
    
    # Get Average of x, y, m(slope)
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0
    m, b = 0, 0

    size = len(lines)

    found_flag = False

    if size != 0:
        for line in lines:
            x1, y1, x2, y2 = line[0]

            x_sum += x1 + x2
            y_sum += y1 + y2
            m_sum += float(y2 - y1) / float(x2 - x1)

        x_avg = x_sum / (size * 2)
        y_avg = y_sum / (size * 2)

        m = m_sum / size
        b = y_avg - m * x_avg

    if m == 0 or b == 0:
        if left:
            pos = 0
        elif right:
            pos = WIDTH
    else:
        y = GAP / 2
        pos = (y - b) / m

        if cam_debug:
            # Get End Point of Line
            xs = (HEIGHT - b) / float(m)
            xe = ((HEIGHT / 2) - b) / float(m)

            BOUND_LIMIT = 10000
            
            if abs(xs) > BOUND_LIMIT or abs(xe) > BOUND_LIMIT:
                return img, int(pos), found_flag
            
            if left:
                line_below_left_mv.add_sample(xs)
                line_upper_left_mv.add_sample(xe)
                line_below_left = line_below_left_mv.get_wmm()
                line_upper_left = line_upper_left_mv.get_wmm()
                found_flag = True
                # Left: Blue
                color = (255, 0, 0)

            if right:
                line_below_right_mv.add_sample(xs)
                line_upper_right_mv.add_sample(xe)
                line_below_right = line_below_right_mv.get_wmm()
                line_upper_right = line_upper_right_mv.get_wmm()
                found_flag = True
                # Right: Red
                color = (0, 0, 255)

            cv2.line(img, (int(xs), HEIGHT), (int(xe), HEIGHT / 2), color, 3)

    return img, int(pos), found_flag

# show image and return lpos, rpos
def process_image(frame):
    global WIDTH
    global cam, cam_debug
    global line_upper_left, line_upper_right, line_below_right, line_below_left

    # Get Perspective Translated Image
    pers_img = perspective(frame)
    # cv2.imshow('pers_img', pers_img)

    # Get Line of Translated Image
    all_lines = houghLine(pers_img)

    # Divide Left, Right Lines
    if all_lines is None:
        return WIDTH / 2, WIDTH / 2, pers_img, False

    left_lines, right_lines = divide_left_right(all_lines)

    # Get Position of Lines
    left_found, right_found = False, False
    res_img, lpos, left_found = get_line_pos(pers_img, left_lines, left = True)
    res_img, rpos, right_found = get_line_pos(pers_img, right_lines, right = True)

    line_found = (left_found or right_found)

    print('lPos %d, rPos %d' %(lpos, rpos))

    return lpos, rpos, res_img, line_found

def start():
    global image
    global motor
    global WIDTH, HEIGHT

    rospy.init_node('auto_drive')
    motor = rospy.Publisher('xycar_motor', xycar_motor, queue_size=1)
    # image = rospy.Subscriber("/usb_cam/image_raw/compressed", 
    #                         CompressedImage, img_callback, queue_size = 1)
    image = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)                        
    print("---------- Xycar C1 HD v1.0 ----------")
    time.sleep(3)

    t_check = time.time()
    f_n = 0

    while not rospy.is_shutdown():
        while not image.size == (WIDTH * HEIGHT * 3 * RESIZE):
            # print(image.shape)
            continue

        f_n += 1
        image = cv2.resize(image, dsize = (WIDTH, HEIGHT), interpolation = cv2.INTER_CUBIC)

        if (time.time() - t_check) > 1:
            t_check = time.time()
            f_n = 0

        cv2.imshow('src_img', image)

        bright_img = brightness_control(image, BRIGHTNESS_VAL)
        sharp_img = sharpness_control(bright_img)
        # cv2.imshow('sharp_img', sharp_img)
        line_found = False
        lpos, rpos, res_img, line_found = process_image(sharp_img)

        center = (lpos + rpos) / 2
        line_upper_center = (line_upper_left + line_upper_right) / 2
        line_below_center = (line_below_left + line_below_right) / 2
        
        delta_x = line_upper_center - (WIDTH / 2)

        if delta_x == 0:
            delta_x = 0.0001

        grad = (HEIGHT / 2) / delta_x
        angle = math.degrees(math.atan(grad))

        if angle > 0:
            angle = 90 - angle
            angle = min(20, angle)
        else:
            angle = -90 - angle
            angle = max(-20, angle)

        print("THETA: %f" %angle)
        drive(angle, 18)

        if line_found:
            cv2.line(res_img, (WIDTH / 2, HEIGHT), (int(line_upper_center),
                    (HEIGHT / 2)), (0, 255, 0), 3)

            # cv2.line(res_img, (int(line_below_center), HEIGHT), (int(line_upper_center),
            #         (HEIGHT / 2)), (0, 255, 0), 3)

        cv2.putText(res_img, 'angle: ' + str(angle)[:5], (int(WIDTH * 0.6), int(HEIGHT * 0.15)), 
                    1, 1, (0, 255, 255), 1)
        cv2.imshow('res_img', res_img)
        cv2.waitKey(1)

if __name__ == '__main__':
    start()

