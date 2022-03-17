# Project
## Mission 1: Line Detection by using Perspective Transform

### 1. ROS Bag Play
```
$ roscore			& ROS Master 실행
$ rosbag play ~.bag		& ROS Bag 실행
```

![image](https://user-images.githubusercontent.com/53277342/158741246-1390a776-6afd-4be6-955e-9402c530ae57.png)

- 문제점  
	- 현상
		- bag 파일이 실행되지 않음
	- 원인
		- compressed 형식으로 저장된 bag 파일이어서 실행되지 않았음  
- 해결법  
	``` python
	# 패키지 부분
	# from sensor_msgs.msg import Image
	# from cv_bridge import CvBridge
	# -> Image 대신 CompressedImage 패키지 사용
	from sensor_msgs.msg import CompressedImage

	# Callback 함수 부분
	# image = bridge.imgmsg_to_cv2(data, "bgr8")
	# -> Bridge 대신 imdecode 사용
	np_arr = np.fromstring(data.data, np.uint8)
	image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

	# Topic Subscribe 부분
	# image = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
	# -> 경로 /compressed 로 변경
	# -> Image 객체를 Compressed Image 로 변경
	image = rospy.Subscriber("/usb_cam/image_raw/compressed",
				CompressedImage, img_callback, queue_size = 1)
	```

### 2. Image Processing
1. Lower Brightness   
	![image](https://user-images.githubusercontent.com/53277342/158741281-761f636a-3909-464f-8040-c4f516276195.png)

	``` python
	def brightness_control(image, brightness):
		array = np.full(image.shape, (brightness, brightness, brightness), dtype = np.uint8)
		dark_img = cv2.subtract(image, array)

    return dark_img
	```

2. Set Points for Perspective Transformation  
	![image](https://user-images.githubusercontent.com/53277342/158741302-2c5db7db-8e12-46eb-a1d7-afbd8908a8e9.png)
	``` python
	# Set Points of Source Image 
	top_y_offset = HEIGHT * 0.5
	below_y_offset = HEIGHT * 0.9
	tl_offset = WIDTH * 0.3
	tr_offset = WIDTH * 0.7
	bl_offset = WIDTH * 0.1
	br_offset = WIDTH * 0.9

	src_tl = [tl_offset, top_y_offset]
	src_tr = [tr_offset, top_y_offset]
	src_bl = [bl_offset, below_y_offset]
	src_br = [br_offset, below_y_offset]

	src_pt = np.float32([src_tl, src_tr, src_bl, src_br])
	```
	``` python
    # Set Points of Destination Image
    dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])
	```
	
	- 문제점  
		- 화면이 바뀜에 따라 원근 변환을 위한 영역을 어떻게 선택해야할지 고민  
		- 곡선 주행 시 지정 영역 벗어남 문제  
	- 해결법
		- 프레임 이미지 저장하여 차선에 해당하는 영역 적절히 선택  
		- 곡선 주행이어도 극단적으로 꺾이는 경우는 사실상 불가능  

3. Get Translated Image   
	![image](https://user-images.githubusercontent.com/53277342/158741346-d9e4db9a-d9f5-4bed-b5c7-de472fa27df5.png)  
	``` python
	pers_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
	dst_img = cv2.warpPerspective(frame, pers_mat, (WIDTH, HEIGHT))
	```
	
4. Binarize Image by using HLS Value  
	![image](https://user-images.githubusercontent.com/53277342/158741384-87e56e30-00ff-4c1f-9eba-7b979b17f558.png)  
	``` python
	blur_img = cv2.GaussianBlur(img,(5, 5), 0)
    hls = cv2.cvtColor(blur_img, cv2.COLOR_BGR2HLS)
    h, l, s = cv2.split(hls)

    _, th = cv2.threshold(l, 100, 255, cv2.THRESH_BINARY)
    cv2.imshow('threshold_img', th)
	```  
	- 문제점
		- 현상
			- 차선 인식 이진화가 잘 이루어지지 않음
		- 원인
			- 영상 밝기 변화
	- 해결법
		- HLS 도입
		- 영상 Brightness 조절
		- 밝기 균일한 영상 이용
	
5. Get Edge by using Canny Edge Algorithm  
	![image](https://user-images.githubusercontent.com/53277342/158741418-878a9e1f-4f7c-427f-8ecb-0c2e649a5327.png)
	``` python
	edge_img = cv2.Canny(np.uint8(th), 60, 75)
	```

6. Get Lines  
	![image](https://user-images.githubusercontent.com/53277342/158741441-ca299aef-8b81-429e-820b-1f6619b7ec02.png)
	
	1. Find All Lines by Hough Transformation
		``` python
		# Get Line by Hough Transformation
		all_lines = cv2.HoughLinesP(edge_img, 1, math.pi / 180, 30, 20, 10)
		```
	
	2. Filter Slope and Divide Left and Right Lines
		``` python
		def divide_left_right(lines):
			global WIDTH

			low_slope_threshold = 0.1
			high_slope_threshold = 30

			# Calculate Slope and Filtering with Threshold
			slopes = []
			new_lines = []

			for line in lines:
				x1, y1, x2, y2 = line[0]

				if x2 - x1 == 0:
					slope = 0
				else:
					slope = float(y2-y1) / float(x2-x1)

				if low_slope_threshold < abs(slope) < high_slope_threshold:
					slopes.append(slope)
					new_lines.append(line[0])

			# Divide Left and Right Lines by Distance from Center
			left_lines = []
			right_lines = []
			th = WIDTH * 0.03

			for j in range(len(slopes)):
				line = new_lines[j]
				x1, y1, x2, y2 = line

				if (abs(slope) > 0.1) and (x2 < WIDTH / 2 - th):
					left_lines.append([line.tolist()])
				elif (abs(slope) > 0.1) and (x1 > WIDTH / 2 + th):
					right_lines.append([line.tolist()])

			return left_lines, right_lines
		```
	3. Get Line Position
		``` python
		# Get Line Position
def get_line_pos(img, lines, left=False, right=False):
    global WIDTH, HEIGHT
    global cam_debug
    global line_upper_left, line_upper_right, line_below_right, line_below_left
    
    # Get Average of x, y, m(slope)
    x_sum = 0.0
    y_sum = 0.0
    m_sum = 0.0

    size = len(lines)
    
    m = 0
    b = 0

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
        y = Gap / 2

        pos = (y - b) / m

        if cam_debug:
            # Get End Point of Line
            xs = (HEIGHT - b) / float(m)
            xe = ((HEIGHT / 2) - b) / float(m)
            
            if left:
                line_below_left = xs
                line_upper_left = xe
                color = (255, 0, 0)

            if right:
                line_below_right = xs
                line_upper_right = xe
                color = (0, 0, 255)

            cv2.line(img, (int(xs), HEIGHT), (int(xe), (HEIGHT / 2)), color, 3)
            print("xs: %d, xe: %d" %(xs, xe))

    return img, int(pos)
		```

