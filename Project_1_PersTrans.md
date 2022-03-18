# Project 1: Line Detection with Perspective Transform

## 1. Execute ROS Bag File
![image](https://user-images.githubusercontent.com/53277342/158774855-3dcb51fd-5d94-456f-9106-362e89b9260d.png)  

```
$ roscore			& ROS Master 실행
$ rosbag play ~.bag		& ROS Bag 실행
```

- 문제점  
	- 현상
		- bag 파일이 실행되지 않음
	- 원인
		- compressed 형식으로 저장된 bag 파일  
- 해결법  
	1. 패키지 부분 Image 대신 CompressedImage 패키지 사용
	``` python
	# from sensor_msgs.msg import Image
	# from cv_bridge import CvBridge
	from sensor_msgs.msg import CompressedImage
	```

	2. Callback 함수 부분 Bridge 대신 imdecode 사용
	``` python
	# image = bridge.imgmsg_to_cv2(data, "bgr8")
	np_arr = np.fromstring(data.data, np.uint8)
	image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
	```

	3. Topic Subscribe 부분 경로 /compressed, Image 객체 Compressed Image 로 변경
	``` python
	# image = rospy.Subscriber("/usb_cam/image_raw/", Image, img_callback)
	image = rospy.Subscriber("/usb_cam/image_raw/compressed",
				CompressedImage, img_callback, queue_size = 1)
	```
---
## 2. Image Processing
### 2-1. Lower Brightness   
![image](https://user-images.githubusercontent.com/53277342/158775440-edb82a5d-a917-4906-814c-e57fc5d389a6.png)  
- 사용 이유
	- 원본 영상으로 HLS 이진화가 잘 이루어지지 않아 밝기 조절 필요

``` python
def brightness_control(image, brightness):
	array = np.full(image.shape, (brightness, brightness, brightness), dtype = np.uint8)
	dark_img = cv2.subtract(image, array)

return dark_img
```
---
### 2-2. Set Points for Perspective Transformation  
![image](https://user-images.githubusercontent.com/53277342/158771307-6ee04f01-24b2-4ac7-a484-2fb60707c808.png)  
1. Set Points of Source Image 
``` python
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

2. Set Points of Destination Image
``` python
dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])
```

- 문제점  
	- 화면이 바뀜에 따라 원근 변환을 위한 영역을 어떻게 선택해야할지 고민  
	- 곡선 주행 시 지정 영역 벗어남 문제  
- 해결법
	- 프레임 이미지 저장하여 차선에 해당하는 영역 적절히 선택  
	- 곡선 주행이어도 극단적으로 꺾이는 경우는 사실상 불가능  
---
### 2-3. Get Translated Image   
![image](https://user-images.githubusercontent.com/53277342/158774947-f1b9f592-5d29-48da-991b-ef3b22a00993.png)  

``` python
pers_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
dst_img = cv2.warpPerspective(frame, pers_mat, (WIDTH, HEIGHT))
```
---
### 2-4. Binarize Image by using HLS Value  
![image](https://user-images.githubusercontent.com/53277342/158775115-9ebc1f59-6c84-4f83-921f-cfce85ff80cf.png)  

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
----
### 2-5. Get Edge by using Canny Edge Algorithm   
![image](https://user-images.githubusercontent.com/53277342/158775143-b4fd2282-f7da-4c87-901b-f274820a4a68.png)  

``` python
edge_img = cv2.Canny(np.uint8(th), 60, 75)
```
----
### 2-6. Get Lines    
1. Find All Lines by Hough Transformation  
	![image](https://user-images.githubusercontent.com/53277342/158775212-c3c53158-df2f-4eb9-9360-ec8d7c8f88c9.png)  

	``` python
	# Get Line by Hough Transformation
	all_lines = cv2.HoughLinesP(edge_img, 1, math.pi / 180, 30, 20, 10)
	```
----
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
----
3. Get Line Position  
	![image](https://user-images.githubusercontent.com/53277342/158775266-7092d6b4-a9ec-494c-884f-04e0bc286d3b.png)  

	``` python
	def get_line_pos(img, lines, left=False, right=False):
		global WIDTH, HEIGHT
		global cam_debug
		global line_upper_left, line_upper_right, line_below_right, line_below_left

		# Get Average of x, y, m(slope)
		x_sum = 0.0
		y_sum = 0.0
		m_sum = 0.0
		m, b = 0, 0

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

				if left:
					line_below_left = xs
					line_upper_left = xe
					# Left: Blue
					color = (255, 0, 0)

				if right:
					line_below_right = xs
					line_upper_right = xe
					# Righ: Red
					color = (0, 0, 255)

				cv2.line(img, (int(xs), HEIGHT), (int(xe), (HEIGHT / 2)), color, 3)

		return img, int(pos)
	```
	- 문제점
		- 현상
			- 인식된 차선 변동이 과함
		- 원인
			- 차선이 아닌 다른 선 인식하여 과한 변동 일어남
	- 해결법
		- (가중)이동평균 필터 이용하여 완화
----
4. Add Weighted Moving Average
	- 사용 이유
		- 차선 인식 안정화
		
	``` python
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
	```

	``` python
	line_below_left_mv = MovingAverage(15)
	line_upper_left_mv = MovingAverage(15)
	line_below_right_mv = MovingAverage(15)
	line_upper_right_mv = MovingAverage(15)
	```
---
## 3. Problem and Solving in Other Parts
1. Socket Address Already in Use
	- 현상
		- roscore 명령 실행 시 오류
	- 원인
		- 같은 포트 번호에 다른 프로세스 실행 중인 상태
	- 해결법
		1. 실행 중인 프로세스 확인
		```
		$ netstat -lntp
		```
		2. 해당 프로세스의 PID 확인 후 Kill Process
		```
		$ kill -9 "PID"
		```
		- PID 나오지 않는 경우 sudo로 실행
---
## Result
![image](https://user-images.githubusercontent.com/53277342/158775352-ed5bfff3-39a6-4c3a-b9c0-199156df2af7.png)  

- Blue Line: Left Line
- Red Line: Right Line
- Greed Line: Direction to Move in Driver's View
