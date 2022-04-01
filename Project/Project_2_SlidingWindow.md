# Project 2: Lane Detection with Sliding Window
> <b>Advanced Lane Detection by using Sliding Window</b>  
> [**View Code**](https://github.com/HoyeonYu/ROS_Study/blob/master/Project/Project_2_SlidingWindow.py)

---

## 1. Execute Unity Driving Simulator
```
$ roslaunch rosbridge_server rosbridge_websocket.launch
$ ./xycar3dsimulator.x86_64
```

---
## 2. Warp Image by Perspective Transform
1. Target Area  
	![image](https://user-images.githubusercontent.com/53277342/161211979-2a60acac-2193-43a6-9269-0cb0acd76d5f.png)  

2. Warped Image  
	![image](https://user-images.githubusercontent.com/53277342/161212031-3a1ea87f-522a-4428-9cb3-d0c6b99cd7a2.png)  

```python
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

	# Set Points of Destination Image
	dst_pt = np.float32([[0, 0], [WIDTH, 0], [0, HEIGHT], [WIDTH, HEIGHT]])

	# Get Perspective Transform Matrix
	warp_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
	warp_inverse_mat = cv2.getPerspectiveTransform(dst_pt, src_pt)

	# Get Translated Image
	dst_img = cv2.warpPerspective(img, warp_mat, (WIDTH, HEIGHT))
	cv2.imshow('dst_img', dst_img)
```

---
## 3. Binarize Image by HLS Value
1. Previous Binarization  
![image](https://user-images.githubusercontent.com/53277342/161212773-84680820-044e-4aff-b348-ddb277b3fcf9.png)  

2. Advanced Binarization: Filter Yellow Lane  
![image](https://user-images.githubusercontent.com/53277342/161212801-24975a69-6b25-400d-8809-f0a59e919bf4.png)

	```python
	def binarize_image(src_img):
		blur = cv2.GaussianBlur(src_img, (5, 5), 0)
		hls = cv2.cvtColor(blur, cv2.COLOR_BGR2HLS)
		hls_yellow_binary = cv2.inRange(hls, (20, 145, 100), (70, 255, 255))
		hls_all_binary = cv2.inRange(hls, (0, 145, 0), (255, 255, 255))
		bin_lane_img = cv2.bitwise_xor(hls_yellow_binary, hls_all_binary)
	```

---
## 4. Find Lane with Sliding Window
![image](https://user-images.githubusercontent.com/53277342/161215195-a1725871-68e9-496d-8a34-d5c7468d8fef.png)

1. Find Start Point by using Histogram
	```python
	HIST_FIND_HEIGHT_RATE = 0.6
	histogram = np.sum(bin_img[int(HEIGHT * HIST_FIND_HEIGHT_RATE):, :], axis=0)
	midpoint = int(WIDTH / 2)
	```

2. Make Border for Broaden View
	```python
	sliding_img = np.dstack((bin_img, bin_img, bin_img)) * 255
	sliding_img = cv2.copyMakeBorder(sliding_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)

	cur_left_x = np.argmax(histogram[:midpoint])
	cur_right_x = np.argmax(histogram[midpoint:]) + midpoint
	```

3. Find Next Upper Window Area
	```python
	for window_idx in range(N_WINDOWS):
		win_yl = HEIGHT - (window_idx + 1) * WINDOW_HEIGHT
		win_yl_part = win_yl + WINDOW_PART_RATE * WINDOW_HEIGHT
		win_yh = HEIGHT - window_idx * WINDOW_HEIGHT

		win_xll = cur_left_x - WINDOW_MARGIN
		win_xlh = cur_left_x + WINDOW_MARGIN
		win_xrl = cur_right_x - WINDOW_MARGIN
		win_xrh = cur_right_x + WINDOW_MARGIN

		left_found_upper_part = \
		((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xll) & (nz[1] < win_xlh)).nonzero()[0]
		right_found_upper_part = \
		((nz[0] >= win_yl) & (nz[0] < win_yl_part) & (nz[1] >= win_xrl) & (nz[1] < win_xrh)).nonzero()[0]

	```
	1. Window Finding Cases
		1. If: (Window Found),  
			- Save Current X Position and Calculate Difference Between Previous Data   

			```python
			# Left Window Found
			if len(left_found_upper_part) > MIN_PIX:
				cur_left_x = np.int(np.mean(nz[1][left_found_upper_part]))
				prev_left_diff = cur_left_x - prev_left_x
				left_found_flag.append(True)
				total_left_found_flag = True
			```

			```python
			# Right Window Found
			if len(right_found_upper_part) > MIN_PIX:
				cur_right_x = np.int(np.mean(nz[1][right_found_upper_part]))
				prev_right_diff = cur_right_x - prev_right_x
				right_found_flag.append(True)
				total_right_found_flag = True
			```

		2. Else, If: (Window Not Found but Another Side Window Found),  
			- Estimate X Position from Another Side X Position by using Fixed Lane Width  

			```python
			# Left Window Not Found
			else:
				cur_left_x = prev_left_x + prev_right_diff
				prev_left_diff = prev_right_diff
				if window_idx == 0:
					cur_left_x = cur_right_x - LANE_WIDTH
				left_found_flag.append(False)
			```

			```python
			# Right Window Not Found
			else:
				cur_right_x = prev_right_x + prev_left_diff
				prev_right_diff = prev_left_diff
				if window_idx == 0:
					cur_right_x = cur_left_x + LANE_WIDTH
				right_found_flag.append(False)
			```

		3. Else: (Not Both Lane Found),  
			- Estimate X Position with Own Previous Position and Difference  
			
			```python
			# Both Left, Right Window Not Found
			if (len(left_found_upper_part) < MIN_PIX) and (len(right_found_upper_part) < MIN_PIX):
				cur_left_x = prev_left_x + prev_left_diff
				cur_right_x = prev_right_x + prev_right_diff
				left_found_flag.append(False)
				right_found_flag.append(False)
			```

	2. Save Left, Right Window X Positions  
		```python
		        x_left_list.append(cur_left_x)
        x_right_list.append(cur_right_x)

        y_left_list.append((win_yl + win_yh) / 2)
        y_right_list.append((win_yl + win_yh) / 2)

        prev_left_x = cur_left_x
        prev_right_x = cur_right_x
		```
	
	3. Check X Position whether Same or Different Lane
		1. Check Upper, Middle, Below X Position and Calculate Difference with Previous Position
			```python
			y1_point = int(N_WINDOWS * 0.8 / N_WINDOWS)
			y2_point = int(N_WINDOWS * 0.5 / N_WINDOWS)
			y3_point = int(N_WINDOWS * 0.2 / N_WINDOWS)

			left_dx1 = abs(x_left_list[y1_point] - prev_x_left_list[y1_point])
			left_dx2 = abs(x_left_list[y2_point] - prev_x_left_list[y2_point])
			left_dx3 = abs(x_left_list[y3_point] - prev_x_left_list[y3_point])

			right_dx1 = abs(x_right_list[y1_point] - prev_x_right_list[y1_point])
			right_dx2 = abs(x_right_list[y2_point] - prev_x_right_list[y2_point])
			right_dx3 = abs(x_right_list[y3_point] - prev_x_right_list[y3_point])
			
			left_diff = (left_dx1 + left_dx2 + left_dx3) / 3
  			right_diff = (right_dx1 + right_dx2 + right_dx3) / 3
			```
		
		2. Found Lane Cases
			1. If: (Both Lane Correct),  
				- Return
				
			2. Else, If: (Not One Lane Correct),  
				- Estimate Lane from Another Correct Lane by using Fixed Lane Width  
				
				```python
				if left_diff > MAX_AVG_GAP:
					x_left_list = x_right_list[:] - np.full(len(x_right_list), LANE_WIDTH)

				if right_diff > MAX_AVG_GAP:
					x_right_list = x_left_list[:] + np.full(len(x_left_list), LANE_WIDTH)

				```
			3. Else: (Not Both Lane Correct),
				- Estimate Lane with Own Previous Lane
				```python
				if (left_diff > MAX_AVG_GAP) and (right_diff > MAX_AVG_GAP):
					x_left_list = prev_x_left_list[:]
					x_right_list = prev_x_right_list[:]
				```
	
	4. Save Start Position with Moving Average Filtering
		```python
		# Update Position List
		cur_left_x_mv.add_sample(x_left_list[0])
		cur_right_x_mv.add_sample(x_right_list[0])
		```

	
	
	
	
	
	
	
