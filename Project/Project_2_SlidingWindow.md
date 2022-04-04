# Project 2: Lane Detection with Sliding Window
> <b>Advanced Lane Detection by using Sliding Window</b>  
> [**View Code**](https://github.com/HoyeonYu/ROS_Study/blob/master/Project/Project_2_SlidingWindow.py)

---
## Problem of Previous Sliding Window Algorithm
1. Detect Lanes Only in Image
	- Lane Actually Exist, but Not Visible in Camera Image
	![image](https://user-images.githubusercontent.com/53277342/161479818-fcc8e2e9-d9e7-440d-966e-127ee58152c4.png)
	![image](https://user-images.githubusercontent.com/53277342/161479768-66f378e6-3230-459e-af3e-6d5ed568d56e.png)
	![image](https://user-images.githubusercontent.com/53277342/161479784-3e88017a-8330-466d-ab76-95ae4b0c453a.png)
2. Not Follow Previous Window
	![image](https://user-images.githubusercontent.com/53277342/161481210-0ce89be1-8fa9-4749-be5e-87241fc40ace.png)
3. Distorted Lane Fill
	![image](https://user-images.githubusercontent.com/53277342/161481230-f5a47e8a-ab28-4d86-8db5-9d90ec4f50c6.png)
---

## Advanced Sliding Window
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
			![image](https://user-images.githubusercontent.com/53277342/161481599-de03c851-a33f-465e-b224-90a72d95ea87.png)
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
			![image](https://user-images.githubusercontent.com/53277342/161481814-a850ff18-b87e-45f4-8879-33860d5914c3.png)
			```python
			# Left Window Not Found
			else:
				cur_left_x = prev_left_x + prev_right_diff
				prev_left_diff = prev_right_diff
				if window_idx == 0:
					cur_left_x = cur_right_x - LANE_WIDTH
				left_found_flag.append(False)
			```
			![image](https://user-images.githubusercontent.com/53277342/161481663-847aedef-ddbf-4b1d-8ab5-760c8a70c0e3.png)  
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
---

## 5. Get Velocity for Distance by using IMU Sensor
```python
# IMU Topic Callback for Checking Velocity of Car
def imu_callback(data):
    global imu_callback_time, vel_x, vel_y, cur_vel, prev_vel, first_drive_flag

    if imu_callback_time is None:
        dt = 0
        imu_callback_time = time.time()

        return
    else:
        dt = time.time() - imu_callback_time

    # Get Velocity of X, Y
    vel_x += (data.linear_acceleration.x * dt)
    vel_y += (data.linear_acceleration.y * dt)

    cur_vel = math.sqrt(math.pow(vel_x, 2) + math.pow(vel_y, 2))

    # Velocity == 0 if Same Velocity
    if cur_vel == prev_vel:
        vel_x, vel_y = 0, 0
        cur_vel = 0
        first_drive_flag = True

    prev_vel = cur_vel
    imu_callback_time = time.time()
```

```python
move_dist = cur_vel * (time.time() - prev_time)
```

---

## 6. Calculate Angle for Steering
1. Get Unity Distance-CV Pixel Ratio
	```python
	METER_PER_PIXEL = 0.055
	```

2. Get Car-Camera Distance   
	![image](https://user-images.githubusercontent.com/53277342/161472795-fe32c992-995c-4f2e-a9e7-139a946b5615.png)  
	- Front Wheel (5.53, 1.62, 6.78)
	- Back Wheel (5.56, 1.68, 2.80)
	- Camera (6.96, 4.18, 6.47)
	- Distance: 3.62(m)  
		-> Pixel Translated Distance: 65.82(pxl)

3. Calculate Angle
	```python
	# Calculate Drive Angle
	def get_drive_angle(dist):
		global x_left_list, x_right_list

		move_y_dist = dist * METER_PER_PIXEL
		target_y = TARGET_X_IDX * WINDOW_HEIGHT
		target_y -= move_y_dist
		idx_y = int((float(target_y) / HEIGHT) * N_WINDOWS)
		idx_y = min(max(0, idx_y), N_WINDOWS - 1)

		diff_x = ((x_left_list[idx_y] + x_right_list[idx_y]) / 2) - (WIDTH / 2)

		control_grad = diff_x / float(target_y + CAR_PIXEL)
		control_angle = math.degrees(math.atan(control_grad))

		return control_angle, target_y
	```

---

## 7. Fill Front Lane  
![image](https://user-images.githubusercontent.com/53277342/161466394-cbd2ac0e-5416-49f7-8d0a-cea623b218f9.png)

1. Polyfit Left, Right Points
```python
x_left_list, y_left_list, x_right_list, y_right_list, sliding_img = sliding_window(bin_lane_img)

left_fit = np.polyfit(np.array(y_left_list), np.array(x_left_list), 2)
right_fit = np.polyfit(np.array(y_right_list), np.array(x_right_list), 2)
```

2. Broaden Filled Area
```python
# Border Points
border_top_y_offset = top_y_offset
border_below_y_offset = below_y_offset
border_tl_offset = ((tl_offset * IMG_BORDER) / (WIDTH/2)) + tl_offset
border_tr_offset = (WIDTH+2*IMG_BORDER) - border_tl_offset
border_bl_offset = ((bl_offset * IMG_BORDER) / (WIDTH/2)) + bl_offset
border_br_offset = (WIDTH+2*IMG_BORDER) - border_bl_offset

border_src_tl = [border_tl_offset, border_top_y_offset]
border_src_tr = [border_tr_offset, border_top_y_offset]
border_src_bl = [border_bl_offset, border_below_y_offset]
border_src_br = [border_br_offset, border_below_y_offset]

border_src_pt = np.float32([border_src_tl, border_src_tr, border_src_bl, border_src_br])
border_dst_pt = np.float32([[0, 0], [WIDTH+2*IMG_BORDER, 0], [0, HEIGHT], [WIDTH+2*IMG_BORDER, HEIGHT]])
```

3. Get Inversed Warp Matrix
```python
# Get Perspective Transform Matrix
border_warp_mat = cv2.getPerspectiveTransform(border_src_pt, border_dst_pt)
border_warp_inverse_mat = cv2.getPerspectiveTransform(border_dst_pt, border_src_pt)
```

3. Fill Color in Inversed Matrix
	1. Fill Polygon  
	![image](https://user-images.githubusercontent.com/53277342/161476453-20c0071a-5f10-497f-ba69-c923579cc1ed.png)  
	![image](https://user-images.githubusercontent.com/53277342/161476480-430b2e55-e4b8-452f-b0ef-71e13b8b0e83.png)  

	```python
	warp_img = cv2.copyMakeBorder(warp_img, 0, 0, IMG_BORDER, IMG_BORDER, cv2.BORDER_CONSTANT, cv2.BORDER_CONSTANT)
	plot_y = np.linspace(0, HEIGHT - 1, HEIGHT)
	color_warp = np.zeros_like(warp_img).astype(np.uint8)

	left_fit_x = left_fit[0] * (plot_y**2) + left_fit[1]*plot_y + left_fit[2] + IMG_BORDER
	right_fit_x = right_fit[0] * (plot_y**2) + right_fit[1]*plot_y + right_fit[2] + IMG_BORDER

	left_points = np.array([np.transpose(np.vstack([left_fit_x, plot_y]))])
	right_points = np.array([np.flipud(np.transpose(np.vstack([right_fit_x, plot_y])))])
	points = np.hstack((left_points, right_points))

	color_warp = cv2.fillPoly(color_warp, np.int_([points]), GREEN_COLOR)
	cv2.imshow('color_warp', color_warp)
	```

	2. Inverse Perspective Transform  
	![image](https://user-images.githubusercontent.com/53277342/161476681-e967e145-759b-44e9-98e8-3eaab5fbaa44.png)    
  
	```python
	new_warp = cv2.warpPerspective(color_warp, border_warp_inverse_mat, (WIDTH+2*IMG_BORDER, HEIGHT))
	new_warp = new_warp[:, IMG_BORDER:IMG_BORDER+WIDTH]
	cv2.imshow('new_warp', new_warp)
	```

	3. Compose in Source Image  
	![image](https://user-images.githubusercontent.com/53277342/161476850-1d6a8036-cbb3-4bfa-a0be-29f82fa7867b.png)  

	```python
	new_lane_img = cv2.addWeighted(image, 1, new_warp, 0.3, 0)
	cv2.imshow('new_lane_img', new_lane_img)
	```
	
---

## 6. Result
1. Case: Normal Lane  
	![image](https://user-images.githubusercontent.com/53277342/161477759-0299615d-af46-4982-866c-fe4f1890a860.png)
	![image](https://user-images.githubusercontent.com/53277342/161477784-d1e6d7dd-7874-4c37-88f5-2f1b781c8706.png)
	- Previous  
	![image](https://user-images.githubusercontent.com/53277342/161477672-9c8b34ad-7e67-430d-9d35-45096b2b9fbc.png)
	- Advanced  
	![image](https://user-images.githubusercontent.com/53277342/161477726-f433733c-25ef-4d29-b07f-c84388fe558b.png)
	- Filled Lane  
	![image](https://user-images.githubusercontent.com/53277342/161477805-e44e34a6-2b0a-4176-9893-3e5fec137498.png)

---
2. Case: Lane Missing  
	![image](https://user-images.githubusercontent.com/53277342/161478004-96430e9d-fe13-483d-881e-8718057d9fb0.png)
	![image](https://user-images.githubusercontent.com/53277342/161478033-7b5cae14-4a98-431a-ae49-102a621d3b46.png)
	- Previous Result   
	![image](https://user-images.githubusercontent.com/53277342/161478053-61fbdba3-1e69-431f-a076-12b8118b9147.png)
	- Advanced Result    
	![image](https://user-images.githubusercontent.com/53277342/161478074-377615cd-cbb0-4e14-a351-3ec6eb7cd16a.png)
	- Filled Lane Result  
	![image](https://user-images.githubusercontent.com/53277342/161478092-1607d0be-d108-48af-b9af-9184f9e6aecd.png)

----
3. Case: Lane Not Detected  
	![image](https://user-images.githubusercontent.com/53277342/161475568-8f6ccbd1-948e-4cde-912d-204c722342b7.png)
	![image](https://user-images.githubusercontent.com/53277342/161475596-b56fbe92-bcb0-4884-a5f7-a1669f04e370.png)
	- Previous Result  
	![image](https://user-images.githubusercontent.com/53277342/161475665-743187cc-65ce-4e6e-abf5-41bd032fb981.png)
	- Advanced Result  
	![image](https://user-images.githubusercontent.com/53277342/161475686-74b59f61-b204-40eb-a427-7090369a54d1.png)
	- Filled Lane Result  
	![image](https://user-images.githubusercontent.com/53277342/161475708-666e53de-823b-4cc9-803b-56b274dd514b.png)
	
---
4. Case: Other Lane Detected  
	![image](https://user-images.githubusercontent.com/53277342/161475135-e8ceacaf-4995-4f40-a002-4cbf655b5b3b.png)
	![image](https://user-images.githubusercontent.com/53277342/161475242-0bffa975-8dd1-47c9-942b-71c47b2cb812.png)
	- Previous Result  
	![image](https://user-images.githubusercontent.com/53277342/161475071-2e3b0968-6a1f-420a-b20f-bc8360240654.png)
	- Advanced Result  
	![image](https://user-images.githubusercontent.com/53277342/161475103-aa1b51d8-a1ca-4c45-b122-dcc0aac26f83.png)
	- Lane Filled Result  
	![image](https://user-images.githubusercontent.com/53277342/161475265-03ca5c4a-9f1e-49a9-9c3d-6fa97abe8445.png)
---
