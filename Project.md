# Project
## Mission 1: Line Detection by using Perspective Transform

### 1. ROS Bag Play
```
$ roscore			& ROS Master 실행
$ rosbag play ~.bag		& ROS Bag 실행
```
<img src="https://user-images.githubusercontent.com/53277342/158725256-828a9b66-df98-4647-9b0f-a31cba627164.png" width="40%"/>
![image](https://user-images.githubusercontent.com/53277342/158736196-cfe87e07-3167-403e-b54e-cd07cb778a6a.png)

- 문제점  
	- 현상
		- 실행되지 않음
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
	
### 2. Set Points of Source Image  
<img src="https://user-images.githubusercontent.com/53277342/158735316-bba2e8f3-ee8b-4d3e-9790-5897c756890b.png" width="40%"/>
![image](https://user-images.githubusercontent.com/53277342/158736151-5ba66eaa-5080-4ec4-bb33-8b8618e7ef56.png)

``` python
# Set Points of Source Image 
top_y_offset = 250
below_y_offset = 430
tl_offset = 200
tr_offset = 450
bl_offset = 50
br_offset = 600

src_tl = [tl_offset, top_y_offset]
src_tr = [tr_offset, top_y_offset]
src_bl = [bl_offset, below_y_offset]
src_br = [br_offset, below_y_offset]

src_pt = np.float32([src_tl, src_tr, src_bl, src_br])
```
- 문제점  
	- 화면이 바뀜에 따라 원근 변환을 위한 영역을 어떻게 선택해야할지 고민  
	- 곡선 주행 시 지정 영역 벗어남 문제  
- 개선 방안
	- 프레임 이미지 저장하여 차선에 해당하는 영역 적절히 선택  
	- 곡선 주행이어도 극단적으로 꺾이는 경우는 사실상 불가능  

2. Set Points of Destination Image
``` python
dst_pt = np.float32([[0, 0], [Width, 0], [0, Height], [Width, Height]])
```

3. Get Translated Image
<img src="https://user-images.githubusercontent.com/53277342/158499389-8b33ac1d-cd18-4cd6-8f35-dd3c24d05ad5.png" width="40%"/>
![image](https://user-images.githubusercontent.com/53277342/158736230-c7eca2e2-32cb-4050-bf7b-0a89ee383291.png)

```python
pers_mat = cv2.getPerspectiveTransform(src_pt, dst_pt)
dst_img = cv2.warpPerspective(frame, pers_mat, (Width, Height))
```

- 문제점  
    - 영상의 밝기, 배경이 고르지 않은 까닭에 차선 인식이 제대로 이루어지지 않음  
![image](https://user-images.githubusercontent.com/53277342/158736266-b95ee5c0-d979-4138-ada0-ee2e47f50155.png)
![image](https://user-images.githubusercontent.com/53277342/158736298-3c3ca081-40bd-4e18-a44b-a4c4e884cf01.png)
![image](https://user-images.githubusercontent.com/53277342/158736333-7d1ea49d-5f85-4fd6-8657-04be5e7b13da.png)
