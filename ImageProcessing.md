# Image Processing

[**1. OpenCV**](#1-opencv)  
[**2. Hough Transform**](#2-hough-transform)  
[**3. 이미지 기하학적 변형**](#3-이미지-기하학적-변형)  

## 1. OpenCV
1. 개요
	- 컴퓨터 비전 분야에서 널리 이용
	- 영상 데이터 표현, 변환, 분석에 필요한 도구 제공
	- 프로그래밍 인터페이스: 주로 C++, Python
	- 크로스 플랫폼 프레임워크: Windows, Linux, MacOS
	- 머신러닝 프레임워크 지원
	
2. 이미지 표현
	- 점 하나 표현하는 [B, G, R] 3채널 형태 2차원 배열로 표현
	- Data Type: numpy.ndarry
	- 좌표계  
		<img src="https://user-images.githubusercontent.com/53277342/157155404-8c1579db-bfeb-48b4-baf0-7642028e6eca.png" width="50%"/>

3. 이미지 처리
	- 도형 그리기
	```python
	line(img, startPos, endPos, color, thickness)		# Line
	rectangle(img, startPos, endPos, color, thickness)	# Rectangle
	circle(img, centerPos, radius, color, thickness)	# Circle
	putText(img, text, startPos, font, fontScale, color	# Text
	```
	
	- 파일 처리
	```python
	imread(fileName, flags)					# Read Image	
	imshow(fileName, img)					# Show Image
	imwrite(fileName, img)					# Save Image
	```
	
	- 동영상 재생
		- 재생 프로세스
			1. Video Capture 오브젝트 생성
			2. 카메라 디바이스 / 동영상 파일 읽음
			3. 읽은 프레임 화면에 표시
			4. 재생 끝나면 VideoCapture 오브젝트 릴리즈
			5. 윈도우 닫기
	```python
	cap = cv2.VideoCapture(input)	# input 카메라 영상: 0, 동영상: 파일명
	
	while True:
		ret, frame = cap.read()
		
		if ret:
			cv2.imshow('frame', frame)
			
			if cv2.waitKey(1) > 0:
				break
				
	cap.release()
	cv2.destroyAllWindows()
	```
	
	- ROI (Region of Interest)
		- 이미지 슬라이싱으로 설정
	
	- HSV 색 표현  
		<img src="https://user-images.githubusercontent.com/53277342/157157633-a891ec96-3d8c-4981-8443-a12ce024ee80.png" width="20%"/>  
		- H(Hue): 색상, S(Saturation): 채도(선명도), V(Value): 명도

## 2. Hough Transform
1. 직선 표현  
	![image](https://user-images.githubusercontent.com/53277342/157174175-43752adb-feac-4dcf-b6d6-49473de33491.png)  
	- Image Space
		- 일반적인 x, y 좌표계
	- Parameter Space  
		- Image Space에서의 직선의 기울기, 절편을 각각 x, y축으로 표현   
		![image](https://user-images.githubusercontent.com/53277342/157176884-4dd59131-f73b-4578-8371-02a689bcae04.png)   
		- 두 직선의 교점은 Image Space에서 두 점 지나는 직선 의미  
		- 겹치는 직선이 많은 교점일수록 Image Space에서 직선 존재할 가능성 높음
		- 한계
			- 기울기 무한인 직선 표현 어려움
			- Hough Space 보완	
	- Hough Space  
		<img src="https://user-images.githubusercontent.com/53277342/157181092-83d0a9bb-1572-4596-bd8c-d59d5408910a.png" width="50%"/>
		- x축과의 각도(θ), 원점에서 직선까지의 수선의 길이(ρ)를 각각 x, y축으로 표현
		- 기울기 무한인 직선 표현 가능
		- 곡선이 많이 겹치는 교점일수록 Image Space에서 직선 존재 가능성 높음

2. Hough Transform
	- 직선 검출 방식
		1. 입력 영상 GrayScale 변환
		2. Canny Edge로 외곽선 추출
		3. ρ, θ 간격 설정
		4. 외곽선 점들에 대한 (ρ, θ) 좌표값 구함
		5. 오차범위 내 (ρ, θ) 좌표값 갖는 점들 하나의 직선으로 구성
	
	- OpenCV 함수
		```python
		# HoughLines 함수 (직선 검출)
		cv2.HoughLines(img, rho, thetha, threshold)
		# img: 8bit 흑백 이미지
		# rho: Hough Space에서 얼만큼 ρ 증가시키면서 조사할지
		# theta: 얼만큼 θ 증가시키면서 조사할지
		# threshold: threshold 개수 이상의 교점은 하나의 직선 형성
		```
		
		```python
		# HoughLinesP 함수 (선분 검출)
		cv2.HoughLinesP(img, rho, thetha, threshold, minLineLength, maxLineGap)
		# minLineLength: 선분 최소 길이
		# maxLineLength: 간격 최대 길이
		# Return 검출 선분 양끝점
		```

	- OpenCV 차선 검출
		1. Image Read
		2. GrayScale
		3. Gaussian Blur: 노이즈 제거
		4. Canny: Edge 검출
		5. ROI
		6. HoughLinesP

	- 조향각 설정
		- 양쪽 차선 중점이 영상의 중앙에서 벗어난 픽셀 수 확인

3. 차선 인식 주행
	- 프로그램 흐름도
		1. 카메라 노드에서 토픽 구독, 영상 프레임 획득
		2. 영상 프레임 OpenCV 함수 처리
		3. OpenCV 영상처리
		4. 차선 위치 확인, 중앙에서 치우침 확인
		5. 핸들 조향각 설정 계산
		6. 모터 제어 토픽 발행, 차량 움직임 조종

	- Launch 파일
		```xml
		<!-- h_drive.launch -->
		<launch>
			<!-- 모터 제어기 구동 -->
			<include file="$(find xycar_motor)/launch/xycar_motor.launch" />
		
			<!-- 카메라 구동 -->
			<node name="usb_cam" output="screen" pkg="usb_cam" type="usb_cam_node">
				<param name="video_device" value="/dev/videoCAM" />
				<param name="autoexposure" value="false" />
				<param name="exposure" value="162" />
				<param name="image_width" value="640" />
				<param name="image_height" value="480" />
				<param name="pixel_format" value="yuyv" />
				<param name="camera_frame_id" value="usb_cam" />
				<param name="io_method" value="mmap" />
			</node>

			<!-- 프로그램 실행 -->
			<node name="h_drive" output="screen" pkg="h_drive" type="h_drive.py" />
		</launch>
		```

## 3. 이미지 기하학적 변형
1. Warping
	- 이미지 복원 목적 영상 이동, 회전, 크기 변환 처리기법
	- 강체변환 (Rigid-Body)
		- 크기, 각도 보존되는 변환
		- Translation
			- 변환 행렬으로 연산
			```python
			dst = cv2.warpAffine(src, mat, dsize, dst, flags, borderMode, borderValue)
			# mat: 2x3 변환 행렬
			# dsize: 결과 이미지 크기 (width, height)
			# flags: 보간법 알고리즘
			# borderMode: 외곽영역 보정
			# borderValue: 외곽영역 보정 사용 색상 값
			```
		- Rotation
			- 변환 행렬식 연산
			```python
			dst = cv2.getRotationMatrix2D(center, angle, scale)
			# scale: 확대 축소 비율
			```
	- 유사변환 (Similarity)
		- 크기 변경, 각도 보존
		- Scaling
			```python
			dst = cv2.resize(src, dsize, dst, fx, fy, interpolation)
			# fx, fy: 크기 배율, dsize 생략 시 적용
			# interpolation: 보간법 알고리즘 선택 플래그, warpAffine과 동일
			```
	- 선형변환 (Linear)
		- Vector 공간에서의 이동
	- Affine 변환
		- 선형변환 + 이동변환, 선의 수평성은 유지
	- 원근변환 (Perspective)
		- 원근법 적용한 변환		
		- 좌상-좌하-우상-우하 순서로 좌표값 지정
			```python
			dst = cv2.warpPerspective(stc, mat, dsize)
			```
		- 차선 추출에 사용
			- 원근 현상 없애는 변환 이용
			1. 도로 이미지를 Bird Eye View 변형 처리
			2. 차선 찾아 원본 이미지에 합침
	
2. 원근 변환과 슬라이딩 윈도우 이용한 차선 검출
	1. Camera Calibration (카메라 보정)
		- 이미지 왜곡
			- 카메라의 다양한 내부적 요인들로 인해 발생
		- 왜곡된 지점을 왜곡되지 않은 지점으로 매핑해 왜곡 제거
		- 에러 감지에 용이한 체스판 이미지 사용







