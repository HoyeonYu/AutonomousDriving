# XyCar

## 1. 원격 접속환경 구축
1. 자이카 환경
	- 프로세서
	- SSD
	- 무선 랜
	- (키보드 + 마우스 + 모니터)

2. 무선 접속
	- Text 기반 원격접속
		- SSH 원격접속
			- PuTTY
	- GUI 기반 원격접속
		- 가상 네트워크 컴퓨팅 VNC (Virtual Network Computing)
		- Wifi 무선공유기 사용
		- AP로 동작
		- SSID: 무선 공유기 이름

## 2. 자이카 기능
- 자율주행
	- 차선 인식해서 벗어나지 않게 주행
- 뷰어 실행
	- 장착된 센서 가동
	- 센서 데이터 시각화해 보여줌
- 사람 인식 주행
	- 카메라에 잡힌 사람 쫓아 주행 (YOLO)
- 화면 크기 확장
	- VNC 화면 크기 확장
- Depth 카메라 뷰어 실행
	- Depth 카메라 영상 화면 보여줌
- 카메라 위치 조정
	- 카메라 위치 세팅 화면 보여줌
- 장애물 회피 주행
	- 거리센서 기반으로 장애물 회피 주행
	- 주행 방식 시나리오
		1. 장애물 감지 전까지 직진
		2. 장애물 감지 시 2초동안 후진
		3. 오른쪽으로 2초동안 우회전
		4. 다시 1번으로
- 조이스틱 제어
	- 안드로이드 앱으로 차량 수동조작

## 3. 자이카 
1. 자이카 하드웨어  
	![image](https://user-images.githubusercontent.com/53277342/156704150-2caf7fce-142c-4785-8b72-769943f6d99f.png)

2. 자이카 소프트웨어  
	![image](https://user-images.githubusercontent.com/53277342/156706822-2a2fbe08-b1e3-417f-86a2-20e351298b7c.png)

3. 자이카 구성도  
	![image](https://user-images.githubusercontent.com/53277342/156704551-ce06d8c0-d3b6-417c-a50e-fe308709e49f.png)

4. 자동차 구동부  
	![image](https://user-images.githubusercontent.com/53277342/156704732-6faa935b-cc76-4fca-90b8-b059c5fe881f.png)
	- 구동 모터
		- 전진, 후진 기능
		- 원리
			- 플레밍의 왼손 법칙
			- 모터 회전속도는 공급되는 전력량에 따라 결정
		- BLDC 모터 사용
	- 조향 모터
		- 좌우 정해진 각도 내에서 왕복 동작
		- 원리
			- PWM 신호(Duty Cycle 제어)로 모터 회전각도 조종
		- 서보 모터 사용
	- 모터 제어기
		- ESC (Electronic Speed Controller)
			- 모터 스피드 제어기
			- VESC (Vedder ESC)
				- 오픈소스 ESC
				- 구동모터 전후진 속도 제어
				- 조향모터 좌우 회전 제어

## 4. 자이카 ROS 패키지
1. ROS 패키지
	- ROS 노드 정보
		- 모터제어기
			- /xycar_motor
		- 카메라
			- /usb_cam
		- IMU 센서
			- /xycar_imu
		- 라이다
			- /xycar_lidar
		- 초음파센서
			- /xycar_ultrasonic
		- Depth 카메라
			- /camera/realsense2_camera
	- ROS 토픽 정보
		- 모터제어기
			- /xycar_motor
		- 카메라
			- /usb_cam/image_raw
		- IMU 센서
			- /imu
		- 라이다
			- /scan
		- 초음파센서
			- /xycar_ultrasonic
		- Depth 카메라
			- /camera/color/image_raw
			- /camera/depth/image_rect_raw

2. 자이카 노드, 토픽 연결도  
	![image](https://user-images.githubusercontent.com/53277342/156708333-3d651218-c92e-481e-8f00-d04cc34026f2.png)

3. 명령어
	- $ roscd "Package" # Package 경로 이동
	- $ rostopic list	# 토픽 목록 확인
	- $ rostopic info "Topic"	# 토픽 정보 확인
	- $ rostopic type "Topic"	# 토픽 메시지 타입 경로 확인
	- $ rosmsg show xycar_msgs/"Topic"	# 토픽 메시지 타입 확인
	- $ rostopic echo "Topic"	# 토픽 메시지 확인
	- $ rqt_graph	# 노드와 토픽 연결관계 시각적 확인

4. 모터 제어 방법
	- 속도 값 담아서 /xycar_motor 토픽 발행
		- speed: -50 (후진) ~ 0 (중앙) ~ 50 (전진)
	- 조향 값 담아서 /xycar_motor 토픽 발행
		- angle: -50 (좌) ~ 0 (중앙) ~ 50 (우)
	- 발행 주기
		- 0.7초 동안 토픽 없는 경우
			- 속도 0, 조향 각도 0 















