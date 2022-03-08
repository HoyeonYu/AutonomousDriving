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

## 2. 차량주행 시뮬레이터 설계
1. 시뮬레이터 UI
	- 스크린 표시
	- 키 입력에 따른 작동
		- 차량의 움직임 정의

2. 차량 주행 시뮬레이터 설계
	- 배경지식
		![image](https://user-images.githubusercontent.com/53277342/156953680-9934998c-2c75-4f05-9c25-123e2ebecd73.png)
		- 선속도
			- 직선 방향으로 움직이는 속도
			- 선속도 = 이동거리 / 이동시간 (v = l/t)
		- 각속도
			- 회전축으로부터 돌아가는 속도
			- 각속도 = 이동각도 / 이동시간 (ω = θ/t)
		- 선속도 = 회전 반지금 X 각속도
		- Ackerman Steering  
![image](https://user-images.githubusercontent.com/53277342/156954537-a36686d7-31c9-4289-b3fa-75a7847c0676.png)  
			- R: 회전반경, L: 축거, t: 윤거, 회전반경은 매우 크다고 가정
			![image](https://user-images.githubusercontent.com/53277342/156955509-a648b4c5-70fa-4d7a-a2a5-3a7b394ca2ed.png)
			- Ackerman Angle
				- 두 바퀴의 각도의 산술평균, 두 바퀴의 중점 각도
				![image](https://user-images.githubusercontent.com/53277342/156955773-004c368f-b3bc-4014-a493-e75fe2879101.png)
				- Neutral Steer
					- 기준으로 UnderSteer, OverSteer
					![image](https://user-images.githubusercontent.com/53277342/156955821-99548c5e-509b-4f2a-bdbd-32d0dec20a89.png)
		- 회전반경
			- 핸들을 돌렸을 때 얼마나 큰 원을 그리며 회전하는지
			- 회전반경 = 휠베이스 / tanθ (r = L/tanθ)
			- 휠베이스
				- 앞바퀴 축 ~ 뒷바퀴 축 거리
				- 상수값  
			![image](https://user-images.githubusercontent.com/53277342/156957369-a2fcd756-e777-4d38-80a8-99e1e380aff9.png)
		

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

5. 기능
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

## 4. 센서
1. 카메라 센서
2. IMU 센서
	- Inertial Measurement Unit, 관성 측정 장치
	- 가속도계, 회전속도계, 자력계 이용해 물체에 가해지는 힘, 회전 각속도 측정 장치
	- 가속도 센서 (Accelerometer)
		- 직선방향 가속도(움직임) 감지, 중력가속도 감지
		- MEMS (Micro Electro Mechanical Systems) 기술로 만들어짐
		- 반도체 칩 내부의 입체적 구조물로 감지
	- 자이로 센서 (Gyroscope)
		- 각속도 감지
		- MEMS 기술로 개발
		- X, Y, Z축 기준 회전 움직임 감지
	- 지자기 센서 (Magnetomter)
		- N극 방향 감지, 동서남북 방위각 감지
		- 9축 IMU 센서에 포함
	- 세 방향의 축  
	![image](https://user-images.githubusercontent.com/53277342/156968389-55361064-b6a8-48f1-8634-5a22834c129c.png)  
	![image](https://user-images.githubusercontent.com/53277342/156968665-fc6e8709-8167-4520-b8a9-b0b099aef3ab.png)  
		- Yaw
			- 차량의 좌,우회전 감지
		- Roll
			- 차량이 코너링할 때 옆으로 기울어짐 감지
		- Pitch
			- 차량이 언덕 오르내림 감지

3. 라이다 센서
	- LIDAR vs RADAR
		- LIDAR (Light Imaging Detection And Ranging)
			- 레이저 신호의 반사파 이용
			- 짧은 주파수 이용
			- 작은 물체, 재질 감지 가능
			- 3D 단색 이미지 구성 가능
		- RADAR (RAdio Detection And Ranging)
			- 전파 신호의 반사파 이용
			- 속도 감지 가능
			- 긴 작동거리
			- 빛 환경 상관없이 작동

4. 초음파 센서
	- 초음파
		- 인간이 들을 수 있는 가청 주파수 대역(20Hz ~ 20kHz)보다 높은 진동수로 발생하는 파동
	- 시그널
		- Vcc
			- 부품 전력 공급 (DC 5V)
		- GND
			- 그라운드 연결
		- Trig
			- 센서 동작시키기 위한 트리거 시그널 (입력)
		- Echo
			- 거리 측정 결과 전달 시그널 (출력)
	- 동작 시나리오
		1. 시작
		2. 초음파 발사와 수신
		3. 시간차 출력

## 5. 자이카 ROS 패키지
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
		- 차선 자율주행
			- /auto_drive
		- 사람 자율주행
			- /human_track
		- 장애물 회피주행
			- /sensor_drive
		- YOLO 객체인식
			- /darknet_ros
		- 안드로이드 앱 수동주행
			- /joystick_cam
			- /android/virtual_speed_joystick
			- /android/virtual_steering_joystick
			- /android/camera_view
		- 뷰어
			- /image_view
			- RVIZ 
	- ROS 토픽 정보
		- 모터제어기
			- /xycar_motor
		- 카메라
			- /usb_cam/image_raw
			- /usb_cam/image_raw_compressed
		- IMU 센서
			- /imu
		- 라이다
			- /scan
		- 초음파센서
			- /xycar_ultrasonic
		- Depth 카메라
			- /camera/color/image_raw
			- /camera/depth/image_rect_raw
		- 조이스틱 조종기
			- /android_motor_speed
			- /android_motor_steering

2. 자이카 노드, 토픽 연결도  
	![image](https://user-images.githubusercontent.com/53277342/156972650-e85f684f-eb8f-4985-aa90-7895633af419.png)

3. 명령어
	```
	$ roscd "Package" # Package 경로 이동
	$ rostopic list	# 토픽 목록 확인
	$ rostopic info "Topic"	# 토픽 정보 확인
	$ rostopic type "Topic"	# 토픽 메시지 타입 경로 확인
	$ rosmsg show xycar_msgs/"Topic"	# 토픽 메시지 타입 확인
	$ rostopic echo "Topic"	# 토픽 메시지 확인
	$ rqt_graph	# 노드와 토픽 연결관계 시각적 확인
	```

4. 모터 제어 방법
	- 속도 값 담아서 /xycar_motor 토픽 발행
		- speed: -50(후진) ~ 0(중앙) ~ 50(전진)
	- 조향 값 담아서 /xycar_motor 토픽 발행
		- angle: -50(좌) ~ 0(중앙) ~ 50(우)
	- 발행 주기
		- 0.7초 동안 토픽 없는 경우
			- 속도 0, 조향 각도 0 

## 6. 필터
- 재귀 필터
	- 기존에 계산해둔 결과값을 새로운 데이터 계산에 사용
	- 누적된 과거 데이터와 현재 측정치로 현 상태 추정
	- 평균 필터
		- 평균 계산에 사용
		- ex) 센서 초기화에 사용
		- 데이터 개수, 이전 평균값만 알면 빠르게 계산 가능
		- 데이터 실시간 처리 적합
		- 시간에 따라 변하는 물리량 부적합
	- 이동평균 필터
		- 전체 측정 데이터 사용 X, 지정된 개수의 최근 측정값만 사용해 평균 계산
		- ex) 증권가 주가 추이
		- 측정 데이터 잡음 제거 유용
		- 이전 데이터 가지고 있어야하므로 데이터 개수에 따라 시간 지연 발생








