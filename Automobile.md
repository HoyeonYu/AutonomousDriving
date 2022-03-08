# Automobile

[**1. 자동차 이론**](#1-자동차-이론)  
[**2. 센서**](#2-센서)  
[**3. 필터**](#3-필터)  
[**4. 자율주행**](#4-자율주행)  

## 1. 자동차 이론
1. 속도
	![image](https://user-images.githubusercontent.com/53277342/156953680-9934998c-2c75-4f05-9c25-123e2ebecd73.png)  
	- 선속도
		- 직선 방향으로 움직이는 속도
		- 선속도 = 이동거리 / 이동시간 (v = l/t)
	- 각속도
		- 회전축으로부터 돌아가는 속도
		- 각속도 = 이동각도 / 이동시간 (ω = θ/t)
		- 선속도 = 회전 반지금 X 각속도

2. Ackerman Steering  
	![image](https://user-images.githubusercontent.com/53277342/156954537-a36686d7-31c9-4289-b3fa-75a7847c0676.png)    
	- R: 회전반경, L: 축거, t: 윤거, 회전반경은 매우 크다고 가정  
	![image](https://user-images.githubusercontent.com/53277342/156955509-a648b4c5-70fa-4d7a-a2a5-3a7b394ca2ed.png)  
	- Ackerman Angle  
		- 두 바퀴의 각도의 산술평균, 두 바퀴의 중점 각도  
		- Neutral Steer  
			![image](https://user-images.githubusercontent.com/53277342/156955773-004c368f-b3bc-4014-a493-e75fe2879101.png)  
			- 기준으로 UnderSteer, OverSteer  
		
3. 회전반경
	![image](https://user-images.githubusercontent.com/53277342/156955821-99548c5e-509b-4f2a-bdbd-32d0dec20a89.png)  
	- 핸들을 돌렸을 때 얼마나 큰 원을 그리며 회전하는지
	- 회전반경 = 휠베이스 / tanθ (r = L/tanθ)
	- 휠베이스
		- 앞바퀴 축 ~ 뒷바퀴 축 거리
		- 상수값  
		![image](https://user-images.githubusercontent.com/53277342/156957369-a2fcd756-e777-4d38-80a8-99e1e380aff9.png)  
      
## 2. 센서
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

## 3. 필터
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
		- 가중 이동평균 필터
			- 최신 데이터에 산술 방식으로 가중치를 더 주는 방식

## 4. 자율주행
1. 자율주행 6단계
	1. Level 0
		- 운전자가 차량 제어 전부 수행
	2. Level 1
		- 운전자가 직접 운전, 특정 주행 모드에서 시스템 조향 or 감,가속
	3. Level 2
		- 운전자가 직접 운전, 특정 주행 모드에서 시스템 조향 and 감,가속
	4. Level 3
		- 특정 주행 모드에서 시스템 차량 제어 전부 수행, 운전자는 시스템 요청시에만 개입
	5. Level 4
		- 특정 주행 모드에서 시스템 차량 제어 전부 수행, 운전자 개입 불필요
	6. Level 5
		- 모든 주행 상황에서 시스템 차량 제어 전부 수행

2. 자율주행 요소기술
	- 자율주행 프로세스
		- 인지
		- 판단
		- 제어
	- 기술
		- 고정밀 지도
			- 백터맵
			- 포인트맵
		- Localization 기술
			- 정밀지도와 연동해 차량의 현재 위치 파악
		- Global Path Planning
			- 목적지까지의 경로 찾기
		- Object Detection
			- 주변 상황 인식
		- Object Tracking
			- 각 오브젝트에 고유 ID 부여해 추적, 예상되는 주행 경로 예측
		- Local Path Planning
			- 다음 이동할 곳으로 경로 찾기
		- Behavior Selecetor
			- 행위 결정
				- 운전 의사 결정, 방법, 성향 고려
		- Vehicle Control
			- 주행 제어
				- 원하는 대로 차량 움직이게 함
