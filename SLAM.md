# SLAM

1. SLAM 개요
	1. 정의
		- Simultaneous Localization And Mapping
		- 자신의 위치 파악과 동시에 지도를 만드는 것
	2. 필요성
		- 지도를 만드는 작업 Mapping + 만들어진 지도에서 자신의 위치를 파악하는 Localization 동시에 수행되어야
	3. 종류
		- 사용 센서에 따른 분류
			- 카메라 사용 SLAM (Visual SLAM)
				- 장점
					- 저렴한 비용
					- 방대한 정보
				- 단점
					- 심도 추정 어려움
						- 다른 센서와 융합(센서 퓨전)해서 사용
				- ex) ORB2-SLAM, Pro-SLAM, DSO SLAM
			- 라이다 사용 SLAM
				- 장점
					- 정밀한 측정
						- 포인트 클라우드
					- 심도 측정 가능
				- 단점
					- 포인트 클라우드의 밀도 정교함 낮음
						- 라이다 외 다른 센서 데이터 추가
					- 높은 수준의 처리 능력 필요
				- ex) Cartographer SLAM, Gmapping, Hector SLAM
	4. 흔한 문제점
		- 위치 추정 오차
			- 누적될수록 지도 데이터의 붕괴 및 왜곡 발생
		- 위치 추정 실패 및 지도상 위치 상실
			- 로봇의 이동 특성 반영
			- 복원 알고리즘 사용
			- 센서 융합해 대량의 데이터에 기반한 계산 수행
		- 높은 계산 비용
			- 서로 다른 프로세스 병렬 실행
			- 멀티 코어 CPU, SIMD 계산, 임베디드 GPU 사용
	5. 중요 요소
		- 어떤 센서 사용하는지
		- 특징점 어떻게 정의하는지
		- 특징점 간 어떤 Matching 방법 사용하는지
		- 어떤 최적화 방법 사용해 오류 보정할지
	
2. Cartographer
	1. 개요
		- 구글에서 개발한 SLAM 라이브러리
		- 2D, 3D 환경의 SLAM 실시간 제공
	2. 용어
		- Submap
			- 전체 map의 한 부분
		- Frontend = Local SLAM = Local Trajectory Building
			- 스캔한 라이다 데이터들로 Submap 만드는 과정
		- obal SLAM = Map Building
			- 만들어진 Submap들의 관계 보정해 최종 Global Map 생성하는 과정
	3. 설치
		1. 패키지 정보 업데이트, 패키지 다운로드
			```
			$ sudo apt-get update
			$ sudo apt-get install –y python-wstool python-rosdep ninja-build stow
			```
			
		2. Cartographer 다운로드
			```
			$ mkdir carto_ws && cd carto_ws && wstool init src
			$ wstool merge -t src http://raw.githubusercontent.com/cartographer-project/cartographer_Ros/master/cartographer_ros.rosinstall
			$ wstool update -t src
			```
			
		3. Dependency 설치
			```
			$ rosdep update
			$ rosdep install --from-paths src --ignore-src –rosdistro=${ROS_DISTRO} -y
			```
		
		4. 오픈소스 C++ 라이브러리 abseil 설치
			```
			$ src/cartographer/scripts/install_abseil.sh
			```

		5. 관리자 계정에서 환경 적용, 설치
			```
			$ sudo su -
			$ source /home/USER_NAME/.bashrc && cd /home/USER_NAME/carto_ws/
			$ catkin_make_isolated --install --use-ninja --install-space /opt/ros/melodic/
			```

4. Carographer 알고리즘
	- 전체 알고리즘
		1. Voxel Filter
			- 라이다 포인트 Down Sampling
		2. Pose Extrapolator
			- Ceres Scan Matching 초기값 지정
			- Real Correlative Scan Matching으로 대체 가능
		3. Ceres Scan Matching
			- 최신 Submap에 대해 현재 측정된 라이다 데이터들의 pose(x, y, theta) 구함
		4. Motion Filter
			- 일정 조건 미충족 라이다 데이터들 제거
		5. Submap Update
			- 위 과정 거친 라이다 데이터들로 Submap 갱신

	- Frontend 알고리즘
		1. (Adaptive) Voxel Filter
			- 사용 이유
				- 모든 라이다 데이터들을 Matching에 사용하면 계산량 과다
					- Down Sampling 작업: 포인트의 개수 줄임
			- 용어
				- Voxel
					- 2D에서의 pixel 단위를 3D로 확장
			- 방법
				- 1개 Voxel Grid 내부 라이다 데이터들의 평균점 1개만 사용, 나머지 제거



















