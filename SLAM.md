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

4. Cartographer 알고리즘
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

		2. Pose Extrapolator
			- 방법
				- IMU, odom 데이터 이용해 가장 최근 pose가 정해진 데이터로부터 다음 데이터 pose 추정
					- IMU, odom 사용 불가 경우 Real Correlative Scan Matching 사용
				- Translation, Rotation 정보 반환
					- Ceres Scan Matcher의 초기값으로 사용
		3. Ceres Scan Matching
			- 방법
				- 최근 Submap에 대한 라이다 데이터의 pose를 최적화 과정 통해 구함
		4. Motion Filter
			- 사용 이유
				- Pose가 비슷한 데이터의 불필요한 연산량, 제거 목적
			- 방법
				- 최근 데이터의 pose와 가까우면서 일정 시간 이내의 데이터인 경우 제거
		5. Submap Update
			- Pose 정보 가지는 데이터들로 Submap 업데이트
			- Submap은 Probabilty Grid로 제작, Grid는 장애물 존재 확률값 가짐
			- Pose 기준으로 라이다 데이터들 각각 Submap 상에 위치
				- Hit Grid
					- 라이다 데이터와 가장 가까운 Grid
				- Miss Grid
					- Pose와 라이다 데이터를 잇는 직선이 지나는 Grid들
				
5. Carographer 파라미터 튜닝
	- Frontend 파라미터
		- trajectory_builder.lua
			- 기능
				- Submap 생성 과정에서 사용되는 옵션 설정 파일
				- trajectory_builder_2d(3d).lua 파일 참조
			- 파라미터
				- pure_localization_trimmer.max_submaps_to_keep
					- 최근 몇 개의 Submap 저장할지 설정
		- trajectory_builder_2d(3d).lua
			- 기능
				- 2D(3D) SLAM에서 사용하는 파라미터 설정 파일
			- 파라미터
				- use_imu_data
					- 중력 방향 결정할 때 IMU 센서 사용할지 여부 설정
					- 2D SLAM에서는 선택, 3D SLAM에서는 필수
				- min_range (max_range)
					- 사이 거리 값을 가지는 라이다 데이터만 SLAM에 사용
				- min_x (max_z)
					- 3D 라이다에서 수집된 Point Cloud를 2D SLAM에서 사용할 때 Z값 제한두어 포인트 변환
				- missing_data_ray_length
					- max_range보다 큰 값 가지는 라이다 데이터 저장
				- voxel_filter_size
					- Voxel Filter 크기 설정
				- real_time_correlative_scan_matching
					- 현재 Submap과 현재 들어온 라이다 데이터를 대상으로 matching 수행
					- 현재 Pose 기준으로 일정 거리, 각도 내에 있는 비슷한 Scan 데이터 찾음
					- linear_search_window
						- Search 거리 범위 설정
					- angular_search_window
						- Search 각도 범위 설정
				- ceres_scan_matching
					- occupied_space_weight
						- 각각 Scan 데이터의 신뢰도
					- translation_weight
						- Initial Pose의 Translation 신뢰도
					- rotation_weight
						- Initial Pose의 Rotation 신뢰도
					- 신뢰도 높을수록 Scan Matching에 많이 반영됨
				- motion_filter
					- 세 기준 중 하나라도 만족하는 라이다 데이터를 Submap Update에 사용
					- max_time_seconds
						- 시간이 지나서 얻은 데이터만 사용
					- max_distance_meters
						- 특정 거리 이상의 데이터만 사용
					- max_angle_radians
						- 특정 각도 이상의 Heading 차이가 있는 데이터만 사용
				- submaps
					- num_range_data
						- Submap Update에 사용할 라이다 데이터 개수 결정
					- grid_options_2d.resolution
						- Submap의 Resolition, Default 0.05m
					- range_data_inserter.probabilty_range_data_inserter
						- hit_probabilty
							- Submap Update 과정에서 hit grid의 확률 업데이트할 때 사용하는 확률값
						- miss_probabilty
							- Submap Update 과정에서 miss grid의 확률 업데이트할 때 사용하는 확률값
	- Backend 파라미터
		- map_builder_server.lua
			- 기능
				- Remote 서버에서 최적화에 사용되는 파라미터 설정
			- 파라미터
				- use_trajectory_builder_2d(3d)
					- 2D SLAM, 3D SLAM 중 어떤 SLAM 파라미터 사용할 것인지 결정
				- num_background_threads
					- Backend Optimization 수행할 때 사용할 Thread 수 설정
				- pose_graph = POSE_GRAPH
					- "pose_graph.lua" 파일의 POSE_GRAPH 파라미터 저장
		- map_builder.lua
			- 맵 최적화하는 과정에서 옵션 파라미터 설정
		- pose_graph.lua
			- 기능
				- 맵 최적화 알고리즘에 사용되는 파라미터 설정
			- 파라미터
				- POSE_GRAPH
					- optimize_every_n_nodes
						- 최적화 진행 주기 결정
					- constraint_builder
						- sampling_ratio
							- 가까운 노드 중 Loop Closure Detection 위한 Matching에 사용할 비율 결정
						- min_score
							- 기준 이하 노드들은 Matching에서 제외
						- global_localization_min_score
							- 서로 다른 Trajectory 간 기준 이하 노드들은 제외
					- Optimization Problem
						- local_slam_pose_translation_weight, local_slam_pose_rotation_weight
							- Local SLAM에서 계산된 Pose 얼마나 신뢰할지 결정
						- odometry_translation_weight, odometry_rotation_weight
							- Odometry의 Pose 정보 얼마나 신뢰할지 결정
						- max_num_final_iterations
							- Mapping 후 프로세스 종료 전 Pose Optimization 작업 횟수 결정
	- Custom 파라미터
		- 파일 생성
			- Mapping 작업 시 mapping.lua 파일
			- Localization 작업 시 localization.lua 파일
		- 파라미터
			- options
				- 기능
					- 자신의 ROS 환경 세팅 정보 입력
				- 파라미터
					- map_frame
						- Pose의 부모 프레임
					- tracking_frame
						- SLAM 알고리즘이 Tracking 할 프레임
					- pulished_frame
						- Pose의 자식 프레임 결정, Odom 제공하는 경우 odom, 아닌 경우 base_link
					- odom_frame
						- Local SLAM의 결과로 나온 Pose 발행 토픽 이름
					- provide_odom_frame
						- odom_frame 발행 여부 결정
					- publish_frame_projected_to_2d
						- Pose를 2D로 제한, 2D 평면 벗어나는 Pose 제거
					- use_odometry
						- 차량에서 발행하는 Odom Subscribe 해서 SLAM에 사용
					- use_nav_sat
						- GPS 정보인 Fix 토픽 Subscribe 해서 Global SLAM에 사용
					- use_landmarks
						- Landmark 정보 사용 여부 결정
					- num_laser_scan
						- 사용할 라이다 채널 개수
					- publish_to_tf
						- TF Transform 수행
					- publish_tracked_pose
						- 현재 Pose를 geometry_msgs/PoseStamped 타입의 토픽 발행 여부 결정

6. 작업 디렉토리 구조
	- config
		- 파라미터 다음 .lua 파일 저장
	- bags
		- 센서 데이터가 담긴 bag 파일
	- maps
		- Mapping 결과 저장
		- Global Map인 pbstream 파일 복사
	- launch
		- Mapping 작업에 사용하는 launch 파일 저장
		- 설정값 조정하는 파라미터 파일 저장
	
