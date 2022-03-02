# ROS_Study

## 1. 로봇 소프트웨어 플랫폼
1. 주요 로봇 운영체제
	- Open Source : ROS
	- Closed Source : NAOqi
	- Galapagos: opros, RT middleware

2. 로봇 소프트웨어 플랫폼 효과
	- 하드웨어 플랫폼과 소프트웨어 플랫폼간 인터페이스 확립
	- 모듈형 하드웨어 플랫폼 확산
	- 소프트웨어 인력 진입
	- 유저에게 제공할 서비스 집중
	- 로봇 개발 속도 향상

## 2. 로봇 운영체제 ROS
1. ROS란?
	- 로봇 소프트웨어를 개발하기 위한 소프트웨어 프레임워크
		- 노드간 메시지 교환 방법으로 프로그램 나눠 공동 개발 가능
		- Modeling, Sensing, Navigation, Manipulation 기능 지원
		- 로보틱스 생태계 생성
	- 메타 운영체제 (Meta-Operating System), 미들웨어 (MiddleWare)
		- 운영체제를 이용해 로봇 응용 소프트웨어 개발을 위한 기능 라이브러리 형태로 제공
		- 이기종 디바이스간 통신 지원
		- 소프트웨어 모듈 + 라이브러리 집합 + 도구 집합
	- 자율주행 자동차를 쉽고 빠르게 만들 수 있는 도구

2. ROS 특징
	- 통신 인프라
		- 메시지 파싱 기능
		- 메시지 기록 및 재생
		- 메시지 사용으로 인한 다양한 프로그래밍 언어 사용 가능
		- 분산 매개 변수 시스템
	- 로봇 관련 다양한 기능
		- 로봇에 대한 표준 메시지 정의
		- 로봇 기하학 라이브러리
		- 로봇 기술 언어
		- 진단 시스템
		- 센싱 / 인식
		- 네비게이션
		- 메니퓰레이션
	- 다양한 개발 도구
		- Command-Line Tools
		- RViz
			- 다양한 센서 값 3D 시각화 제공
		- RQT
			- Qt 기반 GUI 응용 개발 도구
			- 노드 정보 그래프로 표현
		- Gazebo
			- 물리 엔진 기반 3차원 시뮬레이터
	- 로봇 SW를 만들기 위한 코드 재사용 용이

3. ROS 기본 용어
	- Master
		- 서로 다른 노드 사이의 통신을 총괄 관리, ROS Core
	- Node
		- 실행 가능한 최소의 단위, 프로세스
		- ROS에서 발생하는 통신의 주체
	- Topic
		- ROS 노드들이 관심을 갖는 이야기
	- Publisher
		- 특정 토픽에 메시지 담아 외부로 송신하는 노드
	- Subscriber
		- 특정 토픽에 담겨진 메시지 수신하는 노드
	- Package
		- 하나 이상의 노드와 노드의 실행을 위한 정보를 묶어놓은 단위
		- 노드, 라이브러리, 데이터, 파라미터 포함

4. ROS 노드간 통신
	- 통신 과정
		1. 마스터(roscore) 실행
		2. 구독자(subscriber) 구동
			- 특정 토픽에 발행되는 메시지 수신 요청
		3. 발행자(publisher) 구동
			- 메시지 발행 의사 전달
		4. 노드 정보 전달
			- 마스터가 발행자 정보 구독자에게 전달
		5. 노드간 접속 요청
			- 구독자가 발행자에게 TCPROS 접속 요청
		6. 노드간 접속 요청에 대한 응답
			- 발행자가 자신의 TCPROS URI 전송해 응답
		7. TCPROS 접속
			- 발행자, 구독자 사이 Socket 연결
		8. 메시지 전송
			- 발행자가 구독자에게 메시지 전송
		9. 메시지 전송 반복
			- 접속 한번 이뤄지면 별도 절차 없이 메시지 송수신 지속
	- 통신 방식
		- Topic 방식
			- 일방적이고 지속적인 메시지 전송
			- 1:1, 1:N, N:N 통신 가능
		- Service 방식
			- 서버가 제공하는 서비스에 클라이언트 요청 보내고 응답 받는 방식
			- 양방향 통신, 일회성 메시지 송수신

5. ROS 주요 명령어
	- roscore
		- ROS 기본 시스템 구동 위해 필요한 프로그램들 실행
	- rosrun "packageName" "nodeName"
		- 패키지에 있는 노드 선택 실행
	- rosnode "indo"
		- 노드의 정보 표시
	- rostopic "option"
		- 토픽의 정보 표시
	- roslaunch "packageName" "fileLaunch"
		- 파라미터 값과 함께 노드 실행
	- rospack list
		- 어떤 패키지들이 있는지 나열
	- rospack find "packageName"
		- 이름 이용해 패키지 검색
	- roscd "locationName"
		- ros 패키지 디렉토리로 이동
	- rosls "locationName"
		- 리눅스 ls와 유사
	- rosed "fileName"
		- 에디터로 파일 편집
	- catkin_create_pkg "packageName" "해당 패키지가 의존하는 다른 패키지"
		- 패키지 만들기
	- catkin_make
		- 패키지 빌드

## 3. ROS 프로그래밍 기초
1. ROS 패키지 만들기
	- 프로그램 실행 권한
		- chmod +x "fileName.py"

2. launch 파일 작성
	- *.launch 파일 내용에 따라 여러 노드들 한꺼번에 실행 가능
	- roslaunch "packageName" "실행시킬 launch 파일 이름"
	- <launch>
			<node pkg = "packageName" type = "fileName" name = "nodeName" />
			< ... />
	   </launch>
	- 실행시킬 노드 정보 XML 형식으로 기록되어있음

3. 태그
	- node 태그
		- 실행할 노드 정보를 입력할 때 사용되는 태그
		- <node pkg="packageName" type="노드가 포함된 소스파일 명" name:"노드 이름" />

	- include 태그
		- 다른 launch 파일 불러오고 싶을 때 사용하는 태그
		- <include file = "같이 실행할 *.launch 파일 경로" />
	
	- param 태그
		- ros 파라미터 서버에 변수 등록, 그 변수에 값 설정하기 위한 태그
		- <param name = "변수 이름", type = "변수 타입", value = "변수 값" />
		- private parameter는 앞에 ~ 붙임
	
## 4. ROS 노드 통신 프로그래밍	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	
	





