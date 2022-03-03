# Autonomous Driving

## 1. 자율주행 자동차 기술소개
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

## 2. 자율주행 요소기술
1. 자율주행 프로세스
	1. 인지
	2. 판단
	3. 제어
2. 기술
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
3. 자율주행 통합 플랫폼
	- Autoware



















