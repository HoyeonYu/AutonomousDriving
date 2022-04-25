# Machine Learning

## 1. 딥러닝의 기초
1. AI, ML, DL
	- AI
		- 인간의 학습, 추론, 지각 능력 및 자연언어의 이해 능력을 컴퓨터 프로그램으로 실현한 기술
	- ML
		- 인간의 학습 능력을 컴퓨터에서 실현한 기술
		- 학습 방법
			- Supervised Learning
				- 정답을 아는 데이터 이용해 학습 수행
			- Unsupervised Learning
				- 정답 모르고 데이터 특징만을 이용한 학습 수행
			- Reinforcement Learning
				- 보상 통해 학습 수행
	- DL
		- 다층 구조 형태 신경망을 기반 다량의 데이터로부터 높은 수준의 추상화 모델 구축한 기술
		- 성능 향상 배경
			- 알고리즘 발전
			- GPU 성능 향상
			- 빅데이터 구축
			
## 2. 딥러닝
- ANN (Artificial Neural Network)
	- 구조
		- Input Layer
		- Hidden Layer
		- Output Layer
	- 학습 과정
		1. Forward Propagation: 입력에 따른 결과 도출
		2. Back Propagation: 이미 알고 있는 정답과 비교해 네트워크(가중치, 편향) 학습
			- 비교 방법: Loss Function
	- 문제점
		- 입력값의 일부만 변경되어도 다른 입력값으로 인식
		- 모든 데이터에 대응하기 위해 많은 데이터 필요
		- 시간 소요, 한정적 성능
- CNN (Convolutional Neural Network)
	- Filter 이용해 합성곱 연산 수행, 특징 추출
	- 용어
		- Channel: RGB / Gray
		- Kernel Size: Convolution 하는 필터 크기
		- Stride: 필터가 한번에 이동하는 정도
		- Padding: Convolition Output 크기 조절 위해 바깥에 테두리 쳐주는 기법
		- Pooling: 특정 영역 내부 정보 압축하는 방식
- Q-Learning
	- Q Value
		- 특정 state에서 긱 action을 취했을 때 미래에 받을 reward의 합
	- Q-Table
		- 여러 state와 action에 대한 Q값을 저장한 표
		- 표에서 가장 큰 Q값을 찾아 행동
		- 학습 과정
			- Update
				- 현재 알고있는 정보: state, action, next state, reward, terminal
				- 앞으로 받을 reward 합
					- 현재 state의 reward + 다음 state의 q값
				- Target = max(앞으로 받을 reward 합)
					- 현재 state의 reward + γ * max(다음 state의 q값)
					- γ : 미래 예측 reward 고려 비중 (0~1)
				- Q-value = (1-α) * Q-value + α * Target (α: learning rate)




