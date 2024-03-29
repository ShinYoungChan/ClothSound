# 옷감 시뮬레이션 소리 합성
* 옷감 입자의 물리적 속성을 활용하여 가상 시뮬레이션 장면에 맞는 옷감 소리 합성
* 입자의 곡률과 마찰 계수를 계산하여 소리를 합성하는 방법

[dbpia 논문](https://www.dbpia.co.kr/journal/articleDetail?nodeId=NODE11140498)

# 프로젝트 기간
* 2022.01 ~ 2022.06

# 역할 및 성과
* 팀 구성: 3명
* 역할: 옷감 소리 데이터 추출 및 소리 합성 방법 개발

# 곡률 계산
![image](https://github.com/ShinYoungChan/ClothSound/assets/40080826/06a51f15-d8ce-47fb-98c2-65058e55d4f1)

* 이웃한 정점들의 법선벡터를 통해 곡률을 간단하게 계산하고 곡률의 부호가 이전 프레임에서의 부호와 다를 경우 구겨짐에 대한 에너지를 계산한다.
* 에너지가 높을 수록 진폭이 커지도록 하고 그 반대의 경우는 작아지도록 설정

# 소리 데이터 베이스
![image](https://github.com/ShinYoungChan/ClothSound/assets/40080826/fe1ce6f9-072d-420b-b3de-0d4a6a17a4af)

* 한 방향으로만 사운드를 합성하기 때문에 불연속적으로 끊기는 문제를 완화하기 위해 원형 기반으로 사운드 데이터 순회가 가능하도록 반전 데이터를 이용한다.

# 소리 매칭
![image](https://github.com/ShinYoungChan/ClothSound/assets/40080826/24ba01fb-f198-477a-8ae5-b7be00f0e897)

* 9가지의 클립을 생성했고 진폭의 크기를 0.3, 0.7, 1.0로 복제하여 총 27개의 클립 데이터를 생성하고 이 샘플들을 3 X 9 형태로 저장
* 샘플의 주파수가 높을수록 높은 열, 진폭이 클수록 높은 행에 위치하도록 정렬하여 저장
