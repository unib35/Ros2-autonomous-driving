# 🚗 ros2-autonomous-track-driving

동의대학교 컴퓨터소프트웨어공학과 2023년 2학기 객체지향모델링 팀프로젝트  
**로봇 자율 주행 시험 (Autonomous Track Driving Simulation using ROS2)**  

---

## 📌 프로젝트 개요

본 프로젝트는 ROS2 및 Gazebo 시뮬레이션 환경에서 자율주행 로봇의 주행 시험을 수행하는 것을 목표로 합니다.  
다양한 센서를 활용하여 주변 환경을 인식하고, 경로를 예측하며, 장애물을 회피하는 기능을 구현하였습니다.

### 🎯 프로젝트 목표
- 자율주행 차량의 기본적인 주행 기능 구현
- 정지선 감지 및 준수
- 장애물 및 보행자 회피
- 언덕 구간에서의 주행 테스트
- Gazebo 시뮬레이션을 활용한 안전한 주행 환경 검증

---

## 🛠 개발 환경
- **운영체제**: Ubuntu 20.04.5 LTS
- **프로그래밍 언어**: Python
- **ROS 버전**: ROS2
- **시뮬레이션 환경**: Gazebo
- **IDE**: PyCharm 2022.2.1 (Community Edition)

---

## 🚗 주요 기능

### 1. 차선 감지 및 주행
- 카메라를 활용하여 차선을 감지하고, 차선을 따라 주행하는 알고리즘 구현.
- 차선 벗어남 감지 및 시간 페널티 부여.

### 2. 정지선 준수
- 정지선 감지 기능을 적용하여, 차량이 정지선 앞에서 일정 시간 정지 후 출발.
- 정지선 준수 여부에 따라 시간 페널티 적용.

### 3. 장애물 감지 및 회피
- LiDAR 및 카메라를 활용하여 장애물을 감지하고 회피 주행.
- 움직이는 장애물 및 보행자와의 충돌 방지 기능 구현.

### 4. 언덕 구간 주행
- 경사로 모델링 및 차량의 가속/감속 로직 적용.
- 정지선에서 일정 시간 멈춘 후 다시 출발하는 기능 포함.

---

## 📹 시연 영상

<p align="center">
  <a href="https://youtu.be/jxftKfr1ns0">
    <img src="https://img.youtube.com/vi/jxftKfr1ns0/0.jpg" width="45%">
  </a>
  <a href="https://youtu.be/iU8S7i99wnU">
    <img src="https://img.youtube.com/vi/iU8S7i99wnU/0.jpg" width="45%">
  </a>
</p>
<p align="center"><b>PR001 - 기본 주행 영상 &nbsp;&nbsp;&nbsp;&nbsp; PR002 - 장애물 회피 주행</b></p>

## 🚀 실행 방법

### 1. 환경 설정
```bash
# ROS2 설치 및 환경 설정
source /opt/ros/foxy/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-repository-url/ros2-autonomous-track-driving.git
cd ..
colcon build
source install/setup.bash
```

### 2. 시뮬레이션 실행
```bash
ros2 launch autonomous_track_driving simulation.launch.py
```

---

## 📝 팀 구성원

- 이종민 (20193174)
- 김민곤 (20193172)
- 조봉균 (20193182)
- 김소희 (20203164)
- 장효은 (20212986)

---

## 📌 참고 문서

- [ROS2 공식 문서](https://docs.ros.org/en/foxy/)
- [Gazebo 공식 문서](http://gazebosim.org/tutorials)
