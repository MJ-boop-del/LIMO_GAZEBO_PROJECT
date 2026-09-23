# LIMO Ackermann 시뮬레이션

## 실행

Ubuntu 22.04, ROS 2 Humble, Gazebo Classic에서 실행한다.

```bash
cd /home/park/Documents/ws_limo_humble
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select limo_car
source install/setup.bash
ros2 launch limo_car ackermann_gazebo.launch.py
```

다른 터미널에서 키보드 조작을 실행한다.

```bash
source /opt/ros/humble/setup.bash
source /home/park/Documents/ws_limo_humble/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.1 -p turn:=0.3
```

`/cmd_vel`은 표준 Twist 의미를 사용한다. `linear.x`는 m/s, `angular.z`는 yaw 각속도
rad/s다. 제자리 회전은 Ackermann 차량에서 지원하지 않는다.

## 좌측 편향의 원인

수정 전 `gazebo/ackermann.xacro`는 `libgazebo_ros_planar_move.so`를 사용했다.
이 플러그인은 앞바퀴 조향과 뒤바퀴 구동을 계산하지 않고 Twist를 차체 모델에 직접 설정한다.
그 상태에서 앞 조향 조인트가 물리적으로 자유롭게 움직였고 바퀴와 바닥 접촉력이 좌우
비대칭으로 작용했다.

기록에는 `linear.x=0.1`, `angular.z=0`인데도 오른쪽 앞 조향각이 약 0.03도에서
2.85도까지 누적됐고, 차체와 IMU yaw가 0도에서 9.42도까지 변했다. 횡방향 이동은
약 0.21m였다. `/odom`과 Gazebo ground-truth pose의 yaw가 일치했으므로 odom 변환 문제가
아니라 시뮬레이션 차체가 실제로 회전한 문제였다.

## 수정 내용

### 실제 Ackermann 구동기

`libgazebo_ros_planar_move.so`를 `libgazebo_ros_ackermann_drive.so`로 교체했다.
뒤 두 바퀴를 구동하고 앞 두 조향 조인트를 좌우 PID로 제어한다. `/odom`은 실제 모델
pose와 속도로 생성되며 발행자는 `limo_ackermann_controller` 하나다.

### 명령 변환

Ackermann 플러그인의 `angular.z`는 yaw 각속도가 아니라 조향각이다.
`scripts/ackermann_twist_adapter.py`가 표준 명령을 아래 식으로 변환한다.

```text
steering_angle = atan(wheelbase * yaw_rate / |speed|)
```

후진 부호와 플러그인이 오른쪽 뒤 바퀴 속도를 기준으로 구동하는 특성도 보정한다.
플러그인 입력은 `/ackermann_controller/cmd_vel`로 분리했다.

### URDF와 접촉 물리

- 바퀴 inertial 원점·축을 cylinder collision의 중심·축과 일치시켰다.
- 차체 box 관성식의 X/Y 항을 올바른 치수에 맞췄다.
- 좌우 조향 조인트의 damping, friction, velocity limit을 동일하게 설정했다.
- 네 바퀴의 마찰·접촉 강성·감쇠를 동일하게 설정해 4ms 물리 스텝의 접촉 떨림을 줄였다.
- 안쪽 바퀴가 30도 관절 한계를 넘지 않도록 중앙 조향각을 0.42rad로 제한했다.

### TF와 조인트 상태

실제 조향·바퀴 joint state를 발행해 robot_state_publisher가 올바른 TF를 계산한다.
odom child frame은 `base_link`다.

## 검증 결과

분리된 Gazebo에서 자동 검증했다.

- 15초 직진: 약 1.50m 이동, 횡방향 오차 약 0.17mm, yaw 변화 약 0.011도
- 정지 후 재직진: 횡방향 오차 약 0.12mm, yaw 변화 약 0.010도
- 좌·우 선회: 명령 ±0.1rad/s에 대해 실제 yaw 각속도 약 +0.099/-0.100rad/s
- 선회 후 조향각: 좌우 모두 0.005도 이내로 복귀
- 후진 직진: 횡방향 오차 약 0.20mm, yaw 변화 약 0.016도
- 최대 조향: 안쪽 바퀴 약 29.1도로 30도 한계 이내

## Start line과 초기 자세

`worlds/roboracer/ifac_roboracer.world`에는 충돌이 없는 F1 스타일의 검정·흰색 반복선을 추가했다.
선은 `x=-0.2`, `y=-0.8~3m`의 +Y 방향으로 놓이며 Gazebo 바닥 위에만 보인다.
`launch/roboracer_sim.launch.py`의 기본 spawn 위치는 선 위쪽 한 칸인 `(x,y)=(0,1)m`이고,
초기 yaw는 `π` rad(180도)다. 따라서 기존 +X 방향과 반대로 -X를 향해 시작한다.
필요하면 `spawn_x`, `spawn_y`, `spawn_yaw` launch 인자로 덮어쓸 수 있다.

수정 후에는 반드시 빌드하고 새 터미널에서 `source install/setup.bash`를 실행한다.

관련 파일: `gazebo/ackermann.xacro`, `scripts/ackermann_twist_adapter.py`,
`gazebo/ackermann_with_sensor.xacro`, `launch/ackermann.launch.py`,
`launch/ackermann_gazebo.launch.py`.
