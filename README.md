# team_8
마커 인식

터미널 1: 카메라 (D435를 USB 3.x 포트에 연결)
source ~/ros2_ws/install/setup.bash
ros2 run realsense2_camera realsense2_camera_node

터미널 2: 마커 검출 노드
source ~/ros2_ws/install/setup.bash
ros2 run marker_detection detection_realsense

터미널 3: 결과 확인 (선택)
source ~/ros2_ws/install/setup.bash
ros2 run rqt_image_view rqt_image_view


시각화))
터미널 1: rviz,gazebo 실행
ros2 launch twolink_gazebo gazebo.launch.py

터미널 2: 로봇 시각화
ros2 run twolink_control serial_node

터미널 3: 로봇 시각화
ros2 run twolink_gazebo joint_publisher.py


cmakelists.txt 수정)) : ament_package()위에 작성
install(PROGRAMS
  scripts/joint_publisher.py
  scripts/dummy_data_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)
>>명령어 실행
cd ~/ros2_ws

chmod +x src/twolink_gazebo/scripts/joint_publisher.py
chmod +x src/twolink_gazebo/scripts/dummy_data_publisher.py

ros2 run pbvs_control pbvs_control_node.py

-------------------------------------------------------------
source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 topic list

ros2 topic list -t

ros2 topic hz /joint_states
ros2 topic echo /aruco/detected --once

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch twolink_gazebo <런치파일명>.launch.py

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run <인식_패키지명> aruco_detector_node_realsense

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run pbvs_control <실행파일명>

source /opt/ros/$ROS_DISTRO/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
ros2 topic echo /aruco/detected --once


[INFO] [1795770050.549502225] [smooth_pbvs_controller]: e_fwd=-0.098, e_lat=-0.028 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770051.081573158] [smooth_pbvs_controller]: e_fwd=-0.104, e_lat=-0.042 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770051.615451828] [smooth_pbvs_controller]: e_fwd=-0.099, e_lat=-0.030 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770052.216287193] [smooth_pbvs_controller]: e_fwd=-0.102, e_lat=-0.045 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770052.748181404] [smooth_pbvs_controller]: e_fwd=-0.113, e_lat=-0.036 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770053.281623567] [smooth_pbvs_controller]: e_fwd=-0.127, e_lat=-0.057 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)
[INFO] [1795770054.382067160] [smooth_pbvs_controller]: e_fwd=-0.142, e_lat=-0.048 | d1=0.0262, d2=-0.0262 -> next=(0.026, -0.026)

