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
