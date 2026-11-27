import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge

import cv2
import numpy as np
from scipy.spatial.transform import Rotation


class ArucoDetectorNode_Realsense(Node):
    def __init__(self):
        super().__init__('aruco_detector_node_realsense')
        self.bridge = CvBridge()

        # 카메라 파라미터 저장
        self.camera_matrix = None
        self.dist_coeffs = None

        # 아루코 설정
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:  # OpenCV 4.7 이상
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.marker_length = 0.06  # m

        # Subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.cam_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/color/camera_info', self.camera_info_callback, 10)

        # Publisher
        self.image_pub = self.create_publisher(Image, '/aruco/image', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/aruco/marker_pose', 10)
        self.detected_pub = self.create_publisher(Bool, '/aruco/detected', 10)

        self.get_logger().info('ArucoDetectorNode start ~~ (Realsense)')

    def camera_info_callback(self, msg):  # CameraInfo를 받으면 실행되는 callback 함수
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            D = np.array(msg.d, dtype=np.float64)
            self.camera_matrix = K
            self.dist_coeffs = D
            self.get_logger().info('CameraInfo OK ~~')

    def image_callback(self, msg):  # Image를 받으면 실행되는 callback 함수
        if self.camera_matrix is None or self.dist_coeffs is None:
            return

        # ROS Image -> OpenCV 이미지
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 영상 중심 표시 (제어 목표 위치)
        h, w = frame.shape[:2]
        cv2.drawMarker(frame, (w // 2, h // 2), (255, 255, 0),
                       cv2.MARKER_CROSS, 20, 1)

        # 아루코 마커 검출
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        detected = False

        if ids is not None and len(ids) > 0:  # 마커 검출에 성공했다면?
            # 마커 3차원 위치 추정
            result = self.my_estimatePoseSingleMarkers(corners, ids)

            if result is not None:
                rvec, tvec, quaternion, idx = result
                detected = True

                # 마커 그리기
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                # 좌표축 그리기 (x: 빨강, y: 초록, z: 파랑)
                cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs,
                                  rvec, tvec, self.marker_length * 0.5)

                tvec = tvec.flatten()
                euler = Rotation.from_quat(quaternion).as_euler('xyz', degrees=True)
                center = corners[idx][0].mean(axis=0)  # 마커 중심 픽셀 좌표

                # 좌표 값을 영상 위에 표시 (cm, deg)
                lines = [
                    f'ID {int(ids[idx][0])}',
                    f'X {tvec[0] * 100:7.1f} cm',
                    f'Y {tvec[1] * 100:7.1f} cm',
                    f'Z {tvec[2] * 100:7.1f} cm',
                    f'Roll  {euler[0]:7.1f} deg',
                    f'Pitch {euler[1]:7.1f} deg',
                    f'Yaw   {euler[2]:7.1f} deg',
                    f'px ({center[0]:.0f}, {center[1]:.0f})',
                ]
                for i, text in enumerate(lines):
                    cv2.putText(frame, text, (10, 25 + i * 22),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

                # 결과 Publish
                pose_msg = PoseStamped()
                pose_msg.header = msg.header  # 카메라 타임스탬프, frame_id 사용
                pose_msg.pose.position.x = float(tvec[0])
                pose_msg.pose.position.y = float(tvec[1])
                pose_msg.pose.position.z = float(tvec[2])
                pose_msg.pose.orientation.x = float(quaternion[0])
                pose_msg.pose.orientation.y = float(quaternion[1])
                pose_msg.pose.orientation.z = float(quaternion[2])
                pose_msg.pose.orientation.w = float(quaternion[3])
                self.pose_pub.publish(pose_msg)

                self.get_logger().info(
                    f'x={tvec[0]:.3f} y={tvec[1]:.3f} z={tvec[2]:.3f}',
                    throttle_duration_sec=0.5)

        if not detected:
            cv2.putText(frame, 'NO MARKER', (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        # 검출 여부 Publish (제어부가 마커 유무를 판단할 수 있도록)
        det_msg = Bool()
        det_msg.data = detected
        self.detected_pub.publish(det_msg)

        # OpenCV -> ROS Image
        out_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        out_msg.header = msg.header
        # 결과 이미지 Publish
        self.image_pub.publish(out_msg)

    def my_estimatePoseSingleMarkers(self, corners, ids):
        # 마커 좌표
        object_points = np.array([
            [-self.marker_length / 2, self.marker_length / 2, 0],   # 좌상단 좌표
            [self.marker_length / 2, self.marker_length / 2, 0],    # 우상단 좌표
            [self.marker_length / 2, -self.marker_length / 2, 0],   # 우하단 좌표
            [-self.marker_length / 2, -self.marker_length / 2, 0]   # 좌하단 좌표
        ], dtype=np.float32)

        if ids is not None:
            for i in range(len(ids)):
                corner = corners[i][0]

                # solvePnP를 사용하여 3차원 위치 추정
                success, rvec, tvec = cv2.solvePnP(
                    object_points, corner, self.camera_matrix, self.dist_coeffs)

                if success:
                    R = Rotation.from_rotvec(rvec.flatten())
                    quaternion = R.as_quat()  # xyzw
                    return rvec, tvec, quaternion, i

        return None


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectorNode_Realsense()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
