import numpy as np
import rclpy
from rclpy.node import Node

from pbvs_control.ik_module import Planar2DOFIK, wrap_to_pi

# ==============================================================================
# [TODO 1] 팀원(인식/시뮬레이션)과 맞춘 메시지 타입 import 및 지정
# 예: from geometry_msgs.msg import PointStamped
# 예: from sensor_msgs.msg import JointState
# 예: from std_msgs.msg import Float64MultiArray
# ==============================================================================
MARKER_MSG_TYPE = None       # TODO
JOINT_STATE_MSG_TYPE = None  # TODO
JOINT_CMD_MSG_TYPE = None    # TODO


class SmoothPBVSControllerNode(Node):
    def __init__(self):
        super().__init__("smooth_pbvs_controller")

        # --- 1. 로봇 기구 파라미터 (실제 URDF 링크 길이에 맞게 수정) ---
        self.L1 = 0.40                  # 링크 1 길이 (m)
        self.L2 = 0.35                  # 링크 2 길이 (m)
        self.camera_yaw_offset = 0.0    # 카메라 장착 각도 오프셋 (rad)

        # --- 2. PBVS 및 부드러운 제어 파라미터 ---
        self.target_dist = 0.30                 # 목표 거리 30cm (m)
        self.kp = 0.25                          # 비례 게인 (0.15 ~ 0.4 추천)
        self.deadband = 0.008                   # 정지 허용 오차 (8mm 이내면 정지)
        self.filter_alpha = 0.3                 # EMA 저역통과 필터 계수 (낮을수록 부드러움)
        self.control_hz = 30.0                  # 제어 루프 주기 (Hz)
        self.max_vel_rad_s = np.deg2rad(45.0)   # 최대 관절 회전 속도 (rad/s)
        self.marker_timeout = 0.5               # 마커 미수신 시 정지 타임아웃 (초)

        # --- 3. [TODO 2] 토픽명 및 조인트 이름 설정 ---
        MARKER_TOPIC = ""       # TODO: 인식 파트 마커 3D 위치 토픽
        JOINT_STATE_TOPIC = ""  # TODO: 현재 조인트 각도 수신 토픽
        JOINT_CMD_TOPIC = ""    # TODO: 조인트 목표 각도 발행 토픽

        self.theta1_name = "theta1"
        self.theta2_name = "theta2"

        # --- 4. 내부 객체 및 상태 변수 ---
        self.ik = Planar2DOFIK(self.L1, self.L2)
        self.max_step_rad = self.max_vel_rad_s / self.control_hz

        self.curr_theta1 = None
        self.curr_theta2 = None
        self.filt_x_cam = None
        self.filt_z_cam = None
        self.last_marker_time = None

        # --- 5. ROS 2 통신 및 타이머 ---
        self.create_subscription(MARKER_MSG_TYPE, MARKER_TOPIC, self.marker_callback, 10)
        self.create_subscription(JOINT_STATE_MSG_TYPE, JOINT_STATE_TOPIC, self.joint_state_callback, 10)
        self.cmd_pub = self.create_publisher(JOINT_CMD_MSG_TYPE, JOINT_CMD_TOPIC, 10)

        self.timer = self.create_timer(1.0 / self.control_hz, self.control_loop)
        self.get_logger().info("Smooth PBVS Controller Node initialized.")

    def joint_state_callback(self, msg):
        """현재 관절 각도(rad) 수신 — 이름 기반 매핑으로 순서 꼬임 방지"""
        # [TODO 3] 사용하는 메시지 구조에 맞게 추출 방식 확인 (JointState 기준 예시)
        name_to_pos = dict(zip(msg.name, msg.position))
        if self.theta1_name in name_to_pos and self.theta2_name in name_to_pos:
            self.curr_theta1 = float(name_to_pos[self.theta1_name])
            self.curr_theta2 = float(name_to_pos[self.theta2_name])

    def marker_callback(self, msg):
        """인식 파트에서 마커의 카메라 기준 순수 3D 위치(m) 수신 및 EMA 필터링"""
        # [TODO 4] 메시지 필드에 맞게 카메라 기준 좌우(x), 깊이(z) 추출
        # 예: raw_x = float(msg.point.x), raw_z = float(msg.point.z)
        raw_x = 0.0  # TODO
        raw_z = 0.0  # TODO

        if self.filt_x_cam is None:
            self.filt_x_cam = raw_x
            self.filt_z_cam = raw_z
        else:
            a = self.filter_alpha
            self.filt_x_cam = a * raw_x + (1.0 - a) * self.filt_x_cam
            self.filt_z_cam = a * raw_z + (1.0 - a) * self.filt_z_cam

        self.last_marker_time = self.get_clock().now()

    def control_loop(self):
        """30Hz 고정 주기 제어 루프"""
        if self.curr_theta1 is None or self.filt_x_cam is None or self.last_marker_time is None:
            return

        # 마커 유실 워치독: 타임아웃 초과 시 이동 중단
        elapsed = (self.get_clock().now() - self.last_marker_time).nanoseconds * 1e-9
        if elapsed > self.marker_timeout:
            return

        # 1. 오차 계산 (목표: 정면 30cm, 좌우 0cm)
        e_fwd = self.filt_z_cam - self.target_dist
        e_lat = -self.filt_x_cam  # OpenCV/ROS Optical 오른쪽(+x) -> 평면 왼쪽(+y) 반전

        # 2. 데드존 이내면 정지 (미세 떨림 방지)
        if abs(e_fwd) < self.deadband and abs(e_lat) < self.deadband:
            return

        # 3. 베이스 좌표계 목표 위치 계산
        curr_x, curr_y, phi = self.ik.forward_kinematics(self.curr_theta1, self.curr_theta2)
        cam_phi = phi + self.camera_yaw_offset

        target_x = curr_x + self.kp * (e_fwd * np.cos(cam_phi) - e_lat * np.sin(cam_phi))
        target_y = curr_y + self.kp * (e_fwd * np.sin(cam_phi) + e_lat * np.cos(cam_phi))

        # 4. 역기구학 계산 및 프레임당 최대 이동 각도 제한
        raw_theta1, raw_theta2 = self.ik.solve(target_x, target_y, self.curr_theta1, self.curr_theta2)
        d_theta1 = float(
            np.clip(wrap_to_pi(raw_theta1 - self.curr_theta1), -self.max_step_rad, self.max_step_rad)
        )
        d_theta2 = float(
            np.clip(wrap_to_pi(raw_theta2 - self.curr_theta2), -self.max_step_rad, self.max_step_rad)
        )

        next_theta1 = wrap_to_pi(self.curr_theta1 + d_theta1)
        next_theta2 = wrap_to_pi(self.curr_theta2 + d_theta2)

        # 5. [TODO 5] 명령 메시지 채우기 및 발행
        cmd = JOINT_CMD_MSG_TYPE()
        # 예 (Float64MultiArray): cmd.data = [next_theta1, next_theta2]
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = SmoothPBVSControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
