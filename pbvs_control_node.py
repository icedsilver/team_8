import numpy as np
import rclpy
from rclpy.node import Node

from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .ik_module import Planar2DOFIK, wrap_to_pi


class SmoothPBVSControllerNode(Node):
    def __init__(self):
        super().__init__("smooth_pbvs_controller")

        # --- 1. 로봇 기구 파라미터 (URDF 반영) ---
        self.L1 = 0.143                 # joint1 ~ joint2 거리 (m)
        self.L2 =                  # joint2 ~ 카메라 중심 거리 (m, 실측 후 보정)
        self.camera_yaw_offset = 0.0    # 카메라 장착 각도 오프셋 (rad)

        # --- 2. PBVS 및 부드러운 제어 파라미터 ---
        self.target_dist = 0.30                 # 목표 거리 30cm (m)
        self.kp = 0.25                          # 비례 게인
        self.deadband = 0.008                   # 정지 허용 오차 (8mm)
        self.filter_alpha = 0.3                 # EMA 저역통과 필터 계수
        self.control_hz = 30.0                  # 제어 루프 주기 (Hz)
        self.max_vel_rad_s = np.deg2rad(45.0)   # 최대 관절 회전 속도 (rad/s)
        self.marker_timeout = 0.5               # 마커 미수신 시 정지 타임아웃 (초)

        # --- 3. 토픽명 및 조인트 이름 설정 (YAML/URDF 반영) ---
        MARKER_POSE_TOPIC = "/aruco/marker_pose"
        MARKER_DETECTED_TOPIC = "/aruco/detected"
        JOINT_STATE_TOPIC = "/joint_states"
        JOINT_CMD_TOPIC = "/joint_trajectory_controller/joint_trajectory"
        SIM_DELTA_TOPIC = "/sim/joint_delta"

        self.theta1_name = "joint1"
        self.theta2_name = "joint2"

        # --- 4. 내부 객체 및 상태 변수 ---
        self.ik = Planar2DOFIK(self.L1, self.L2)
        self.max_step_rad = self.max_vel_rad_s / self.control_hz
        self.dt = 1.0 / self.control_hz

        self.curr_theta1 = None
        self.curr_theta2 = None
        self.marker_detected = False
        self.filt_x_cam = None
        self.filt_z_cam = None
        self.last_marker_time = None

        # --- 5. ROS 2 통신 및 타이머 ---
        self.create_subscription(PoseStamped, MARKER_POSE_TOPIC, self.marker_callback, 10)
        self.create_subscription(Bool, MARKER_DETECTED_TOPIC, self.marker_detected_callback, 10)
        self.create_subscription(JointState, JOINT_STATE_TOPIC, self.joint_state_callback, 10)

        self.cmd_pub = self.create_publisher(JointTrajectory, JOINT_CMD_TOPIC, 10)
        self.sim_delta_pub = self.create_publisher(Float64MultiArray, SIM_DELTA_TOPIC, 10)

        self.timer = self.create_timer(self.dt, self.control_loop)
        self.get_logger().info("Smooth PBVS Controller Node initialized.")

    def joint_state_callback(self, msg: JointState):
        """현재 관절 각도(rad) 수신 — 이름 기반 매핑"""
        name_to_pos = dict(zip(msg.name, msg.position))
        if self.theta1_name in name_to_pos and self.theta2_name in name_to_pos:
            self.curr_theta1 = float(name_to_pos[self.theta1_name])
            self.curr_theta2 = float(name_to_pos[self.theta2_name])

    def marker_detected_callback(self, msg: Bool):
        """마커 검출 여부 수신 — 마커가 사라지면 필터 초기화"""
        self.marker_detected = bool(msg.data)
        if not self.marker_detected:
            self.filt_x_cam = None
            self.filt_z_cam = None

    def marker_callback(self, msg: PoseStamped):
        """인식 파트에서 마커의 카메라 기준 순수 3D 위치(m) 수신 및 EMA 필터링"""
        raw_x = float(msg.pose.position.x)
        raw_z = float(msg.pose.position.z)

        if self.filt_x_cam is None:
            self.filt_x_cam = raw_x
            self.filt_z_cam = raw_z
        else:
            a = self.filter_alpha
            self.filt_x_cam = a * raw_x + (1.0 - a) * self.filt_x_cam
            self.filt_z_cam = a * raw_z + (1.0 - a) * self.filt_z_cam

        self.last_marker_time = self.get_clock().now()

    def publish_sim_delta(self, d_theta1: float, d_theta2: float):
        """시뮬레이션 파트로 조인트 이동량(rad) 발행"""
        delta_msg = Float64MultiArray()
        delta_msg.data = [float(d_theta1), float(d_theta2)]
        self.sim_delta_pub.publish(delta_msg)

    def publish_joint_trajectory(self, next_theta1: float, next_theta2: float):
        """JointTrajectoryController로 목표 각도 발행"""
        traj = JointTrajectory()
        traj.joint_names = [self.theta1_name, self.theta2_name]

        point = JointTrajectoryPoint()
        point.positions = [float(next_theta1), float(next_theta2)]
        # 다음 제어 주기(약 0.033초) 안에 부드럽게 도달하도록 설정
        point.time_from_start = Duration(sec=0, nanosec=int(self.dt * 1e9))

        traj.points.append(point)
        self.cmd_pub.publish(traj)

    def control_loop(self):
        """30Hz 고정 주기 제어 루프"""
        if (
            not self.marker_detected
            or self.curr_theta1 is None
            or self.filt_x_cam is None
            or self.last_marker_time is None
        ):
            self.publish_sim_delta(0.0, 0.0)
            return

        # 마커 유실 워치독: 타임아웃 초과 시 이동 중단
        elapsed = (self.get_clock().now() - self.last_marker_time).nanoseconds * 1e-9
        if elapsed > self.marker_timeout:
            self.publish_sim_delta(0.0, 0.0)
            return

        # 1. 오차 계산 (목표: 정면 30cm, 좌우 0cm)
        e_fwd = self.filt_z_cam - self.target_dist
        e_lat = -self.filt_x_cam

        # 2. 데드존 이내면 정지 (미세 떨림 방지)
        if abs(e_fwd) < self.deadband and abs(e_lat) < self.deadband:
            self.publish_sim_delta(0.0, 0.0)
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

        # 5. Gazebo JointTrajectoryController로 목표 각도 발행
        self.publish_joint_trajectory(next_theta1, next_theta2)

        # 6. 조인트 이동량 발행 (실시간 시뮬레이션 시각화용)
        self.publish_sim_delta(d_theta1, d_theta2)


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
