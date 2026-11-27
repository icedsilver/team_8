import math
import rclpy
from rclpy.node import Node
import serial
from trajectory_msgs.msg import JointTrajectory


class HardwareMotorDriverNode(Node):
    def __init__(self):
        super().__init__("hardware_motor_driver")

        # --- 시리얼 및 토픽 설정 (포트/토픽 필요 시 수정) ---
        self.declare_parameter("port", "")  # 예: "/dev/ttyACM0" 또는 "/dev/ttyUSB0"
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("cmd_topic", "/joint_trajectory_controller/joint_trajectory")
        self.declare_parameter("joint1_name", "joint1")
        self.declare_parameter("joint2_name", "joint2")
        self.declare_parameter("min_angle_rad", -math.pi)
        self.declare_parameter("max_angle_rad", math.pi)
        self.declare_parameter("tx_hz", 30.0)

        self.port = str(self.get_parameter("port").value).strip()
        self.baudrate = int(self.get_parameter("baudrate").value)
        cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.j1_name = str(self.get_parameter("joint1_name").value)
        self.j2_name = str(self.get_parameter("joint2_name").value)
        self.min_angle_rad = float(self.get_parameter("min_angle_rad").value)
        self.max_angle_rad = float(self.get_parameter("max_angle_rad").value)
        tx_hz = float(self.get_parameter("tx_hz").value)

        if not self.port:
            raise ValueError("시리얼 포트('port' 파라미터)가 비어 있습니다. 예: -p port:=/dev/ttyACM0")

        self.target_j1_rad = 0.0
        self.target_j2_rad = 0.0
        self.has_command = False
        self.ser = None

        self._open_serial()

        self.create_subscription(JointTrajectory, cmd_topic, self.trajectory_callback, 10)
        self.timer = self.create_timer(1.0 / tx_hz, self.tx_loop)
        self.get_logger().info(f"Motor driver started on {self.port} @ {self.baudrate} -> {cmd_topic}")

    def _open_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.05)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 포트 열기 실패 ({self.port}): {e}")
            raise

    def _clamp_rad(self, angle_rad: float) -> float:
        return max(self.min_angle_rad, min(self.max_angle_rad, float(angle_rad)))

    def trajectory_callback(self, msg: JointTrajectory):
        """JointTrajectory 메시지에서 joint1, joint2 목표 각도(rad) 추출"""
        if not msg.points:
            return

        point = msg.points[-1]
        joint_index = {name: idx for idx, name in enumerate(msg.joint_names)}

        if self.j1_name not in joint_index or self.j2_name not in joint_index:
            self.get_logger().warn(
                f"Joint names '{self.j1_name}', '{self.j2_name}' not found in {msg.joint_names}",
                throttle_duration_sec=2.0,
            )
            return

        idx1 = joint_index[self.j1_name]
        idx2 = joint_index[self.j2_name]

        if idx1 >= len(point.positions) or idx2 >= len(point.positions):
            return

        self.target_j1_rad = self._clamp_rad(point.positions[idx1])
        self.target_j2_rad = self._clamp_rad(point.positions[idx2])
        self.has_command = True

    def encode_serial_frame(self, j1_rad: float, j2_rad: float) -> bytes:
        """
        [공란] 실제 MCU/모터 드라이버 프로토콜에 맞게 변환하여 bytes 반환
        j1_rad, j2_rad: 안전 범위 제한이 끝난 목표 관절 각도(rad)
        """
        # TODO: MCU가 요구하는 단위(rad, deg, PWM/us, step 등)와 프레임 포맷으로 작성
        raise NotImplementedError("encode_serial_frame()에 실제 시리얼 프로토콜 포맷을 구현하세요.")

    def tx_loop(self):
        """고정 주기로 시리얼 프레임 송신"""
        if not self.has_command or self.ser is None:
            return

        try:
            payload = self.encode_serial_frame(self.target_j1_rad, self.target_j2_rad)
            if payload:
                self.ser.write(payload)
        except NotImplementedError as e:
            self.get_logger().error(str(e), throttle_duration_sec=5.0)
        except serial.SerialException as e:
            self.get_logger().error(f"시리얼 송신 오류: {e}", throttle_duration_sec=1.0)

    def destroy_node(self):
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareMotorDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ValueError, serial.SerialException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()