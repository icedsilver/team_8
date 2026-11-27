import numpy as np


def wrap_to_pi(angle: float) -> float:
    """각도를 [-pi, pi] 범위로 정규화 (최단 회전 방향 보장)"""
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


class Planar2DOFIK:
    """평면 2자유도(RR) 매니퓰레이터 순기구학 및 역기구학 모듈"""

    def __init__(self, L1: float, L2: float, margin: float = 1e-3):
        self.L1 = L1
        self.L2 = L2
        self.r_min = abs(L1 - L2) + margin
        self.r_max = (L1 + L2) - margin

    def forward_kinematics(self, theta1: float, theta2: float) -> tuple[float, float, float]:
        """현재 관절각(rad) -> 엔드이펙터 위치 및 방향 (x, y, phi)"""
        phi = theta1 + theta2
        x = self.L1 * np.cos(theta1) + self.L2 * np.cos(phi)
        y = self.L1 * np.sin(theta1) + self.L2 * np.sin(phi)
        return x, y, phi

    def clamp_to_workspace(self, x: float, y: float) -> tuple[float, float, float]:
        """목표점이 작업공간 밖이면 도달 가능한 경계점으로 투영"""
        r = float(np.hypot(x, y))
        if r < 1e-9:
            return self.r_min, 0.0, self.r_min
        if r > self.r_max:
            scale = self.r_max / r
            return x * scale, y * scale, self.r_max
        if r < self.r_min:
            scale = self.r_min / r
            return x * scale, y * scale, self.r_min
        return x, y, r

    def solve(
        self, target_x: float, target_y: float, curr_theta1: float, curr_theta2: float
    ) -> tuple[float, float]:
        """Elbow-down / Elbow-up 중 현재 자세에서 덜 움직이는 목표 각도(rad) 반환"""
        x, y, r = self.clamp_to_workspace(target_x, target_y)

        cos_theta2 = float(
            np.clip((r**2 - self.L1**2 - self.L2**2) / (2.0 * self.L1 * self.L2), -1.0, 1.0)
        )
        sin_mag = float(np.sqrt(max(0.0, 1.0 - cos_theta2**2)))
        alpha = float(np.arctan2(y, x))

        candidates = []
        for sign in (1.0, -1.0):  # +1: Elbow-down, -1: Elbow-up
            theta2 = float(np.arctan2(sign * sin_mag, cos_theta2))
            beta = float(np.arctan2(self.L2 * np.sin(theta2), self.L1 + self.L2 * np.cos(theta2)))
            theta1 = wrap_to_pi(alpha - beta)
            theta2 = wrap_to_pi(theta2)

            cost = abs(wrap_to_pi(theta1 - curr_theta1)) + abs(wrap_to_pi(theta2 - curr_theta2))
            candidates.append((cost, theta1, theta2))

        _, best_theta1, best_theta2 = min(candidates, key=lambda c: c[0])
        return best_theta1, best_theta2