import math

from robot.config_xlerobot import XLerobotConfig
from robot.xlerobot import XLerobot

HEAD_MOTOR_MAP = {
    "head_motor_1": "head_motor_1",
    "head_motor_2": "head_motor_2",
}

LEFT_ARM_JOINTS = [
    "left_arm_shoulder_pan",
    "left_arm_shoulder_lift",
    "left_arm_elbow_flex",
    "left_arm_wrist_flex",
    "left_arm_wrist_roll",
    "left_arm_gripper",
]

RIGHT_ARM_JOINTS = [
    "right_arm_shoulder_pan",
    "right_arm_shoulder_lift",
    "right_arm_elbow_flex",
    "right_arm_wrist_flex",
    "right_arm_wrist_roll",
    "right_arm_gripper",
]

# SO100/SO101 arm kinematics constants (in meters)
ARM_L1 = 0.1159  # Upper arm length
ARM_L2 = 0.1350  # Lower arm length


class Controller:
    """robot controller utilities."""

    robot: XLerobot
    _connected: bool = False

    def __init__(self) -> None:
        robot_config = XLerobotConfig()
        self.robot = XLerobot(robot_config)
        self._connected = False
        try:
            self.robot.connect()
            self._connected = True
            print("[CONTROLLER] Successfully connected to robot")
        except Exception as e:
            print(f"[CONTROLLER] Failed to connect to robot: {e}")
            print(
                "[CONTROLLER] Robot control will be disabled, but video streaming will continue"
            )

    def move_head(self, pan_deg: float, tilt_deg: float) -> dict[str, float]:
        """Compute and send the action; returns the action used."""
        if not self._connected:
            return {}
        try:
            obs = self.robot.get_observation()
            targets = {
                HEAD_MOTOR_MAP["head_motor_1"]: pan_deg,
                HEAD_MOTOR_MAP["head_motor_2"]: tilt_deg,
            }

            action: dict[str, float] = {}
            for motor_name, target in targets.items():
                current = obs.get(f"{motor_name}.pos", 0.0)
                error = target - current
                kp = 0.81  # simple proportional gain
                action[f"{motor_name}.pos"] = current + kp * error

            self.robot.send_action(action)
            return action
        except Exception:
            return {}

    def move_base(
        self, x_vel: float, theta_vel: float, y_vel: float = 0.0
    ) -> dict[str, float]:
        """Send base velocity command; returns the action used.

        Args:
          x_vel: forward/backward velocity in m/s (positive = forward).
          theta_vel: rotational velocity in deg/s (positive = counter-clockwise).
          y_vel: lateral velocity in m/s (positive = left), default 0 for arrow-style control.
        """
        if not self._connected:
            return {}
        try:
            action = {
                "x.vel": float(x_vel),
                "y.vel": float(y_vel),
                "theta.vel": float(theta_vel),
            }

            self.robot.send_action(action)
            return action
        except Exception:
            return {}

    def inverse_kinematics_2d(
        self, x: float, z: float, l1: float = ARM_L1, l2: float = ARM_L2
    ) -> tuple[float, float] | None:
        """
        Calculate inverse kinematics for a 2-link planar arm (shoulder_lift and elbow_flex).

        Args:
            x: End effector x coordinate (forward distance)
            z: End effector z coordinate (height)
            l1: Upper arm length
            l2: Lower arm length

        Returns:
            (shoulder_lift_deg, elbow_flex_deg) or None if unreachable
        """
        dist_sq = x * x + z * z
        dist = math.sqrt(dist_sq)

        # Check reachability
        if dist > l1 + l2 or dist < abs(l1 - l2):
            return None

        # Law of cosines for elbow angle
        cos_elbow = (l1 * l1 + l2 * l2 - dist_sq) / (2 * l1 * l2)
        cos_elbow = max(-1.0, min(1.0, cos_elbow))
        elbow_rad = math.acos(cos_elbow)

        # Shoulder angle
        alpha = math.atan2(z, x)
        cos_beta = (l1 * l1 + dist_sq - l2 * l2) / (2 * l1 * dist)
        cos_beta = max(-1.0, min(1.0, cos_beta))
        beta = math.acos(cos_beta)
        shoulder_rad = alpha + beta

        # Convert to degrees
        shoulder_lift_deg = (
            math.degrees(shoulder_rad) - 90
        )  # Offset for arm orientation
        elbow_flex_deg = 180 - math.degrees(elbow_rad)

        return shoulder_lift_deg, elbow_flex_deg

    def move_arm(
        self,
        side: str,
        position: dict[str, float],
        orientation: dict[str, float],
        gripper: float,
    ) -> dict[str, float]:
        """
        Move an arm based on end-effector position, orientation, and gripper state.

        This uses simplified inverse kinematics:
        - position.x, position.z -> shoulder_lift, elbow_flex (2D planar IK)
        - position.y -> shoulder_pan (lateral movement)
        - orientation.yaw -> wrist_roll
        - orientation.pitch -> wrist_flex
        - gripper -> gripper position (0-100)

        Args:
            side: 'left' or 'right'
            position: {'x': float, 'y': float, 'z': float} in meters
            orientation: {'roll': float, 'pitch': float, 'yaw': float} in radians
            gripper: 0.0 (closed) to 1.0 (open)

        Returns:
            Action dict sent to robot
        """
        if not self._connected:
            return {}

        try:
            # Select joint names based on side
            if side == "left":
                joints = LEFT_ARM_JOINTS
            elif side == "right":
                joints = RIGHT_ARM_JOINTS
            else:
                print(f"[CONTROLLER] Invalid arm side: {side}")
                return {}

            # Extract position and orientation
            x = position.get("x", 0.2)
            y = position.get("y", 0.0)
            z = position.get("z", 0.1)
            # roll is available but not currently used: orientation.get("roll", 0.0)
            pitch = orientation.get("pitch", 0.0)
            yaw = orientation.get("yaw", 0.0)

            # Calculate 2D inverse kinematics for shoulder_lift and elbow_flex
            ik_result = self.inverse_kinematics_2d(x, z)
            if ik_result is None:
                # Target unreachable, skip update
                return {}

            shoulder_lift_deg, elbow_flex_deg = ik_result

            # Map y position to shoulder_pan (lateral movement)
            # Typical range: -90 to 90 degrees
            shoulder_pan_deg = math.degrees(math.atan2(y, x))
            # Mirror for right arm
            if side == "right":
                shoulder_pan_deg = -shoulder_pan_deg

            # Map orientation to wrist joints
            wrist_flex_deg = math.degrees(pitch)
            wrist_roll_deg = math.degrees(yaw)

            # Gripper: scale from 0-1 to 0-100
            gripper_value = gripper * 100.0

            # Build target positions
            targets = {
                joints[0]: shoulder_pan_deg,  # shoulder_pan
                joints[1]: shoulder_lift_deg,  # shoulder_lift
                joints[2]: elbow_flex_deg,  # elbow_flex
                joints[3]: wrist_flex_deg,  # wrist_flex
                joints[4]: wrist_roll_deg,  # wrist_roll
                joints[5]: gripper_value,  # gripper
            }

            # Get current observation for P control
            obs = self.robot.get_observation()

            # Apply P control for smooth movement
            action: dict[str, float] = {}
            kp = 0.5  # Proportional gain (lower for smoother motion)

            for joint_name, target in targets.items():
                if joint_name.endswith("gripper"):
                    # Gripper uses percentage directly
                    current = obs.get(f"{joint_name}.pos", 50.0)
                    error = target - current
                    action[f"{joint_name}.pos"] = current + kp * error
                else:
                    # Position joints
                    current = obs.get(f"{joint_name}.pos", 0.0)
                    error = target - current
                    action[f"{joint_name}.pos"] = current + kp * error

            self.robot.send_action(action)
            return action

        except Exception as e:
            print(f"[CONTROLLER] Error moving arm: {e}")
            return {}

    def move_arms(
        self,
        left_arm: dict | None = None,
        right_arm: dict | None = None,
    ) -> dict[str, float]:
        """
        Move both arms simultaneously.

        Args:
            left_arm: {'position': {...}, 'orientation': {...}, 'gripper': float} or None
            right_arm: {'position': {...}, 'orientation': {...}, 'gripper': float} or None

        Returns:
            Combined action dict sent to robot
        """
        result = {}

        if left_arm:
            left_action = self.move_arm(
                side="left",
                position=left_arm.get("position", {}),
                orientation=left_arm.get("orientation", {}),
                gripper=left_arm.get("gripper", 1.0),
            )
            result.update(left_action)

        if right_arm:
            right_action = self.move_arm(
                side="right",
                position=right_arm.get("position", {}),
                orientation=right_arm.get("orientation", {}),
                gripper=right_arm.get("gripper", 1.0),
            )
            result.update(right_action)

        return result
