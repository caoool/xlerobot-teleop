from typing import Iterable

from robot.config_xlerobot import XLerobotConfig
from robot.xlerobot import XLerobot

HEAD_MOTOR_MAP = {
  "head_motor_1": "head_motor_1",
  "head_motor_2": "head_motor_2",
}

class Controller:
  """robot controller utilities."""

  robot: XLerobot

  def __init__(self) -> None:
    robot_config = XLerobotConfig()
    self.robot = XLerobot(robot_config)
    try:
      self.robot.connect()
      print(f"[CONTROLLER] Successfully connected to robot")
    except Exception as e:
      print(f"[CONTROLLER] Failed to connect to robot: {e}")
  
  def move_head(self, pan_deg: float, tilt_deg: float) -> dict[str, float]:
    """Compute and send the action; returns the action used."""
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

  def move_base(self, x_vel: float, theta_vel: float, y_vel: float = 0.0) -> dict[str, float]:
    """Send base velocity command; returns the action used.

    Args:
      x_vel: forward/backward velocity in m/s (positive = forward).
      theta_vel: rotational velocity in deg/s (positive = counter-clockwise).
      y_vel: lateral velocity in m/s (positive = left), default 0 for arrow-style control.
    """

    action = {
      "x.vel": float(x_vel),
      "y.vel": float(y_vel),
      "theta.vel": float(theta_vel),
    }

    self.robot.send_action(action)
    return action