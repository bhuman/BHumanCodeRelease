from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .statistic import Statistic
from .util.motion import Motion

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class TargetOscillation(Statistic):
    def __init__(
        self,
        console: Console,
        buffer_length: int = 60,
        threshold: float = np.deg2rad(10),
        grouping_threshold: int = 60,
        *,
        quiet: bool = True,
    ) -> None:
        super().__init__(console=console, quiet=quiet)
        self._buffer_length: int = buffer_length
        self.threshold: float = threshold
        self._grouping_threshold = grouping_threshold
        self._ellipsis = False
        self._buffer = np.zeros((self._buffer_length), dtype=np.float32)
        self._n_updates: int = 0
        self._last_hit_update: int = -1
        self._last_target_angle: float = 0.0

    @staticmethod
    def _normalize(angle: float) -> float:
        if -np.pi < angle < np.pi:
            return angle
        else:
            angle = angle - int(angle / 2 * np.pi) * 2 * np.pi
            if angle > np.pi:
                return angle - 2 * np.pi
            elif angle < -np.pi:
                return angle + 2 * np.pi
            else:
                return angle

    def _update(self, targetDirection: float, rotation: float, frame_idx: int):
        target_angle = self._normalize(targetDirection + rotation)
        self._buffer[self._n_updates % self._buffer_length] = np.abs(
            self._last_target_angle - target_angle
        )

        self._last_target_angle = target_angle

        if (mean := np.mean(self._buffer)) > self.threshold:
            self.hits += 1
            if not self.quiet:
                if self._ellipsis:
                    self.console.print("...")
                self.console.print(
                    f"(frame {frame_idx}, cognition frame {self._n_updates}): {np.rad2deg(mean):.3f} mean angular velocity"
                )
                self._ellipsis = False
            self._last_hit_update = self._n_updates
        else:
            if (self._n_updates - self._last_hit_update) > self._grouping_threshold:
                self._ellipsis = True
        self._n_updates += 1

    def reset(self):
        self._buffer = np.zeros((self._buffer_length), dtype=np.float32)
        self._last_target_angle = 0.0

    def update(self, frame: bhlogs.Frame, frame_idx: int):
        if frame.thread != "Cognition":
            return

        if "MotionRequest" not in frame:
            return
        motion_request = frame["MotionRequest"]

        if "RobotPose" not in frame:
            return
        robot_pose = frame["RobotPose"]

        if not hasattr(motion_request, "targetDirection"):
            return
        targetDirection: float = motion_request.targetDirection

        if not hasattr(motion_request, "motion"):
            return
        motion_type: Motion = Motion(motion_request.motion)

        if not hasattr(robot_pose, "rotation"):
            return
        rotation: float = robot_pose.rotation

        if motion_type != Motion.WALK_TO_BALL_AND_KICK:
            self.reset()
            return

        self._update(targetDirection=targetDirection, rotation=rotation, frame_idx=frame_idx)
