from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .angle import Angle
from .util.motion import Motion

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class TargetOscillation(Angle):
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

    def _update(self, target_direction: float, rotation: float, frame_idx: int) -> None:
        target_angle = self._normalize(target_direction + rotation)
        self._buffer[self._n_updates % self._buffer_length] = np.abs(
            self._normalize(self._last_target_angle - target_angle)
        )

        self._last_target_angle = target_angle

        if (mean := np.mean(self._buffer)) > self.threshold:
            self.hits += 1
            if not self.quiet:
                if self._ellipsis:
                    self.console.print("...")
                self.console.print(
                    f"(frame {frame_idx}, cognition frame {self._n_updates}): {np.rad2deg(mean):.3f}° mean absolute angular difference over {self._buffer_length} frames"
                )
                self._ellipsis = False
            self._last_hit_update = self._n_updates
        elif (self._n_updates - self._last_hit_update) > self._grouping_threshold:
            self._ellipsis = True
        self._n_updates += 1

    def reset(self) -> None:
        self._buffer = np.zeros((self._buffer_length), dtype=np.float32)
        self._last_target_angle = 0.0

    def update(self, frame: bhlogs.Frame, frame_idx: int) -> None:
        if frame.thread != "Cognition" or "MotionRequest" not in frame or "RobotPose" not in frame:
            return

        motion_request = frame["MotionRequest"]
        robot_pose = frame["RobotPose"]

        if not (
            hasattr(motion_request, "targetDirection")
            and hasattr(motion_request, "motion")
            and hasattr(robot_pose, "rotation")
        ):
            return

        target_direction = motion_request.targetDirection
        if not isinstance(target_direction, float):
            msg = f"Expected float, got {type(target_direction)}"
            raise TypeError(msg)
        motion_type: Motion = Motion(motion_request.motion)
        rotation = robot_pose.rotation
        if not isinstance(rotation, float):
            msg = f"Expected float, got {type(rotation)}"
            raise TypeError(msg)

        if motion_type != Motion.WALK_TO_BALL_AND_KICK:
            self.reset()
            return

        self._update(target_direction=target_direction, rotation=rotation, frame_idx=frame_idx)
