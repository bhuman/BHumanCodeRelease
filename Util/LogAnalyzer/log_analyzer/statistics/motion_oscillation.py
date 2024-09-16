from __future__ import annotations

from typing import TYPE_CHECKING

from .statistic import Statistic
from .util.motion import Motion

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class MotionOscillation(Statistic):
    def __init__(
        self,
        console: Console,
        threshold: int = 10,
        *,
        grouping_threshold: int = 60 * 3,
        quiet: bool = True,
    ) -> None:
        super().__init__(console=console, quiet=quiet)
        self._last_motion_type: Motion = Motion.STAND
        self._last_changed_update: int = -1
        self._n_updates: int = 0
        self._ellipsis: bool = False
        self._threshold: int = threshold
        self._grouping_threshold = grouping_threshold

    def _update(self, motion_type: Motion, frame_idx: int) -> None:
        if self._last_motion_type != motion_type:
            if (self._n_updates - self._last_changed_update) < self._threshold:
                if not self.quiet:
                    if self._ellipsis:
                        self.console.print("...")
                    self.console.print(
                        f"(frame {frame_idx}): motion {self._last_motion_type.name} -> {motion_type.name}"
                    )
                    self._ellipsis = False
                self.hits += 1
            self._last_changed_update = self._n_updates
        elif (self._n_updates - self._last_changed_update) > self._grouping_threshold:
            self._ellipsis = True
        self._last_motion_type = motion_type
        self._n_updates += 1

    def update(self, frame: bhlogs.Frame, frame_idx: int) -> None:
        if frame.thread != "Cognition" or "MotionRequest" not in frame:
            return
        motion_request = frame["MotionRequest"]
        if not hasattr(motion_request, "motion"):
            return

        motion_type: Motion = Motion(motion_request.motion)
        self._update(motion_type=motion_type, frame_idx=frame_idx)
