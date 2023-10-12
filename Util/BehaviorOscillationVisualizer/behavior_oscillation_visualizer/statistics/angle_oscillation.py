from __future__ import annotations

from typing import TYPE_CHECKING

import numpy as np

from .statistic import Statistic

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class AngleOscillation(Statistic):
    def __init__(
        self,
        console: Console,
        buffer_length: int = 20,
        threshold: float = np.deg2rad(45),
        *,
        quiet: bool = True,
    ) -> None:
        super().__init__(console=console, quiet=quiet)
        self._buffer_length = buffer_length
        self.threshold = threshold
        self._buffer = np.zeros((self._buffer_length), dtype=np.float32)
        self._n_updates = 0
        self._last_walkingTo = (0.0, 0.0)

    def _update(self, walkingTo: tuple[float, float], frame_idx: int):
        angles = np.arctan2(
            (self._last_walkingTo[0], walkingTo[0]),
            (self._last_walkingTo[1], walkingTo[1]),
            dtype=np.float32,
        )

        self._buffer[self._n_updates % self._buffer_length] = np.abs(angles[0] - angles[1])

        self._last_walkingTo = walkingTo

        if (mean := np.mean(self._buffer)) > self.threshold:
            self.hits += 1
            if not self.quiet:
                self.console.print(f"{frame_idx}: mean angular velocity: {np.rad2deg(mean):.3f}")
        self._n_updates += 1

    def update(self, frame: bhlogs.Frame, frame_idx: int):
        if frame.thread != "Cognition":
            return
        if "BehaviorStatus" not in frame:
            return
        behavior_status = frame["BehaviorStatus"]
        if not hasattr(behavior_status, "walkingTo"):
            return
        walkingTo = (behavior_status.walkingTo.y, behavior_status.walkingTo.x)
        self._update(walkingTo=walkingTo, frame_idx=frame_idx)
