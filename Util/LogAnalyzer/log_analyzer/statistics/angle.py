from __future__ import annotations

import numpy as np

from .statistic import Statistic


class Angle(Statistic):
    @staticmethod
    def _normalize(angle: float) -> float:
        if -np.pi <= angle < np.pi:
            return angle

        angle = angle - int(angle / (2 * np.pi)) * 2 * np.pi
        if angle >= np.pi:
            return angle - 2 * np.pi
        if angle < -np.pi:
            return angle + 2 * np.pi
        return angle
