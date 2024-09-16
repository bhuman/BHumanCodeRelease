from __future__ import annotations

import numpy as np
from rich.console import Console

from .walking_oscillation import WalkingOscillation


def test_thresholds() -> None:
    console = Console()
    threshold = np.deg2rad(45) / 2
    stats = WalkingOscillation(buffer_length=2, threshold=threshold, console=console)
    stats2 = WalkingOscillation(
        buffer_length=2, threshold=threshold + np.deg2rad(1) / 2, console=console
    )
    for i in range(4):
        stats._update((i % 2, 1), i)
        stats2._update((i % 2, 1), i)
    assert stats.hits == 3
    assert stats2.hits != 3


def test_thresholds2() -> None:
    console = Console()
    threshold = np.deg2rad(135) / 2
    stats = WalkingOscillation(buffer_length=2, threshold=threshold, console=console)
    stats2 = WalkingOscillation(
        buffer_length=2, threshold=threshold + np.deg2rad(1) / 2, console=console
    )
    for i in range(4):
        stats._update((-i % 2, 1 - (i % 2) * 2), i)
        stats2._update((-i % 2, 1 - (i % 2) * 2), i)
    assert stats.hits == 3
    assert stats2.hits != 3
