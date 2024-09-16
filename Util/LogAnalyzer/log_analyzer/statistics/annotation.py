from __future__ import annotations

from typing import TYPE_CHECKING

import pybh.logs

from .statistic import Statistic

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class Annotation(Statistic):
    def __init__(
        self,
        console: Console,
        *,
        grouping_threshold: int = 60 * 3,
        quiet: bool = True,
    ) -> None:
        super().__init__(console, quiet=quiet)
        self._last_changed_frame: int = -1
        self._ellipsis: bool = False
        self._grouping_threshold: int = grouping_threshold

    def update(self, frame: bhlogs.Frame, frame_idx: int) -> None:
        if (frame_idx - self._last_changed_frame) > self._grouping_threshold:
            self._ellipsis = True
        if not hasattr(frame, "annotations") or not frame.annotations:
            return

        for el in frame.annotations:
            if isinstance(el, pybh.logs.Annotation):
                if not self.quiet:
                    if self._ellipsis:
                        self.console.print("...")
                        self._ellipsis = False
                    self.console.print(
                        f"(frame {frame_idx:07} thread {frame.thread:9}): {el.name} - {el.description}"
                    )
                self.hits += 1
                self._last_changed_frame = frame_idx
