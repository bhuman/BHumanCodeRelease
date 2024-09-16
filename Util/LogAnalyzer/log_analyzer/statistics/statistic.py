from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class Statistic:
    def __init__(self, console: Console, *args, quiet: bool = True, **kwargs) -> None:
        self.console = console
        self.hits = 0
        self.quiet = quiet

    def update(self, frame: bhlogs.Frame, frame_idx: int) -> None:
        raise NotImplementedError

    def update_hits(self) -> int:
        self.hits += 1
        return self.hits

    def set_console(self, console: Console) -> None:
        self.console = console
