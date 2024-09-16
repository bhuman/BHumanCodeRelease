from __future__ import annotations

from typing import TYPE_CHECKING

from .statistic import Statistic

if TYPE_CHECKING:
    import pybh.logs as bhlogs
    from rich.console import Console


class _Node:
    def __init__(self, node: bhlogs.Record) -> None:
        self.option: str = node.option  # pyright: ignore[reportAttributeAccessIssue]
        self.depth: int = node.depth  # pyright: ignore[reportAttributeAccessIssue]
        # self.state: str = node.state
        # self.optionTime: int = node.optionTime
        # self.stateTime: int = node.stateTime
        # self.parameters: list[str] = [param for param in node.parameters]

    def __eq__(self, __value: object) -> bool:
        if not isinstance(__value, type(self)):
            return NotImplemented
        return __value.option == self.option


class BehaviorOscillation(Statistic):
    def __init__(
        self,
        console: Console,
        threshold: int = 10,
        *,
        grouping_threshold: int = 60 * 3,
        quiet: bool = True,
    ) -> None:
        super().__init__(console=console, quiet=quiet)
        self._last_graph: list[_Node] = []
        self._last_changed_update: int = -1
        self._n_updates: int = 0
        self._ellipsis: bool = False
        self._threshold: int = threshold
        self._grouping_threshold = grouping_threshold

    def _update(self, graph: list[_Node], frame_idx: int) -> None:
        for node, last_node in zip(graph, self._last_graph):
            if last_node != node:
                if (self._n_updates - self._last_changed_update) < self._threshold:
                    if not self.quiet:
                        if self._ellipsis:
                            self.console.print("...")
                        self.console.print(
                            f"(frame {frame_idx}, depth {node.depth:02d}): option {last_node.option} -> {node.option}"
                        )
                        self._ellipsis = False
                    self.hits += 1
                self._last_changed_update = self._n_updates
                break
        else:
            if (self._n_updates - self._last_changed_update) > self._grouping_threshold:
                self._ellipsis = True
        self._n_updates += 1
        self._last_graph = graph

    def update(self, frame: bhlogs.Frame, frame_idx: int) -> None:
        if frame.thread != "Cognition" or "ActivationGraph" not in frame:
            return
        activation_graph = frame["ActivationGraph"]
        if not hasattr(activation_graph, "graph"):
            return

        graph: list[_Node] = [_Node(node) for node in activation_graph.graph]  # pyright: ignore[reportGeneralTypeIssues]
        self._update(graph=graph, frame_idx=frame_idx)
