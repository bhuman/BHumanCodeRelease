"""
pybh

Quick start::

    from pybh import Log

    log = Log('game.log', keep_going=True)
    for frame in log:
        r = frame.robot_pose          # or frame['RobotPose']
        b = frame.ball_model
        if r and b:
            print(r.translation, b.estimate.position)
"""
from __future__ import annotations

from importlib.metadata import PackageNotFoundError, version
from typing import Any

try:
    __version__ = version('pybh')
except PackageNotFoundError:
    __version__ = 'unknown'

try:
    from . import logs as _logs
except ImportError:
    _logs = None  # type: ignore[assignment]

from pybh._frame import Frame  # noqa: E402
from pybh._schema import Schema, Typed  # noqa: E402


class Log:
    """Iterable sequence Frame objects parsed from a B-Human .log file."""

    def __init__(self, path: str, keep_going: bool = False) -> None:
        if _logs is None:
            raise ImportError(
                "pybh's compiled 'logs' extension is not built; "
                'run `pip install -e Make/Python/ --no-build-isolation`'
            )
        self._log = _logs.Log(path, keep_going)
        self._schema = Schema(self._log.type_info)

    @property
    def type_info(self) -> Any:
        """Raw pybh.logs.TypeInfo schema embedded in the log."""
        return self._log.type_info

    @property
    def schema(self) -> Schema:
        return self._schema

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        return getattr(self._log, name)

    def __len__(self) -> int:
        return len(self._log)

    def __iter__(self):
        for frame in self._log:
            yield Frame(frame, self._schema)

    def __getitem__(self, index: int) -> Frame:
        """Frame at index."""
        return Frame(self._log[index], self._schema)

    def extract(self, representation: str, fields: list[str]) -> dict[str, list]:
        """Collect fields."""
        columns: dict[str, list] = {field: [] for field in fields}
        columns['index'] = []
        for i, frame in enumerate(self):
            if representation not in frame:
                continue
            value = frame[representation]
            for field in fields:
                obj = value
                for part in field.split('.'):
                    obj = getattr(obj, part)
                columns[field].append(obj)
            columns['index'].append(i)
        return columns

    def bisect(self, key, target) -> int:
        return bisect_frame(self, key, target)


def bisect_frame(sequence, key, target) -> int:
    lo, hi = 0, len(sequence)
    while lo < hi:
        mid = (lo + hi) // 2
        if key(sequence[mid]) < target:
            lo = mid + 1
        else:
            hi = mid
    return lo


__all__ = ['Frame', 'Log', 'Schema', 'Typed', 'bisect_frame']
