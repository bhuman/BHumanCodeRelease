"""A single log frame; representations accessed by name."""
from __future__ import annotations

from typing import TYPE_CHECKING, Any

import numpy as np

from pybh._schema import Schema, Typed
from pybh._special import SPECIAL

try:
    from . import logs as _logs
except ImportError:
    _logs = None  # type: ignore[assignment]

from pybh.logs import Annotation


class Frame:
    """A single log frame."""

    def __init__(self, raw, schema: Schema) -> None:
        self._f = raw
        self._s = schema

    @property
    def thread(self) -> str:
        return self._f.thread

    @property
    def representations(self) -> list[str]:
        return self._f.representations

    @property
    def annotations(self) -> list[Annotation]:
        return self._f.annotations

    def __contains__(self, x: str) -> bool:
        return x in self._f

    def __getitem__(self, name: str) -> Typed:
        wrapper = SPECIAL.get(name, Typed)
        return wrapper(self._f[name], name, self._s)

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        rep = self._s.rep_for(name)
        if rep is None:
            raise AttributeError(f'frame has no representation {name!r}')
        return self[rep] if rep in self._f else None

    def __dir__(self) -> list[str]:
        return list(super().__dir__()) + sorted(self._s._rep_by_accessor)

    def visualize(self) -> np.ndarray | None:
        if 'JPEGImage' not in self:
            return None
        return self['JPEGImage'].to_rgb()
