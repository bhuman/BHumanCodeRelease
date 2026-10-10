from __future__ import annotations

import keyword
import re
from enum import IntEnum
from typing import Any

import numpy as np

_RESERVED = frozenset({'type'})
_INT_HINTS = ('int', 'short', 'char', 'long')

def _vec2(v) -> np.ndarray:
    return np.array([float(v.x), float(v.y)], dtype=np.float32)


def _vec2i(v) -> np.ndarray:
    return np.array([int(v.x), int(v.y)], dtype=np.int32)


def _vec3(v) -> np.ndarray:
    return np.array([float(v.x), float(v.y), float(v.z)], dtype=np.float32)


def _quat(v) -> np.ndarray:
    return np.array([float(v.w), float(v.x), float(v.y), float(v.z)], dtype=np.float32)


def _mat(m) -> np.ndarray:
    cols = list(m.cols)
    ncols = len(cols)
    nrows = len(list(cols[0].elems)) if ncols else 0
    return np.array(
        [[float(cols[c].elems[r]) for c in range(ncols)] for r in range(nrows)],
        dtype=np.float32,
    )

def snake(name: str) -> str:
    s = re.sub(r'([A-Z]+)([A-Z][a-z])', r'\1_\2', name)
    s = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', s)
    return s.lower()


def py_identifier(type_name: str) -> str:
    return re.sub(r'\W', '_', type_name)


def accessor(field_name: str) -> str:
    name = re.sub(r'\W', '_', snake(field_name))
    return f'{name}_' if keyword.iskeyword(name) or name in _RESERVED else name


def is_ndarray_class(field_names: set[str]) -> bool:
    return (field_names in ({'x', 'y'}, {'x', 'y', 'z'}, {'w', 'x', 'y', 'z'})
            or 'cols' in field_names)


class Schema:

    def __init__(self, type_info) -> None:
        self.classes: dict[str, list[tuple[str, str]]] = {
            k: [tuple(a) for a in v] for k, v in type_info.classes.items()
        }
        self.enums: dict[str, list[str]] = {k: list(v) for k, v in type_info.enums.items()}
        self.primitives: set[str] = set(type_info.primitives)
        self._fields: dict[str, dict[str, tuple[str, str]]] = {}
        self._enum_classes: dict[str, type[IntEnum]] = {}
        self.reps: set[str] = {n[2:] for n in self.enums.get('MessageID', ())
                               if n.startswith('id') and n[2:] in self.classes}
        self._rep_by_accessor: dict[str, str] = {accessor(r): r for r in self.reps}

    def rep_for(self, name: str) -> str | None:
        return self._rep_by_accessor.get(name)

    def field_map(self, type_name: str) -> dict[str, tuple[str, str]]:
        m = self._fields.get(type_name)
        if m is None:
            m = {}
            for ftype, fname in self.classes.get(type_name, ()):
                entry = (fname, ftype)
                m[accessor(fname)] = entry
                m[fname] = entry  # also accept the raw field name
            self._fields[type_name] = m
        return m

    def enum_class(self, type_name: str) -> type[IntEnum]:
        e = self._enum_classes.get(type_name)
        if e is None:
            names = self.enums[type_name]
            e = IntEnum(py_identifier(type_name), {n: i for i, n in enumerate(names)})
            self._enum_classes[type_name] = e
        return e

    def convert(self, raw, type_str: str) -> Any:
        if type_str.endswith('*'):
            if isinstance(raw, (bytes, bytearray)):
                return raw
            return [self.convert(x, type_str[:-1]) for x in raw]
        if type_str.endswith(']'):
            if isinstance(raw, (bytes, bytearray)):
                return raw
            base = type_str[:type_str.rfind('[')]
            return [self.convert(x, base) for x in raw]
        if type_str in self.enums:
            return self._to_enum(raw, type_str)
        if type_str in self.classes:
            return self._convert_class(raw, type_str)
        return raw

    def _to_enum(self, raw, type_str: str):
        e = self.enum_class(type_str)
        if isinstance(raw, str):
            return e[raw] if raw in e.__members__ else raw
        try:
            return e(int(raw))
        except (ValueError, TypeError):
            return raw

    def _convert_class(self, raw, type_str: str):
        fields = self.classes[type_str]
        names = {fname for _, fname in fields}
        if is_ndarray_class(names):
            try:
                if names == {'x', 'y'}:
                    xtype = next(t for t, n in fields if n == 'x')
                    return _vec2i(raw) if any(h in xtype for h in _INT_HINTS) else _vec2(raw)
                if names == {'x', 'y', 'z'}:
                    return _vec3(raw)
                if names == {'w', 'x', 'y', 'z'}:
                    return _quat(raw)
                return _mat(raw)
            except Exception:
                pass
        return Typed(raw, type_str, self)


class Typed:

    __slots__ = ('_r', '_t', '_s')

    def __init__(self, record, type_name: str, schema: Schema) -> None:
        self._r = record
        self._t = type_name
        self._s = schema

    @property
    def raw(self):
        """pybh.logs.Record"""
        return self._r

    @property
    def type_name(self) -> str:
        return self._t

    def __getattr__(self, name: str) -> Any:
        if name.startswith('_'):
            raise AttributeError(name)
        entry = self._s.field_map(self._t).get(name)
        if entry is None:
            raise AttributeError(f'{self._t!r} has no field {name!r}')
        cpp, ftype = entry
        return self._s.convert(getattr(self._r, cpp), ftype)

    def __dir__(self):
        base = list(super().__dir__())
        return base + [snake(n) for _, n in self._s.classes.get(self._t, ())]

    def __repr__(self) -> str:
        return f'<{self._t}>'
