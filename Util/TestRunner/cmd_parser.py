"""
Simple command parser for 'test' commands used in SimRobot.
===========================================================
This parser is less strict than the full command interpreter,
as it does not analyze targets in detail. It is sufficient for
the test runner to understand the command structure and supported options.

Supported modes:
----------------
1. **Game** -``test game <numRuns> [tnm <t1> <t2>] [trn teamA|teamB] [con <file.con>] [rt] [q]``

2. **Situation** - ``test situation <numRuns> <runTimeout> <targets> [tnm …] [trn …] [con …] [fuz …] [rt] [q]``

Calling :py:func:`parse_command` with a raw string or token list
returns a dictionary specific to the command mode, containing
all parsed options.
"""

from __future__ import annotations

import shlex
from typing import Any, Dict, List

__all__ = ["CommandParseError", "parse_command", "unparse_command"]


class CommandParseError(RuntimeError):
    """Raised on any syntactic or semantic error in the command."""


def _require_int(tok, what):
    try:
        return int(tok)
    except ValueError as exc:
        raise CommandParseError(f"{what} must be an integer, got '{tok}'") from exc


def _require_float(tok, what):
    try:
        return float(tok)
    except ValueError as exc:
        raise CommandParseError(f"{what} must be a number, got '{tok}'") from exc


FUZZ_KINDS = {"all", "pos", "rot", "x", "y", "z", "rotx", "roty", "rotz"}
OPTION_KEYS = {"tnm", "trn", "con", "fuz", "rt", "q"}
FLAG_KEYS = {"rt", "q"}
TEAM_IDS = {"teamA", "teamB"}


class BaseParser:
    """Base class for command parsers."""

    # Keys for options that can appear in any command
    DEFAULTS = {
        "teamNums": (None, None),
        "turn": None,
        "conFile": None,
        "rt": False,
        "q": False,
    }

    def __init__(self, tokens):
        self.tokens = tokens
        self.i = 2  # skip 'test <mode>'

    # Cursor helpers
    def _peek(self):
        return self.tokens[self.i] if self.i < len(self.tokens) else ""

    def _next(self):
        if self.i >= len(self.tokens):
            raise CommandParseError("Unexpected end of input")
        tok = self.tokens[self.i]
        self.i += 1
        return tok

    # Option parsers
    def _parse_team_nums(self) -> tuple[int, int]:
        return _require_int(self._next(), "<team1>"), _require_int(self._next(), "<team2>")

    def _parse_turn(self) -> str:
        val = self._next()
        if val not in TEAM_IDS:
            raise CommandParseError("trn must be 'teamA' or 'teamB'")
        return val

    def _parse_user_config(self) -> str:
        path = self._next()
        if not path.lower().endswith(".con"):
            raise CommandParseError("con file must end with .con")
        return path

    def _parse_flags(self) -> dict[str, bool]:
        flags = {"rt": False, "q": False}
        while self._peek() in FLAG_KEYS:
            flags[self._next()] = True
        return flags

    # Public (abstract)
    def parse(self) -> Dict[str, Any]:
        raise NotImplementedError


class GameParser(BaseParser):
    def parse(self):
        res = dict(self.DEFAULTS)
        res.update({"mode": "game", "numRuns": None})

        # positional
        res["numRuns"] = _require_int(self._next(), "<numRuns>")

        # keyword options
        while self.i < len(self.tokens):
            key = self._next()
            if key == "tnm":
                res["teamNums"] = self._parse_team_nums()
            elif key == "trn":
                res["turn"] = self._parse_turn()
            elif key == "con":
                res["conFile"] = self._parse_user_config()
            elif key in FLAG_KEYS:
                self.i -= 1
                res.update(self._parse_flags())
            elif key == "fuz":
                raise CommandParseError("fuz not allowed in game mode")
            else:
                raise CommandParseError(f"Unknown option '{key}' in game command")
        return res


class SituationParser(BaseParser):
    def _parse_fuzzing(self) -> list[tuple[str, float]]:
        pairs = []
        while self.i + 1 < len(self.tokens) and self._peek() not in OPTION_KEYS:
            kind = self._next().lower()
            if kind not in FUZZ_KINDS:
                raise CommandParseError(f"Invalid fuz kind '{kind}'")
            dev = _require_float(self._next(), f"deviation for {kind}")
            pairs.append((kind, dev))
        if not pairs:
            raise CommandParseError("fuz requires at least one <kind deviation> pair")
        return pairs

    def parse(self):
        res = dict(self.DEFAULTS)
        res.update({
            "mode": "situation",
            "numRuns": None,
            "runTimeout": None,
            "fuzzing": [],
            "targets": None,
        })

        # positional
        res["numRuns"] = _require_int(self._next(), "<numRuns>")
        res["runTimeout"] = _require_int(self._next(), "<runTimeout>")

        target_tokens = []
        while self.i < len(self.tokens) and self._peek() not in OPTION_KEYS:
            target_tokens.append(self._next())
        if not target_tokens:
            raise CommandParseError("Missing <listOfTargets>")
        res["targets"] = " ".join(target_tokens)

        # keyword options
        while self.i < len(self.tokens):
            key = self._next()
            if key == "tnm":
                res["teamNums"] = self._parse_team_nums()
            elif key == "trn":
                res["turn"] = self._parse_turn()
            elif key == "con":
                res["conFile"] = self._parse_user_config()
            elif key == "fuz":
                res["fuzzing"] = self._parse_fuzzing()
            elif key in FLAG_KEYS:
                self.i -= 1
                res.update(self._parse_flags())
            else:
                raise CommandParseError(f"Unknown option '{key}' in situation command")
        return res


_DISPATCH = {
    "game": GameParser,
    "situation": SituationParser,
}


def parse_command(cmd: str | List[str]) -> Dict[str, Any]:
    """Parse a test command, returning a dictionary for downstream logic."""
    tokens = cmd if isinstance(cmd, list) else shlex.split(cmd)
    if len(tokens) < 2 or tokens[0] != "test":
        raise CommandParseError("Command must start with 'test <mode>'")
    mode = tokens[1]
    parser_cls = _DISPATCH.get(mode)
    if not parser_cls:
        raise CommandParseError(f"Unsupported mode '{mode}'")
    return parser_cls(tokens).parse()


def unparse_command(cmd: dict) -> str:
    """Convert parsed command dictionary back into a command string."""
    tokens = ["test", cmd["mode"], str(cmd["numRuns"])]

    # Positional args
    if cmd["mode"] == "situation":
        tokens.append(str(cmd["runTimeout"]))
        tokens.extend(cmd["targets"].split())

    # Optional args (order: tnm, trn, con, fuz, rt, q)
    if None not in cmd["teamNums"]:
        team1, team2 = cmd["teamNums"]
        tokens.extend(["tnm", str(team1), str(team2)])

    if cmd.get("turn") is not None:
        tokens.extend(["trn", cmd["turn"]])

    if cmd.get("conFile"):
        tokens.extend(["con", cmd["conFile"]])

    if cmd["mode"] == "situation" and cmd.get("fuzzing"):
        tokens.append("fuz")
        for kind, dev in cmd["fuzzing"]:
            tokens.extend([kind, str(dev)])

    # Boolean flags
    for flag in ["rt", "q"]:
        if cmd.get(flag):
            tokens.append(flag)

    return " ".join(tokens)  # shlex.join(tokens) for shell-safe quoting


# For quick manual testing.
# Example:
#   python test_cmd_parser.py "test game 5 tnm 1 2 trn 1 con example.con rt q"
if __name__ == "__main__":
    import sys
    import pprint
    try:
        parsed = parse_command(sys.argv[1])
        print("Parsed:", pprint.pformat(parsed))
        print("Reconstructed:", unparse_command(parsed))
    except CommandParseError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
