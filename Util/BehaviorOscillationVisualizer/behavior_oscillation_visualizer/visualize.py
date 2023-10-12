from __future__ import annotations

import multiprocessing as mp
import re
from concurrent.futures import ProcessPoolExecutor
from io import StringIO
from pathlib import Path

import click
import numpy as np
import plotext as plt
import pybh.logs as bhlogs
from rich.console import Console
from rich.progress import track

from .statistics.angle_oscillation import AngleOscillation
from .statistics.behavior_oscillation import BehaviorOscillation
from .statistics.motion_oscillation import MotionOscillation
from .statistics.statistic import Statistic
from .statistics.target_oscillation import TargetOscillation


def prepare_paths(path: Path, exclude_invisibles: bool = True) -> list[Path]:
    logs: list[Path] = []
    if path.is_file():
        logs.append(path)
    else:
        pattern = (
            r"^(?!\.)(.*?)__(?!Invisibles).*?(_1stHalf|2ndHalf)?_\d(_\(\d\d\))?.log$"
            if exclude_invisibles
            else r"^(?!\.)(.*?)__(.*?)(_1stHalf|2ndHalf)?_\d(_\(\d\d\))?.log$"
        )
        for log_path in path.rglob("*.log"):
            if re.match(pattern, str(log_path.name)):
                logs.append(log_path)
    return logs


@click.group()
def cli():
    pass


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option("-t", "--threshold", "threshold", type=click.IntRange(min=1), default=1)
@click.option(
    "-g", "--grouping-threshold", "grouping_threshold", type=click.IntRange(min=1), default=120
)
@click.option("-q", "--quiet", is_flag=True, default=False)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
def motion(
    path: Path, threshold: int, grouping_threshold: int, quiet: bool, include_invisibles: bool
):
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = MotionOscillation(
            console=console,
            threshold=threshold,
            grouping_threshold=grouping_threshold,
            quiet=quiet,
        )
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{str(log_path.relative_to(path))}: {stats.hits} hits")
        except RuntimeError:
            console.print(f"{str(log_path.relative_to(path))}: could not parse")


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option("-t", "--threshold", "threshold", type=click.IntRange(min=1), default=1)
@click.option(
    "-g", "--grouping-threshold", "grouping_threshold", type=click.IntRange(min=1), default=120
)
@click.option("-q", "--quiet", is_flag=True, default=False)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
def behavior(
    path: Path, threshold: int, grouping_threshold: int, quiet: bool, include_invisibles: bool
):
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = BehaviorOscillation(
            console=console, threshold=threshold, grouping_threshold=grouping_threshold, quiet=quiet
        )
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{str(log_path.relative_to(path))}: {stats.hits} hits")
        except RuntimeError:
            console.print(f"{str(log_path.relative_to(path))}: could not parse")


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option(
    "-b", "--buffer_size", "buffer_size", type=click.IntRange(min=1), default=80, show_default=True
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.FloatRange(min=0.0, max=180.0),
    default=45.0,
    show_default=True,
)
@click.option("-q", "--quiet", is_flag=True, default=False)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
def walking(path: Path, buffer_size: int, threshold: float, quiet: bool, include_invisibles: bool):
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    rad_threshold = np.deg2rad(threshold)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = AngleOscillation(
            buffer_length=buffer_size, threshold=rad_threshold, console=console, quiet=quiet
        )
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{str(log_path.relative_to(path))}: {stats.hits}")
        except RuntimeError:
            console.print(f"{str(log_path.relative_to(path))}: could not parse")


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option(
    "-b", "--buffer_size", "buffer_size", type=click.IntRange(min=1), default=60, show_default=True
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.FloatRange(min=0.0, max=180.0),
    default=10.0,
    show_default=True,
)
@click.option(
    "-g", "--grouping-threshold", "grouping_threshold", type=click.IntRange(min=1), default=120
)
@click.option("-q", "--quiet", is_flag=True, default=False)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
def target(
    path: Path,
    buffer_size: int,
    threshold: float,
    grouping_threshold: int,
    quiet: bool,
    include_invisibles: bool,
):
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    rad_threshold = np.deg2rad(threshold)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = TargetOscillation(
            buffer_length=buffer_size,
            threshold=rad_threshold,
            grouping_threshold=grouping_threshold,
            console=console,
            quiet=quiet,
        )
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{str(log_path.relative_to(path))}: {stats.hits}")
        except RuntimeError:
            console.print(f"{str(log_path.relative_to(path))}: could not parse")


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option("-n", "--n-buckets", "n_buckets", type=click.IntRange(min=1), default=1)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
def battery(path: Path, n_buckets: int, include_invisibles: bool):
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))

        levels = [
            frame["SystemSensorData"].batteryLevel
            for frame in track(log, console=console, transient=True)
            if "SystemSensorData" in frame
        ]
        window_size = len(levels) // n_buckets

        bucket_levels = []

        for i in range(0, len(levels), window_size):
            lvls = levels[i : i + window_size]
            level = sum(lvls) / len(lvls)
            bucket_levels.append(level)

        plt.simple_bar(bucket_levels, title="Battery level")
        plt.show()


@cli.command()
@click.argument(
    "path",
    type=click.Path(
        exists=True,
        file_okay=True,
        dir_okay=True,
        writable=False,
        readable=True,
        resolve_path=True,
        path_type=Path,
    ),
)
@click.option("-i", "--include-invisibles", is_flag=True, default=False)
@click.option("-n", "--n-processes", "n_processes", type=click.IntRange(min=0), default=1)
def all(path: Path, include_invisibles: bool, n_processes: int):
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    cli_console = Console()
    if n_processes < 1:
        n_processes = mp.cpu_count()

    with ProcessPoolExecutor(max_workers=n_processes) as executor:
        for log_path, stats in executor.map(all_worker, logs):
            cli_console.rule(str(log_path.relative_to(path)))
            if len(stats) > 0:
                for name, hits in stats.items():
                    cli_console.print(f"{name}: {hits}")
            else:
                cli_console.print("could not parse")


def all_worker(log_path: Path):
    stat_classes: list[type[Statistic]] = [
        AngleOscillation,
        BehaviorOscillation,
        MotionOscillation,
        TargetOscillation,
    ]
    stats: list[Statistic] = []
    for stat_class in stat_classes:
        console = Console(file=StringIO(), record=True)
        stat = stat_class(console=console, quiet=False)
        stats.append(stat)
    try:
        log = bhlogs.Log(str(log_path))
        for frame_idx, frame in enumerate(log):
            for stat in stats:
                stat.update(frame=frame, frame_idx=frame_idx)
        for stat in stats:
            save_path = log_path.with_suffix(".html").with_stem(
                log_path.stem + "-" + stat.__class__.__name__ + "-" + str(stat.hits)
            )
            stat.console.save_html(str(save_path))
        return log_path, {stat.__class__.__name__: stat.hits for stat in stats}
    except RuntimeError:
        return log_path, {}


if __name__ == "__main__":
    cli()
