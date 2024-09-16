from __future__ import annotations

import multiprocessing as mp
import re
from concurrent.futures import ProcessPoolExecutor
from io import StringIO
from pathlib import Path
from typing import TYPE_CHECKING

import click
import numpy as np
import pybh.logs as bhlogs
from rich.console import Console
from rich.progress import track

from .statistics.annotation import Annotation
from .statistics.behavior_oscillation import BehaviorOscillation
from .statistics.motion_oscillation import MotionOscillation
from .statistics.target_oscillation import TargetOscillation
from .statistics.walking_oscillation import WalkingOscillation

if TYPE_CHECKING:
    from .statistics.statistic import Statistic

stat_classes: list[type[Statistic]] = [
    Annotation,
    BehaviorOscillation,
    TargetOscillation,
]


def prepare_paths(path: Path, exclude_invisibles: bool = True) -> list[Path]:
    if path.is_file():
        return [path]
    pattern = (
        r"^(?!\.).*?__"
        + (r"(?!Invisibles)" if exclude_invisibles else r"")
        + r".*?(_(1st|2nd)Half)?_\d(_\(\d\d\))?\.log$"
    )
    matcher = re.compile(pattern)
    return [log_path for log_path in path.rglob("*.log") if matcher.match(str(log_path.name))]


@click.group()
def cli() -> None:
    """Analyze logs in the B-Human format.

    This tool is designed to analyze logs in the B-Human format. For a detailed description of the
    commands and their options, refer to the respective command's help text by invoking the command
    with the `--help` flag.
    """
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
@click.option(
    "-i",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    is_flag=True,
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
@click.option(
    "-n",
    "--n-processes",
    "n_processes",
    type=click.IntRange(min=0),
    default=1,
    show_default=True,
    help="Number of processes to use. Set to 0 to use all available CPUs.",
)
def report(path: Path, include_invisibles: bool, n_processes: int) -> None:
    """Generate a report for all available statistics.

    The following statistics are available:

    - Annotation

    - BehaviorOscillation

    - TargetOscillation

    PATH may be a directory containing log files or a single log file.

    This command will generate reports for each log file found in PATH. A report will contain the
    console output of a statistic. The report will be saved as an HTML file next to the respective
    log file. The file name will be the same as the log file with the statistic name and the number
    of hits appended. A file will also be created if the statistic did not have any hits. This is
    to indicate that the statistic was successfully run on the log file.

    Including the hit count in the file name allows for quick identification of logs with potential
    issues.

    Currently parameters known from the individual statistics' commands cannot be adjusted at
    runtime. Refer to the default values in the constructors of the respective statistics classes
    for adjustments.

    The number of processes to use can be adjusted with the `--n-processes` parameter. Set it to 0
    to use all available CPUs. To avoid overloading the system of an unsuspecting user, the
    default has been set to 1.

    The `--include-invisibles` flag can be used to include logs against the Invisibles team (a.k.a.
    a testgame).
    """
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    cli_console = Console()
    if n_processes < 1:
        n_processes = mp.cpu_count()

    with ProcessPoolExecutor(max_workers=n_processes) as executor:
        for log_path, stats in executor.map(report_worker, logs):
            cli_console.rule(str(log_path.relative_to(path)))
            if len(stats) > 0:
                for name, hits in stats.items():
                    cli_console.print(f"{name}: {hits}")
            else:
                cli_console.print("could not parse")


def report_worker(log_path: Path) -> tuple[Path, dict[str, int]]:
    """Worker function for the report command.

    This function is responsible for processing a single log file and generating reports for all
    available statistics.

    Args:
        log_path (Path): The path to the log file to process.
    """
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


@cli.command(hidden=True)
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
    "-q", "--quiet", is_flag=True, default=False, help="Reduce output to a hit count per file."
)
@click.option(
    "-i",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    is_flag=True,
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.IntRange(min=1),
    default=10,
    show_default=True,
    help="Upper threshold of frames to count as a hit.",
)
@click.option(
    "-g",
    "--grouping-threshold",
    "grouping_threshold",
    type=click.IntRange(min=1),
    default=120,
    show_default=True,
    help="Threshold in frames to group hits together.",
)
def motion(
    path: Path,
    threshold: int,
    grouping_threshold: int,
    quiet: bool,
    include_invisibles: bool,
) -> None:
    """Print potential oscillations in motion status.

    PATH may be a directory containing log files or a single log file.

    Potential oscillations are detected by observing changes in the motion type.

    If a change occurs before a given threshold, a hit is counted. It can be adjusted with the
    `--threshold` parameter.

    As a visual aid to identify oscillations in close proximity to each other, ellipses (...) are
    printed between hits that are separated by a certain number frames. This may be adjusted with
    the `--grouping-threshold` parameter.
    """
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
            rel_path = log_path.relative_to(path) if log_path != path else log_path.name
        except ValueError:
            msg = f"Log path {log_path} is not a subpath of {path}. This should not happen."
            raise ValueError(msg) from None
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{rel_path}: {stats.hits} hits")
        except RuntimeError:
            console.print(f"{rel_path}: could not parse.")


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
    "-q", "--quiet", is_flag=True, default=False, help="Reduce output to a hit count per file."
)
@click.option(
    "-i",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    is_flag=True,
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.IntRange(min=1),
    default=10,
    show_default=True,
    help="Upper threshold of frames to count as a hit.",
)
@click.option(
    "-g",
    "--grouping-threshold",
    "grouping_threshold",
    type=click.IntRange(min=1),
    default=120,
    show_default=True,
    help="Threshold in frames to group hits together.",
)
def behavior(
    path: Path,
    threshold: int,
    grouping_threshold: int,
    quiet: bool,
    include_invisibles: bool,
) -> None:
    """Print potential oscillations in behavior status.

    PATH may be a directory containing log files or a single log file.

    Potential oscillations are detected by observing changes in the behavior graph. The graph is
    represented as a list of nodes where parent nodes are assumed to occur before their children.
    No special graph construction is performed and a change in the graph is counted when the list
    of nodes changes.

    If a change occurs before a given threshold, a hit is counted. It can be adjusted with the
    `--threshold` parameter.

    As a visual aid to identify oscillations in close proximity to each other, ellipses (...) are
    printed between hits that are separated by a certain number frames. This may be adjusted with
    the `--grouping-threshold` parameter.
    """
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = BehaviorOscillation(
            console=console,
            threshold=threshold,
            grouping_threshold=grouping_threshold,
            quiet=quiet,
        )
        try:
            rel_path = log_path.relative_to(path) if log_path != path else log_path.name
        except ValueError:
            msg = f"Log path {log_path} is not a subpath of {path}. This should not happen."
            raise ValueError(msg) from None
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{rel_path}: {stats.hits} hits")
        except RuntimeError:
            console.print(f"{rel_path}: could not parse.")


@cli.command(hidden=True)
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
    "-q", "--quiet", is_flag=True, default=False, help="Reduce output to a hit count per file."
)
@click.option(
    "-i",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    is_flag=True,
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
@click.option(
    "-b",
    "--buffer_size",
    "buffer_size",
    type=click.IntRange(min=1),
    default=80,
    show_default=True,
    help="Size of the angle buffer to average over.",
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.FloatRange(min=0.0, max=180.0),
    default=45.0,
    show_default=True,
    help="Threshold in degrees for the mean angle between walking directions.",
)
def walking(
    path: Path,
    buffer_size: int,
    threshold: float,
    quiet: bool,
    include_invisibles: bool,
) -> None:
    """Print potential oscillations in walking direction.

    PATH may be a directory containing log files or a single log file.

    Potential oscillations are detected by computing the normalized angle difference between
    consecutive walking directions.

    If the mean angle exceeds a given threshold, a hit is counted. It can be adjusted with the
    `--threshold` parameter.

    The size of the buffer of angle-differences to average over can be adjusted with the
    `--buffer-size` parameter.

    As a visual aid to identify oscillations in close proximity to each other, ellipses (...) are
    printed between hits that are separated by a certain number frames. This may be adjusted with
    the `--grouping-threshold` parameter.
    """
    console = Console()
    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)

    rad_threshold = np.deg2rad(threshold)

    for log_path in logs:
        log = bhlogs.Log(str(log_path))
        stats = WalkingOscillation(
            buffer_length=buffer_size,
            threshold=rad_threshold,
            console=console,
            quiet=quiet,
        )
        try:
            rel_path = log_path.relative_to(path) if log_path != path else log_path.name
        except ValueError:
            msg = f"Log path {log_path} is not a subpath of {path}. This should not happen."
            raise ValueError(msg) from None
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{rel_path}: {stats.hits}")
        except RuntimeError:
            console.print(f"{rel_path}: could not parse.")


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
    "-q", "--quiet", is_flag=True, default=False, help="Reduce output to a hit count per file."
)
@click.option(
    "-i/",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
@click.option(
    "-b",
    "--buffer_size",
    "buffer_size",
    type=click.IntRange(min=1),
    default=60,
    show_default=True,
    help="Size of the angle buffer to average over.",
)
@click.option(
    "-t",
    "--threshold",
    "threshold",
    type=click.FloatRange(min=0.0, max=180.0, max_open=True),
    default=10.0,
    show_default=True,
    help="Threshold in degrees for the mean angle between target vectors.",
)
@click.option(
    "-g",
    "--grouping-threshold",
    "grouping_threshold",
    type=click.IntRange(min=1),
    default=120,
    show_default=True,
    help="Threshold in frames to group hits together.",
)
def target(  # noqa: PLR0913
    path: Path,
    buffer_size: int,
    threshold: float,
    grouping_threshold: int,
    quiet: bool,
    include_invisibles: bool,
) -> None:
    """Print potential oscillations in ball target vectors.

    PATH may be a directory containing log files or a single log file.

    Potential oscillations are detected by averaging over the normalized absolute angle differences
    between consecutive target vectors.

    If the average exceeds a given threshold, a hit is counted. It can be adjusted with the
    `--threshold` parameter.

    The size of the buffer of angle-differences to average over can be adjusted with the
    `--buffer-size` parameter.

    As a visual aid to identify oscillations in close proximity to each other, ellipses (...) are
    printed between hits that are separated by a certain number frames. This may be adjusted with
    the `--grouping-threshold` parameter.
    """
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
            rel_path = log_path.relative_to(path) if log_path != path else log_path.name
        except ValueError:
            msg = f"Log path {log_path} is not a subpath of {path}. This should not happen."
            raise ValueError(msg) from None
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{rel_path}: {stats.hits}")
        except RuntimeError:
            console.print(f"{rel_path}: could not parse.")


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
    "-q", "--quiet", is_flag=True, default=False, help="Reduce output to a hit count per file."
)
@click.option(
    "-i",
    "--include-invisibles/--exclude-invisibles",
    "include_invisibles",
    default=False,
    show_default=True,
    help="Include matching logs against the Invisibles team (a.k.a. a testgame).",
)
def annotation(
    path: Path,
    quiet: bool,
    include_invisibles: bool,
) -> None:
    """Print annotations from logs.

    PATH may be a directory containing log files or a single log file.

    This can be used to receive a quick overview of annotations in logs without starting a full
    SimRobot instance. Based on this overview, you can decide which logs and at which frame(s) you
    want to investigate further.

    This may also be combined with other CLI-Tools such as `grep` to prepare canary-like reports for
    quick identification of issues.
    """
    console = Console()

    logs = prepare_paths(path, exclude_invisibles=not include_invisibles)
    for log_path in logs:
        log = bhlogs.Log(str(log_path))

        stats = Annotation(console=console, quiet=quiet)
        try:
            rel_path = log_path.relative_to(path) if log_path != path else log_path.name
        except ValueError:
            msg = f"Log path {log_path} is not a subpath of {path}. This should not happen."
            raise ValueError(msg) from None
        try:
            for frame_idx, frame in enumerate(track(log, transient=True, console=console)):
                stats.update(frame=frame, frame_idx=frame_idx)
            console.print(f"{rel_path}: {stats.hits}")
        except RuntimeError:
            console.print(f"{rel_path}: could not parse.")
