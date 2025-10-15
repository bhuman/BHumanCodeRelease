import argparse
import math
import os
import multiprocessing
import shutil
import tempfile
import subprocess
import platform
import zlib
import time
import sys
import psutil
import csv
import logging
from os.path import join as pjoin
import signal
from cmd_parser import parse_command, CommandParseError, unparse_command

PROCESS_CHECK_DELAY = 3
EXTRA_WAIT_TIME = 360

HALF_DURATION = 600
HALF_BREAK_DURATION = 10

COLUMN_HEADERS = {
    "game": ("Game No", "Half", "Kickoff", "Score A", "Score B", "Budget A", "Budget B", "Winner"),
    "situation": ("Test Run", "Target Hit Code", "Success")
}


def parse_arguments():
    """
    Parse command-line arguments.

    Returns:
        argparse.Namespace: Parsed arguments.
    """
    parser = argparse.ArgumentParser(description="B-Human Test Runner")
    parser.add_argument("scene", help="Path to the scene file (.ros2) or directory.")
    parser.add_argument("--env", choices=["debug", "develop", "release"], default="release",
                        help="Environment mode: release, debug, or develop (default: release).")
    parser.add_argument("--testcmd", help="Test command enclosed in double quotes.")
    parser.add_argument("--workers", type=int, default=multiprocessing.cpu_count(),
                        help="Number of processes to run (default: number of CPU cores).")
    parser.add_argument("--loglevel", choices=["debug", "info", "warning", "error", "critical"], default="warning",
                        help="Logging level (default: warning).")
    parser.add_argument("--gui", action="store_true",
                        help="Run the tests with the SimRobot GUI (default: no GUI).")
    args = parser.parse_args()
    if args.workers < 1:
        parser.error("Number of workers must be at least 1.")
    args.loglevel = args.loglevel.upper()
    return args


def crc32hex(string):
    """
    Compute the CRC32 checksum of a string and return it as a hexadecimal string.

    Args:
        string (str): The string to compute the checksum of.

    Returns:
        str: The hexadecimal representation of the CRC32 checksum.
    """
    return f"{zlib.crc32(string.encode()):08x}"


def get_bhuman_root():
    """
    Get the path to the B-Human root directory by traversing the directory tree.

    Returns:
        str: The B-Human root directory path.

    Raises:
        FileNotFoundError: If the B-Human root directory is not found.
    """
    current_dir = os.path.abspath(os.getcwd())
    while True:
        if os.path.basename(current_dir) == "B-Human" and os.path.isdir(pjoin(current_dir, "Config")):
            return current_dir
        parent_dir = os.path.dirname(current_dir)
        if parent_dir == current_dir:
            raise FileNotFoundError("This script must be run from within the B-Human directory.")
        current_dir = parent_dir


def get_bhuman_relpath(file_path):
    """
    If the given file_path is inside BHUMAN_ROOT, strip everything above BHUMAN_ROOT.
    Otherwise, return the original path.

    :param file_path: The full file path to process.
    :return: A normalized relative path if inside BHUMAN_ROOT, else the original path.
    """
    abs_path = os.path.abspath(file_path)

    if abs_path.startswith(BaseConfig.bhuman_root):
        return os.path.relpath(abs_path, BaseConfig.bhuman_root)

    return file_path


class BaseConfig:
    """
    Base configuration class to manage paths and directories.
    """
    bhuman_root = get_bhuman_root()
    temp_scenes_dir = pjoin(bhuman_root, "Config", "Scenes", "Temp")

    @staticmethod
    def setup_dirs():
        """
        Create the necessary directories for the test runner.
        """
        os.makedirs(BaseConfig.temp_scenes_dir, exist_ok=True)


class TestConfig:
    """
    Configuration for the test environment.

    Attributes:
        env (str): Environment mode (debug, develop, release).
        testcmd (str): Test command to run.
        num_workers (int): Number of workers (processes).
        simrobot_path (str): Path to the SimRobot executable.
        tempdir (str): Temporary directory for the tests.
        scene (str): Path to the scene file.
        mode (str): Type of the test (game or situation).
        num_runs (int): Number of test runs.
        time_per_run (int): Time allotted for each test run.
    """

    def __init__(self, args):
        """
        Initialize the test configuration based on the parsed arguments.

        Args:
            args (argparse.Namespace): The command-line arguments.
        """
        self.env = args.env.capitalize()
        self.testcmd = args.testcmd
        self.gui = args.gui
        self.num_workers = args.workers
        self.simrobot_path = self.get_simrobot_path(self.env)
        self.tempdir = self.get_temp_dir()
        self.scene = self.get_scene_path(args.scene)

        try:
            self.parsed_cmd = parse_command(self.testcmd)
        except CommandParseError as e:
            raise ValueError(f"Invalid test command: {e}")

        self.mode = self.parsed_cmd["mode"]
        self.num_runs = self.parsed_cmd["numRuns"]
        self.time_per_run = HALF_DURATION * 2 + \
            HALF_BREAK_DURATION if self.mode == "game" else self.parsed_cmd["runTimeout"]
        self.num_workers = min(args.workers, self.num_runs)

    @staticmethod
    def get_simrobot_path(env):
        """
        Get the path to the SimRobot executable based on the environment.

        Args:
            env (str): The environment mode (debug, develop, release).

        Returns:
            str: The path to the SimRobot executable.

        Raises:
            FileNotFoundError: If the SimRobot executable is not found.
        """
        if platform.system() == "Windows":
            path = pjoin(BaseConfig.bhuman_root, "Build", "Windows", "SimRobot", env, "SimRobot.exe")
        elif platform.system() == "Darwin":
            path = pjoin(BaseConfig.bhuman_root, "Build", "macOS",  "SimRobot",
                         env, "SimRobot.app/Contents/MacOS/SimRobot")
        else:
            path = pjoin(BaseConfig.bhuman_root, "Build", "Linux",  "SimRobot", env, "SimRobot")

        if not os.path.exists(path):
            raise FileNotFoundError(f"SimRobot executable not found at {path}.")

        return path

    @staticmethod
    def get_temp_dir():
        """
        Get the temporary directory for the tests.

        Returns:
            str: The path to the temporary directory.
        """
        return pjoin(tempfile.gettempdir(), "b-human")

    @staticmethod
    def get_scene_path(scene):
        """
        Get the absolute path to the scene file.

        Args:
            scene (str): Path to the scene file.

        Returns:
            str: The absolute path to the scene file.

        Raises:
            ValueError: If the scene file does not have the .ros2 extension.
        """
        if not scene.endswith(".ros2"):
            raise ValueError("Scene file must have the .ros2 extension.")
        return os.path.normpath(scene if os.path.isabs(scene) else pjoin(BaseConfig.bhuman_root, scene))


class TestRunner:
    """
    A class that runs the tests by managing processes and collecting results.

    Attributes:
        config (TestConfig): The test configuration.
        test_scenes (list): List of test scenes to run.
        results_dirs (dict): Dictionary of process IDs and their result directories.
        active_pids (list): List of process IDs for running tests.
        aggregated_results (list): List of aggregated test results.
    """

    def __init__(self, config):
        """
        Initialize the test runner with the provided configuration.

        Args:
            config (TestConfig): The test configuration.
        """
        self.config = config
        self.test_scenes = []
        self.results_dirs = {}
        self.active_pids = []
        self.aggregated_results = []
        self.results_file = ""

        signal.signal(signal.SIGINT, self.handle_termination)
        signal.signal(signal.SIGTERM, self.handle_termination)

    def create_test_scene(self, scene_src, testcmd):
        """
        Create a test scene by copying the scene and configuration files, and appending the test command.

        Args:
            scene_src (str): Path to the source scene file.
            testcmd (str): The formatted test command.

        Returns:
            str: The path to the generated test scene.
        """
        src_filename = os.path.splitext(os.path.basename(scene_src))[0]
        dst_filename = f"{src_filename}_{crc32hex(testcmd)}"

        # Due to relative includes in config files, we cannot use any other directories than bhuman
        scene_dir = os.path.dirname(scene_src)
        scene_dst = pjoin(scene_dir, f"{dst_filename}.ros2")

        config_src = pjoin(scene_dir, f"{src_filename}.con")
        config_dst = pjoin(scene_dir, f"{dst_filename}.con")

        shutil.copy(scene_src, scene_dst)
        shutil.copy(config_src, config_dst)

        with open(config_dst, "a") as f:
            f.write(f"\n{testcmd}\n")

        return scene_dst

    def prepare_test_scenes(self):
        """
        Prepare the test scenes based on the distribution of runs across workers.
        """
        runs_distribution = self.distribute_runs(self.config.num_runs, self.config.num_workers)
        worker_idx = 0
        for runs, workers in runs_distribution.items():
            for _ in range(workers):
                updated_cmd = {
                    **self.config.parsed_cmd,
                    "numRuns": runs,
                    "teamNums": (worker_idx * 2, worker_idx * 2 + 1),
                }
                scene_path = self.create_test_scene(self.config.scene, unparse_command(updated_cmd))
                self.test_scenes.append(scene_path)
                worker_idx += 1

    def build_process_results_dir(self, scene_path, pid):
        """
        Build the directory to store the results of a test process.

        Args:
            scene_path (str): Path to the scene file.
            pid (int): Process ID.

        Returns:
            str: The path to the test results directory.
        """
        dir_id = crc32hex(f"{scene_path}:{pid}")
        return pjoin(self.config.tempdir, f"test_{dir_id}")

    def run_tests(self):
        """
        Run the tests by launching processes for each test scene.
        """
        with multiprocessing.Pool(processes=self.config.num_workers) as pool:
            for pid, scene in zip(pool.map(self.run_single_test, self.test_scenes), self.test_scenes):
                self.active_pids.append(pid)
                self.results_dirs[pid] = self.build_process_results_dir(scene, pid)

    def run_single_test(self, scene_path):
        """
        Run a single test in a subprocess.

        Args:
            scene_path (str): Path to the scene file.

        Returns:
            int: The process ID of the started process.
        """
        try:
            if self.config.gui:
                proc = subprocess.Popen([self.config.simrobot_path, scene_path])
            else:
                proc = subprocess.Popen([self.config.simrobot_path, "-noWindow", scene_path])
            logging.info(f"Started process {proc.pid}.")
            return proc.pid
        except OSError:
            self.shutdown("Failed to start the SimRobot process.")

    def run(self):
        """
        Run the entire testing process: prepare scenes, run tests, monitor processes, and collect results.
        """
        self.prepare_test_scenes()
        self.run_tests()
        self.monitor_processes()
        self.collect_results(export_csv=True)
        self.print_results()
        self.evaluate_results()

    def monitor_processes(self):
        """
        Monitor the processes, checking their status and handling timeouts.
        """
        # runs_per_cpu = max(math.ceil(self.config.num_runs / multiprocessing.cpu_count()), 1)
        # deadline = time.time() + runs_per_cpu * self.config.time_per_run + EXTRA_WAIT_TIME
        deadline = time.time() + self.config.num_runs * self.config.time_per_run + EXTRA_WAIT_TIME

        while self.active_pids:
            for pid in list(self.active_pids):
                state_file = pjoin(self.results_dirs[pid], "state.txt")
                test_status = self.fetch_test_status(state_file)

                if test_status == "finished":
                    logging.info(f"Process {pid} has finished.")
                    self.active_pids.remove(pid)
                    self.terminate_processes(pid)

                elif not psutil.pid_exists(pid):
                    self.shutdown(f"Process {pid} has died unexpectedly.")

            if time.time() > deadline:
                self.shutdown("Timeout reached.")

            time.sleep(PROCESS_CHECK_DELAY)

    def collect_results(self, export_csv=False):
        """
        Collect the results of the tests and save them to a CSV file.
        """
        collected = []
        for results_dir in self.results_dirs.values():
            try:
                with open(pjoin(results_dir, "results.csv"), "r") as f:
                    r = csv.reader(f)
                    next(r, None)  # Skip header
                    collected.extend(r)
            except FileNotFoundError:
                logging.warning(f"Results file not found in {results_dir}.")

        if self.config.mode == "game":
            # First two lines belong to the same game
            self.aggregated_results = [[((i - 1) // 2) + 1] + row[1:] for i, row in enumerate(collected, start=1)]
        else:
            self.aggregated_results = [[i] + row[1:] for i, row in enumerate(collected, start=1)]
                
        if export_csv:
            self.results_file = pjoin(self.config.tempdir, f"{time.strftime('%Y%m%d%H%M%S')}_results.csv")
            with open(self.results_file, "w", newline="") as f:
                writer = csv.writer(f)
                writer.writerow(COLUMN_HEADERS[self.config.mode])
                writer.writerows(self.aggregated_results)

            logging.info(f"Aggregated results saved to {self.results_file}.")

    def print_results(self):
        """
        Print the aggregated results of the tests.
        """
        if not self.aggregated_results:
            logging.info("No results to display.")
            return

        header = COLUMN_HEADERS[self.config.mode]
        col_widths = [max(len(str(value)) for value in col) for col in zip(header, *self.aggregated_results)]

        def format_row(row):
            return " | ".join(f"{str(value):<{col_widths[i]}}" for i, value in enumerate(row))

        print("=" * 80)
        print("Scene:", get_bhuman_relpath(self.config.scene))
        print("Command:", self.config.testcmd)
        print("Results:", self.results_file)
        print("=" * 80)

        print(format_row(header))
        print("-+-".join("-" * width for width in col_widths))

        for row in self.aggregated_results:
            if self.config.mode == "situation":
                fmt_results = [row[0], row[1], row[2].replace("true", "✓").replace("false", "✗")]
            else:
                fmt_results = row
            print(format_row(fmt_results))

    def evaluate_results(self):
        """
        Evaluate the results of the tests and exit with an error if not all runs were successful.
        """
        if self.config.mode == "situation":
            for row in self.aggregated_results:
                if row[2] != "true":
                    self.shutdown("Test failed.")

    def handle_termination(self, signum, _):
        """
        Handle termination signals (SIGINT, SIGTERM) by cleaning up processes.
        """
        self.shutdown(f"Received termination signal ({signum}).")

    def shutdown(self, message):
        """
        Clean up and shut down the program with an error message.

        Args:
            message (str): The error message to log.
        """
        self.terminate_processes(*self.active_pids)
        logging.error(message)
        sys.exit(1)

    @staticmethod
    def distribute_runs(num_runs, num_workers):
        """
        Distribute the number of runs across the available workers.

        Args:
            num_runs (int): The total number of runs.
            num_workers (int): The number of workers.

        Returns:
            dict: A dictionary with the number of runs per worker.
        """
        base, extra = divmod(num_runs, num_workers)
        result = {}
        if base > 0:
            result[base] = num_workers - extra
        if extra > 0:
            result[base + 1] = extra
        return result

    @staticmethod
    def terminate_processes(*pids):
        """
        Terminate the specified SimRobot processes.

        Args:
            pids (int): The process IDs to terminate.
        """
        for pid in pids:
            try:
                proc = psutil.Process(pid)
                if proc.name().removesuffix(".exe") == "SimRobot":
                    proc.terminate()
            except psutil.Error:
                pass

    @staticmethod
    def fetch_test_status(state_file):
        """
        Fetch the status of the test from the state file.

        Args:
            state_file (str): Path to the state file.

        Returns:
            str: The status of the test, or None if the file is not found.
        """
        try:
            with open(state_file, "r") as f:
                for line in f:
                    if line.startswith("status ="):
                        return line.split("=")[1].strip().strip(";")
        except FileNotFoundError:
            return None
        return None


def process_scene_for_test(scene_file):
    """
    Prepare a scene file by extracting the test command and updating configuration.

    Args:
        scene_file (str): Path to the original scene file (.ros2).

    Returns:
        tuple: 
            - new_scene_file (str): The new scene file with a unique name.
            - testcmd (str or None): The extracted test command, if found.
    """
    scene_basepath = os.path.splitext(scene_file)[0]
    scene_basename = os.path.basename(scene_basepath)

    with open(os.path.splitext(scene_file)[0] + ".con", "r") as f:
        lines = f.readlines()

    testcmd = None
    new_config_lines = []

    for line in lines:
        if line.startswith("test"):
            testcmd = line.strip()
            break
        new_config_lines.append(line)

    if testcmd is None:
        return None, None

    new_scene_name = f"{scene_basename}_{crc32hex(testcmd)}"
    new_scene_file = pjoin(BaseConfig.temp_scenes_dir, new_scene_name + ".ros2")
    new_config_file = pjoin(BaseConfig.temp_scenes_dir, new_scene_name + ".con")

    shutil.copy(scene_file, new_scene_file)

    with open(new_config_file, "w") as f:
        f.writelines(new_config_lines)

    return new_scene_file, testcmd


def find_test_scenes(scene_dir):
    """
    Find test scenes in the specified directory.

    Args:
        scene_dir (str): Path to the directory containing the test scenes.

    Returns:
        list: List of test scene files with their test commands.
    """
    scenes = []
    for file in os.listdir(scene_dir):
        if file.endswith(".ros2"):
            scenes.append(pjoin(scene_dir, file))

    test_scenes = []
    for scene in scenes:
        scene, testcmd = process_scene_for_test(scene)
        if scene:
            test_scenes.append((scene, testcmd))
    return test_scenes


def resolve_test_scenes(scene, testcmd):
    """
    Get the test scenes based on the input arguments.

    Args:
        scene (str): Path to the scene file or directory.
        testcmd (str): Test command enclosed in double quotes.

    Returns:
        list: List of test scenes with their test commands.
    """
    if testcmd is None:
        scene = scene if os.path.isabs(scene) else pjoin(BaseConfig.bhuman_root, scene)
        if os.path.isdir(scene):
            return find_test_scenes(scene)
        return [process_scene_for_test(scene)]
    return [(scene, testcmd)]


def main():
    """
    Main function to initiate the test process.
    """
    BaseConfig.setup_dirs()
    args = parse_arguments()

    logging.getLogger().setLevel(args.loglevel)

    for scene, testcmd in resolve_test_scenes(args.scene, args.testcmd):
        logging.info(f"Running test for scene {get_bhuman_relpath(scene)}.")
        args.scene, args.testcmd = scene, testcmd
        config = TestConfig(args)
        TestRunner(config).run()


if __name__ == "__main__":
    main()
