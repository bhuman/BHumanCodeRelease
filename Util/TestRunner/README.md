# Test Runner

A test runner for automated testing, designed to run tests on a given scene using a specified command. This tool supports parallel execution with multiple workers (processes) and tracks the results of the tests.

## Requirements

- Python 3.x
- `psutil` (for process management)

Install required packages using pip:

```bash
pip install psutil
```

## Usage

### Command-Line Arguments

The script requires the following command-line arguments:

- `scene`: **Positional argument**. Path to a raw scene file (e.g., `.ros2` file), a test scene file with a configuration that includes a test command, or a directory containing such test scene files.
- `--env`: Environment mode. Options: `debug`, `develop`, `release` (default: `develop`).
- `--testcmd`: Test command enclosed in double quotes (e.g., `--testcmd "test game 10"`).
- `--workers`: Number of worker processes to run (default: number of CPU cores).

### Example

Run the test runner with the desired configuration:

```bash
python test_runner.py /path/to/scene_or_directory --env develop --testcmd "test game 10" --workers 4
```
This command will run the test command "test game 10" on the specified scene using 4 worker processes in the develop environment.

## Running the Tests

Once the necessary arguments are provided, the script will:

1. Parse the test command and configuration.
2. If a directory is provided for the `scene`, it will find all `.ros2` scene files and extract their respective test commands.
3. Distribute the runs across the available workers.
4. Run each test in a separate worker process.
5. Monitor the processes and terminate them once completed or if they fail.
6. Collect results into a CSV file.

## Output

- A CSV file will be generated with the test results, containing either match details for the `game` test type or summary statistics for the `situation` test type.
  
  Example output file:
  
  ```
  results_1632948372.csv
  Match ID, Team A Score, Team B Score, Winner
  1, 3, 1, Team A
  2, 2, 2, Draw
  ```

## Logging

The test runner generates logs at the `DEBUG` level, providing detailed information on the execution process, including process IDs, error messages, and results collection. Logs will be printed to the standard output by default.
