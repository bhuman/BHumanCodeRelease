# Behavior Oscillation Visualizer

## Installation

```
pip install -r requirements.txt
```

> Note: This project requires Python Bindings for the B-Human Framework (pybh). For installation please refer to the [Wiki](https://wiki.b-human.de/master/python-bindings/).

A small tool to quickly iterate over a log and output changes in the ActivationGraph based on a user-specified 'phase' threshold. Facilitates quickly circling in on oscillating frame ranges.

## Usage
```
python3 -m behavior_oscillation_visualizer.visualize behavior [-t <threshold>] [-q] <log_path>
-t <threshold>: a kind of phase threshold. Must be at least 1. Default: 1
-q            : only output the total number of hits
<log_path>    : Path to the log to be analyzed
```
```
python3 -m behavior_oscillation_visualizer.visualize battery [-n <n_buckets>] <log_path>
-n <n_buckets>: Number of bars in bar plot. Must be at least 1. Default: 1
<log_path>    : Path to the log to be analyzed
```
