# Log Analyzer

A tool to iterate over a log and find interesting things.

These include indicators of oscillating behavior and relevant annotations.


## Installation

As the library `pybh` is built as part of the installation, a developer
version of Python 3.9 or higher is required. On Ubuntu (also inside the
Windows Subsystem for Linux, but without `sudo` if executed as `root`),
install the following packages:

```shell
sudo apt install python3-dev python3-venv
```

On macOS, the Python that comes with Xcode can be used. 

This instruction assumes that the directory in which this `README.md` is
located is selected as the current directory.

Make sure that these two directories do not exist:

```shell
rm -rf ../../Make/Python/build ../../Make/Python/pybh.egg-info
```

Then install the requirements (append `--use-feature=in-tree-build` if
`pip` is older than version 21.3):

```shell
python3 -m venv .venv \
&& source .venv/bin/activate \
&& pip install -r requirements.txt
```

If the Python that comes with Xcode on macOS is used, a few more steps
are required (remove `--use-feature=in-tree-build` if you upgraded `pip`
to a more recent version):

```shell
python3 -m venv .venv \
&& rm -rf .venv/include \
&& ln -s "$(readlink -f /Applications/Xcode.app/Contents/Developer/Library/Frameworks/Python3.framework/Headers)" .venv/include \
&& ln -s /Applications/Xcode.app/Contents/Developer/Library/Frameworks/Python3.framework/Versions/3.9/Python3 .venv/lib/libpython3.9.dylib \
&& source .venv/bin/activate \
&& pip install -r requirements.txt --use-feature=in-tree-build
```


## Usage

```
python3 -m log_analyzer [OPTIONS] COMMAND [ARGS]...

  Analyze logs in the B-Human format.

  This tool is designed to analyze logs in the B-Human format. For a detailed
  description of the commands and their options, refer to the respective
  command's help text by invoking the command with the `--help` flag.

Options:
  --help  Show this message and exit.

Commands:
  annotation  Print annotations from logs.
  behavior    Print potential oscillations in behavior status.
  report      Generate a report for all available statistics.
  target      Print potential oscillations in ball target vectors.
```
