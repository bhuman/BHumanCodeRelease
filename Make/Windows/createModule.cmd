@echo off
pushd "%~dp0"
bash ../Linux/createModule
generate.cmd
popd
