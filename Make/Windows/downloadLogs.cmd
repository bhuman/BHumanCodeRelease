@echo off
pushd "%~dp0"
bash ../Common/downloadLogs %*
popd
