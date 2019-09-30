@echo off
set SHELLOPTS=igncr
pushd "%~dp0"
set USER=%USERNAME%
bash ../Common/copyLogs %*
popd 
