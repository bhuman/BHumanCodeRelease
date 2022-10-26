@echo off
pushd "%~dp0"
bash ../Common/downloadCalibration %*
popd
