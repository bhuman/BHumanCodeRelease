@echo off
pushd "%~dp0"
bash ../Common/deploy %*
popd
