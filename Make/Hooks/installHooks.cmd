@echo off
pushd "%~dp0"
bash ./installHooks %*
popd
