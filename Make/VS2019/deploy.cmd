@echo off
set SHELLOPTS=igncr

:: Add bash.exe to PATH
set PATH=%PATH%;%systemroot%\Sysnative\

pushd "%~dp0"
bash.exe ../Common/deploy %*
popd
