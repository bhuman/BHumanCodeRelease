@echo off
set SHELLOPTS=igncr
pushd "%~dp0"
bashexec ../Common/copyfiles %*
set STATUS=%ERRORLEVEL%
popd 
if not %STATUS%==0 SET A_BUILD_ERROR_HAS_HAPPENED 2>nul
