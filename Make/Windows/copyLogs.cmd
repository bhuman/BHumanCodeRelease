@echo off
for /f "tokens=* usebackq" %%f in (`wsl wslpath '%~sdp0'`) do (set basedir=%%f)
bash "%basedir%../Common/copyLogs" %*
