@echo off
pushd "%~dp0"
bash -c "cp ../../Config/Keys/id_rsa_nao /tmp/id_rsa_nao && id -g | sed 's/$/ \/tmp\/id_rsa_nao/' | xargs chgrp"
for /f "usebackq tokens=*" %%i in (`bash -c "echo `(grep ssse3 </proc/cpuinfo ; echo 'ssse3=false') | head -1 | sed 's/.*ssse3 .*/ssse3=true/'` `(grep avx2 </proc/cpuinfo ; echo 'avx2=false') | head -1 | sed 's/.*avx2 .*/avx2=true/'`;"`) do ..\..\Util\SimRobot\Util\mare\Windows\bin\mare.exe --vcxproj=2017 %%i
popd
