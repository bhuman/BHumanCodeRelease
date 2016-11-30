@echo off
pushd "%~dp0"
id -g | sed 's@$@ ../../Config/Keys/id_rsa_nao@' | xargs chgrp
for /f "usebackq" %%i in (`bash -c "( grep avx </proc/cpuinfo ; echo 'avx=false') | head -1 | sed 's/.*avx .*/avx=true/'"`) do ..\..\Util\SimRobot\Util\mare\Windows\bin\mare.exe --vcxproj=2015 %%i
popd
