@echo off
pushd "%~dp0"
net localgroup | grep '*' | head -n 1 | sed "s! !\\\ !g" | sed "s!.\(.*\)!\1 ../../Config/Keys/id_rsa_nao!" | xargs chgrp
..\..\Util\mare\Windows\bin\mare.exe --vcxproj=2013
popd
