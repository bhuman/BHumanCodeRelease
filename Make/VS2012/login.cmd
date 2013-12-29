@echo off
echo sc Remote %1 >../../Config/Scenes/connect.con
ssh -i ../../Config/Keys/id_rsa_nao -o StrictHostKeyChecking=no nao@%1
