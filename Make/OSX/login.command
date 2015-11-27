#!/bin/sh
if [ $TERM == dumb ]; then open $0; exit; fi
clear
cd `dirname $0`
set -eu
read -p 'Cable or WLAN (C/w): ' MODE
TEAM=`grep <../../Config/settings.cfg "teamNumber" | sed "s%[^=]*=[ ]*\([0-9]*\).*%\1%"`
WIRED=`grep BASE_ETH0_IP= ../../Install/Network/wired | sed "s/[^=]*=.\([^%]*\).*/\1/"`
WLAN=`grep BASE_WLAN0_IP= ../../Install/Network/wireless | sed "s/[^=]*=.\([^%]*\).*/\1/"`
if [ "$MODE" == "w" ]; then
  SUBNET=$WLAN$TEAM.
else
  SUBNET=$WIRED$TEAM.
fi
read -p "Connect to $SUBNET" IP
REMOTE=$SUBNET$IP
if [ `ping -t 1 -c 1 $REMOTE >/dev/null && echo 1 || echo 0` = "0" ]; then
  read -p "$REMOTE not reachable" DUMMY
  exit 1
fi
./login $REMOTE
