#!/bin/bash
##
## Copyright (C) 2015 Aldebaran Robotics
##

PRINTF=/usr/bin/printf
I2CSET=/usr/sbin/i2cset
I2CGET=/usr/sbin/i2cget
TEST=/usr/bin/test
LSPCI=/usr/bin/lspci
GREP=/bin/grep
I2C_DEVICE_NUMBER=0

get_i2c_device_number() {
   local dir_cmd="ls -l /dev/i2c-head"
   I2C_DEVICE_NUMBER=$($dir_cmd | sed 's:.* -> i2c-\(.*\)$:\1:')
}

usage() {
  ${PRINTF} "\nusage: ${0} [hold|release|toggle]\n\n"
}

reset_camera_release() {
  ${I2CSET} -y $I2C_DEVICE_NUMBER 0x41 0x1 0xc  i
}

reset_camera_hold() {
  ${I2CSET} -y $I2C_DEVICE_NUMBER 0x41 0x1 0x0  i
}

reset_camera_config_output() {
  local cfg=$(${I2CGET} -y $I2C_DEVICE_NUMBER 0x41 0x3)
  if ${TEST} $((cfg & 0xc)) -eq 0 ; then
    # already configured as output
    return
  fi
  # preset output value (hold reset lines)
  reset_camera_hold
  # set GPIO direction:
  # - pin 0 (usb hub reset): input
  # - pin 1 (reserved):      input
  # - pin 2 (CX3 top camera reset): output
  # - pin 3 (CX3 top camera reset): output
  ${I2CSET} -y $I2C_DEVICE_NUMBER 0x41 0x3 0xf3 i
}

get_i2c_device_number
reset_camera_config_output
case "${1}" in
  hold)
    reset_camera_hold
    ;;
  release)
    reset_camera_release
    ;;
  toggle)
    reset_camera_hold
    reset_camera_release
    ;;
  *)
    usage
    exit 1
    ;;
esac
