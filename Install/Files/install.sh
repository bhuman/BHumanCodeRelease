#!/bin/sh
##
## This file was auto-generated.
##
## Copyright (C) 2010, 2011, 2012 Aldebaran Robotics
## + B-Human modifications WORK IN PROGRESS
##

stty -cread # disable terminal receiver.

#
# Set at image creation time
#
# Return:
#  - 130: if image_file is missing in upgrade_system
#  - 133: if image_raw_size is missing in upgrade_system
#  - 135: if dest_part is missing in upgrade_system
#  - 150: if add_partition has no device argument
#  - 151: if add_partition has no part_num argument
#  - 152: if add_partition has no device argument
#  - 153: if add_partition has an internal error
#  - 154: if add_partition has an internal error
#  - 155: if add_partition fail to fdisk
#  - 156: if add_partition cannot alter the partition table of device
#  - 157: if add_partition failed to fdisk
#  - 158: if add_partition cannot create a new device
#  - 160: if check_size has no dest_part
#  - 161: if min_size_kb has no min_size_kb
#  - 171: if first argument of check_image is empty
#  - 173: if installer size does not match size in header
#  - 174: if header hash is not good
#  - 175: if installer hash is not good
#  - 176: if image hash is not good
#  - 181: if check_gzip is missing an argument
#  - 185: if dd fail in check_gzip
#  - 210: if count_part get an invalid argument
#  - 220: if get_device get an invalid argument
#  - 221: if the first argument of get_device is not of type "mmcblk[0-9]+" or "sd[a-z]"
#  - 251: if this script is not launch by PID 1
#  - 254: if DFU_VERSION is not setted
#  - 255: if DFU_INIT is not setted

# any size or partition offset is given as multiple of SIZE_BASE, ie. in KByte
SIZE_BASE="1024"

INSTALLER_SIZE="1024"

IMAGE_CMP_SIZE="99999999"

IMAGE_RAW_SIZE="99999999"

ARCHIVE_CMP_SIZE="99999999"

PART_SYSTEM_NODE="/dev/mmcblk0p3"
PART_SYSTEM_UUID="42424242-1120-1120-1120-424242424242"

# returns the device node from a uuid
## get_partition_from_uuid 2a2a2a2a-a1de-a1de-a1de-2a2a2a2a2a2a => /dev/mmcblk0p2
get_partition_from_uuid() {
  # Example 'blkid' output:
  #  /dev/sda1: LABEL="boot" UUID="6f9a2340-ed6c-41b3-b063-6e914569f12d" TYPE="ext2" PARTUUID="a0e9cfc5-01"
  blkid | grep $1 | cut -d ':' -f 1
}

PART_DFU_UUID="2a2a2a2a-a1de-a1de-a1de-2a2a2a2a2a2a"
PART_DFU_NODE=$(get_partition_from_uuid "${PART_DFU_UUID}")
INSTALLER_PATH="${0}"

INSTALLER_VERSION="2"
DEFAULT_FS=ext3

MAGIC_SIZE=4096

ROOT_IMAGE_OFFSET="$(( ( ${MAGIC_SIZE} / ${SIZE_BASE} ) + ${INSTALLER_SIZE} ))"
HOME_ARCHIVE_OFFSET="$(( ${ROOT_IMAGE_OFFSET} + ${IMAGE_CMP_SIZE} ))"

LOG_FILE="/dev/null"

DEST_PART="${1}"
USER_PART="${2}"
IMAGE_FILE="${3}"
[ -n "${DEST_PART}" ] || DEST_PART="${PART_SYSTEM_NODE}"

# notification IDs
NOTIF_UPGRADE_SUCCESS=100
NOTIF_VERSION_MISMATCH=101
NOTIF_CPU_SW_MISMATCH=102
NOTIF_ROBOT_SW_MISMATCH=103
NOTIF_IMAGE_INTEGRITY=104
NOTIF_PART_LAYOUT=105
NOTIF_WRITING_ERROR=110
NOTIF_WRITTEN_DATA_INTEGRITY=111
NOTIF_FACTORY_RESET=120
# no notification for factory reset failure here,
# if something went wrong during that step, it wil be detected and handled in the main system.
# NOTIF_FACTORY_RESET_ERROR=

: ${_DEBUG:="true"}

#################################################################################

# init stuff
rm -f /tmp/get-head-id ; touch /tmp/get-head-id
# For some reasons, zcat doesn't work while gunzip -c does.
base64 -d <<EOF | gunzip -c > /tmp/get-head-id
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
eDzXxoB/ILR7/PwN+Rhf55hYAAA=
EOF
chmod 755 /tmp/get-head-id



${_DEBUG} && mount

# Sanity checks
[ -n "${DFU_INIT}" ]           || exit 255
[ -n "${DFU_VERSION}" ]        || exit 254
# This script is executed by DFU, which should be PID 1.
# If Parent PID is not 1, exit with error return code 251
[ $(cat "/proc/$$/stat" | cut -d' ' -f4) = '1' ] || exit 251

# read flags
FLAGS=$(hexdump "${IMAGE_FILE}" -n16 -s8 -v -e '16/1 "%02x"')
# [ $(( 0x$(echo "${FLAGS}" | cut -b1) & 8 )) -eq 0 ] || FLAG_NAME='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b1) & 4 )) -eq 0 ] || FLAG_NAME='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b1) & 2 )) -eq 0 ] || FLAG_NAME='y'
[ $(( 0x$(echo "${FLAGS}" | cut -b1) & 1 )) -eq 0 ] || HALT_AFTER_UPGRADE='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b2) & 8 )) -eq 0 ] || FLAG_NAME='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b2) & 4 )) -eq 0 ] || FLAG_NAME='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b2) & 2 )) -eq 0 ] || FLAG_NAME='y'
# [ $(( 0x$(echo "${FLAGS}" | cut -b2) & 1 )) -eq 0 ] || FLAG_NAME='y'

USER_DIR=$(cat /proc/mounts | grep "^${USER_PART}" | cut -d' ' -f2)
if [ ${#USER_DIR} -gt 0 ] ; then
  LOG_FILE="${USER_DIR}/.image/upgrade.log"
  mkdir -p $(dirname "${LOG_FILE}")
  touch "${LOG_FILE}"
fi

# for development purpose
${_DEBUG} && DBG_FILE="$(dirname "${IMAGE_FILE}")/upgrade.dbg" && touch "${DBG_FILE}" || \
  DBG_FILE="/dev/null"

#
# Functions
#


einfo() {
  echo -e "[info ] opn: ${@}" | tee -a "${DBG_FILE}"
  echo -e "${@}" >> "${LOG_FILE}"
}

eerror() {
  local status=$1 ; shift
  echo -e "[error] opn: ${@} (exitcode=${status})" | tee -a "${DBG_FILE}"
  echo -e "ERROR: ${@} (exitcode=${status})" >> "${LOG_FILE}"
  return ${status}
}

elog() {
  ${_DEBUG} || return 0 ; echo -e "[debug] opn: ${@}" | tee -a "${DBG_FILE}" >&2
}

watchdog() {
  local duration=${1} loop_ite="${2}" ; shift 2
  local cmd="${@}"
  local cmdpid=
  test -e ${loop_ite} && chmod +x ${loop_ite} &>/dev/null
  rm -f /tmp/.status
  sh -c "${cmd} ; status=\$? ; echo \${status} >/tmp/.status ; exit \${status}" & cmdpid=$!
  timeout -t ${duration} -s TERM \
      sh -c "\
export cnt=0 ; \
while kill -0 ${cmdpid} &>/dev/null ; do \
  ${loop_ite} ${cmdpid} ; \
  if [ \${cnt} -lt 10 ] ; then \
    cnt=\$(( \${cnt} + 1 )) ; \
  else \
    cnt=0 ; \
  fi ; \
  sleep 1 ; \
done" >&2
  kill -TERM ${cmdpid} &>/dev/null
  sync & syncpid=$!
  timeout -t ${duration} -s TERM \
    sh -c "while kill -0 ${cmdpid} &>/dev/null || kill -0 ${syncpid} &>/dev/null ; do sleep 1 ; done"  >&2
  kill -TERM ${syncpid} &>/dev/null
  kill -KILL ${cmdpid}  &>/dev/null
  kill -KILL ${syncpid} &>/dev/null
  local status=255
  [ -f /tmp/.status ] && status=$(tail -n1 /tmp/.status)
  rm -f /tmp/.status
  return ${status}
}

enotify() {
  local chest_state="${1}"
  [ ${chest_state} = "poll" ] && chest_state=
  local notification="${2}"
  shift 2
  while [ ${#} -gt 0 ] ; do
    notification="${notification};${1}" ; shift
  done
  echo "${notification}" >&2
}

die() {
  local status="${1}" function="${2}" ; shift 2
  local message="${@}"
  echo "[error] ${function}: exitcode: ${status}, error: '${message}'" >&2
  return ${status}
}

get_device() {
  echo "${1}" | grep -q '/dev/' || \
    die 220 "get_device" "invalid argument" || return $?
  local dev=$(basename "${1}" | grep -o "^\(mmcblk[0-9]\+\|sd[a-z]\)")
  [ -n "${dev}" ] || \
    die 221 "get_device" "invalid argument" || return $?
  [ -e "/sys/block/${dev}" ] || return 222
  echo "/dev/${dev}"
}

get_node() {
  local node=
  if echo "${1}" | grep -q "mmcblk" ; then
    node="${1}p${2}"
  else
    node="${1}${2}"
  fi
  echo "${node}"
}

get_part_nr() {
  echo "${1}" | grep -o '[^0-9][0-9]\+$' | cut -b2-
}

# given a partition or disk device, returns the number of partitions
# in the parent disk.
#   ex: count_part /dev/sda       => 3 (if sda[123])
#   ex: count_part /dev/sda2      => 3 (if sda[123])
#   ex: count_part /dev/mmcblk0   => 3 (if mmcblk0p[123])
#   ex: count_part /dev/mmcblk0p1 => 3 (if mmcblk0p[123])
count_part() {
  local pattern= device=$(basename "$1")

  [ -n "${device}" ] || \
    die 210 "count_part" "invalid argument" || return $?

  case "$device" in
    sd[a-z]) pattern="${device}[0-9]\+";;
    sd[a-z][0-9]) pattern="${device/[0-9]/}[0-9]\+";;
    mmcblk[0-9]p[0-9]) pattern="${device/p[0-9]/p}[0-9]\+";;
    mmcblk[0-9]) pattern="${device}p[0-9]\+";;
    *) die 210 "count_part" "invalid argument" || return $?;;
  esac

  grep -s -c "$pattern" /proc/partitions
}

get_image_version() {
  local image_file="${1}"
  [ -n "${image_file}" ] || image_file="${IMAGE_FILE}"
  local cmd="hexdump \"${image_file}\" -n8 -s192 -v -e '8/1 \"%02x\"'"
  cmd="${cmd} | sed -e 's/\([0-9a-f]\{4\}\)/ 0x\1/g'"
  elog "cmd: ${cmd}"
  local newversion=$(printf "%0d.%0d.%0d.%0d" $(watchdog $((2*60)) true "${cmd}") | \
    sed -e 's/.[0-9]\+$//g' -e 's/\.0$//g') || \
    newversion="unknown"
  [ -n "${newversion}" ] || newversion="unknown"
  echo "${newversion}"
}

read_system_version() {
  local old_system_part="${1}"
  local old_version="none"
  if test -b "${old_system_part}" ; then
    mkdir -p /tmp/old_system
    if watchdog $((2*60)) true "mount ${old_system_part} /tmp/old_system &>/dev/null" ; then
      if test -f /tmp/old_system/usr/share/opennao/core-version.txt ; then
        old_version=$(watchdog $((2*60)) true "head -n1 /tmp/old_system/usr/share/opennao/core-version.txt")
      else
        old_version="unknown"
      fi
      watchdog $((2*60)) true "umount -f /tmp/old_system &>/dev/null"
    fi
  fi
  echo "${old_version}" | sed -e 's/\.0$//'
}

#
# partition checking utilities
#

check_image() {
  # Check installer size
  local file="${1}" offset="${2}" size="${3}"
  [ -n "${file}" ]    || die 171 "check_image" "missing arguments" || return $?
  local cmd1= cmd2= res1= res2= status=
  cmd1="hexdump '${file}' -n8 -s96 -v -e '\"0x\" 8/1 \"%02x\"'"
  elog "cmd1: ${cmd1}"
  res2=$(( ${INSTALLER_SIZE} * ${SIZE_BASE} ))
  res1=$(( $(watchdog $((5*60)) true "${cmd1}") ))
  status=$?
  elog "installer size (magic): ${res1}"
  elog "installer size (vars.): ${res2}"
  ( [ ${status} -eq 0 ] && [ -n "${res1}" ] && [ -n "${res2}" ] && \
    [ ${res1} -eq ${res2} ] ) || return 173
  elog "installer size test: PASSED"

  # Check header hash
  cmd1="dd if='${file}' bs=1 count=$(( ${MAGIC_SIZE} - 56 )) skip=56 2>/dev/null"
  cmd1="${cmd1} | sha256sum | cut -b-64"
  cmd2="hexdump '${file}' -n32 -s24 -v -e '32/1 \"%02x\"'"
  elog "cmd1: ${cmd1}"
  elog "cmd2: ${cmd2}"
  res1=$(watchdog $((5*60)) true "${cmd1}")
  status=$?
  res2=$(watchdog $((5*60)) true "${cmd2}")
  status=$(( $? + ${status} ))
  elog "magic hash computed: ${res1}"
  elog "magic hash (magic) : ${res2}"
  ( [ ${status} -eq 0 ] && [ -n "${res1}" ] && [ -n "${res2}" ] && \
    [ "${res1}" = "${res2}" ] ) || return 174
  elog "magic's hash test: PASSED"

  # Check installer hash
  cmd1="sha256sum '${INSTALLER_PATH}' | cut -b-64"
  cmd2="hexdump '${file}' -n32 -s104 -v -e '32/1 \"%02x\"'"
  elog "cmd1: ${cmd1}"
  elog "cmd2: ${cmd2}"
  res1=$(watchdog $((5*60)) true "${cmd1}")
  status=$?
  res2=$(watchdog $((5*60)) true "${cmd2}")
  status=$(( $? + ${status} ))
  elog "installer hash computed: ${res1}"
  elog "installer hash (magic) : ${res2}"
  ( [ ${status} -eq 0 ] && [ -n "${res1}" ] && [ -n "${res2}" ] && \
    [ "${res1}" = "${res2}" ] ) || return 175
  elog "installer's hash test: PASSED"

  # Check image hash
  cmd1="dd if='${file}' bs='${SIZE_BASE}' count='${size}' skip='${offset}' 2>/dev/null"
  cmd1="${cmd1} | sha256sum | cut -b-64"
  cmd2="hexdump '${file}' -n32 -s136 -v -e '32/1 \"%02x\"'"
  elog "cmd1: ${cmd1}"
  elog "cmd2: ${cmd2}"
  res1=$(watchdog $((15*60)) true "${cmd1}")
  status=$?
  res2=$(watchdog $((5*60)) true "${cmd2}")
  status=$(( $? + ${status} ))
  elog "image hash computed: ${res1}"
  elog "image hash (magic) : ${res2}"
  ( [ ${status} -eq 0 ] && [ -n "${res1}" ] && [ -n "${res2}" ] && \
    [ "${res1}" = "${res2}" ] ) || return 176
  elog "image's hash test: PASSED"

  return 0
}


#
# partition checking utilities
#

# Check that partition $1 is big enough to hold $2 kilobytes.
#
# Returns CHECK_SIZE_PART_TO_SMALL if the partition is too small, 0 otherwise.
CHECK_SIZE_PART_TOO_SMALL=162
check_size() {
  local dest_part="$(basename ${1})"
  local min_size_kb="${2}"
  elog "dest_part     : ${dest_part}"
  elog "min_size_kb   : ${min_size_kb}"
  [ -n "${dest_part}" ]   || die 160 "check_size" "missing arguments" || return $?
  [ -n "${min_size_kb}" ] || die 161 "check_size" "missing arguments" || return $?
  local dest_part_size=$(grep "${dest_part}" /proc/partitions | \
    sed -e 's/^\([^0-9]\+[0-9]\+\)\{2\}[^0-9]\+\([0-9]\+\).*$/\2/')
  elog "dest_part_size: ${dest_part_size}"
  [ ${dest_part_size} -ge ${min_size_kb} ] ||
    return "${CHECK_SIZE_PART_TOO_SMALL}"
  return 0
}

add_partition() {
  local device="${1}" part_num="${2}" part_size_kb="${3}" part_start_kb="${4}"
  local part_size= part_start=
  local cmd= status=

  elog "device        : ${device}"
  elog "part_num      : ${part_num}"
  elog "part_size_kb  : ${part_size_kb}"
  elog "part_start_kb : ${part_start_kb}"

  # sanity checks:
  [ -n "${device}" ]   || die 150 "add_partition" "missing arguments" || return $?
  [ -n "${part_num}" ] || die 151 "add_partition" "missing arguments" || return $?
  [ -b "${device}" ]   || die 152 "add_partition" "missing arguments" || return $?
  local newnode=$(get_node "${device}" "${part_num}")
  elog "newnode       : ${newnode}"
  # alter partition table
  local old_part_count=$(count_part "${device}") new_part_count=
  cmd="fdisk -u -l '${device}' | grep -i '^unit' | sed -e 's/.*\?= \([0-9]\+\) bytes$/\1/'"
  local unit_size_b=$(watchdog $((2*60)) true "${cmd}")
  status=$?
  elog "part_count    : ${old_part_count}"
  elog "unit_size_b   : ${unit_size_b}"
  [ -n "${old_part_count}" ] || die 153 "add_partition" "internal error" || return $?
  ( [ ${status} -eq 0 ] && [ -n "${unit_size_b}" ] ) || \
    die 154 "add_partition" "internal error" || return $?
  [ -z "${part_start_kb}" ]  || part_start=$(( ${part_start_kb} * 1024 / ${unit_size_b} ))
  if [ -n "$part_size_kb" ] ; then
    # if possible, calculate the exact end sector because fdisk tend to
    # "interpret" values like +2G.
    if [ -n "$part_start" ] ; then
      part_size=$(( ${part_start} + ${part_size_kb} * 1024 / ${unit_size_b} ))
    else
      part_size="+${part_size_kb}K"
    fi
  fi
  # echo "--- debug: fdisk:" ; fdisk -u -l ; echo "--- fdisk ---"
  cmd="echo -e 'n\np\n${part_num}\n${part_start}\n${part_size}\nw\n'"
  cmd="${cmd} | fdisk -u '${device}' &>/dev/null ; sync ; sync"
  watchdog $((10*60)) true "${cmd}" || return 154
  sleep 1
  mdev -s
  sleep 1
  cmd="fdisk -l '${device}' | grep '^${device}' | wc -l"
  new_part_count=$(watchdog $((2*60)) true "${cmd}") || return 155
  elog "old part count: ${old_part_count}"
  elog "new part count: ${new_part_count}"
  [ ${new_part_count} -gt ${old_part_count} ] || \
    die 156 "add_partition" "failed to alter the partition table" || return $?
  ## update the kernel partition table
  local node=$(get_node "${device}" "${part_num}")
  elog "node          : ${node}"
  cmd="fdisk -u -l '${device}' | grep '^${node}' | sed -e 's/[ ]\+/ /g'"
  local newpart=$(watchdog $((2*60)) true "${cmd}") || return 157
  elog "newpart       : ${newpart}"
  local kstart=$(( $(echo "${newpart}" | cut -d' ' -f2) * ${unit_size_b} / 512 ))
  elog "kstart        : ${kstart}"
  local ksize=$(( $(echo "${newpart}" | cut -d' ' -f4 | sed -e 's/[^0-9]//g') * 1024 / 512 ))
  elog "ksize         : ${ksize}"
  cmd="addpart ${device} ${part_num} ${kstart} ${ksize} &>/dev/null ||"
  cmd="${cmd} partx -a --nr ${part_num} ${device} ;"
  # don't fail here, the detection will take care of this if fails, just avoid to be stucked here
  cmd="${cmd} sleep 1 ; while [ ! -b ${newnode} ] ; do sleep 1 ; done"
  elog "cmd: ${cmd}"
  watchdog $((5*60)) true "${cmd}"
  [ -b "${newnode}" ] || \
    die 158 "add_partition" "new device not found as expected" || return $?
  elog "partition added: ${newnode}"
  return 0
}

# Parse fdisk's output and returns the $2'th column for partition $1, converted
# from sectors to kBs.
#
# This makes sense only for columns 3 (start), 4 (end), 5 (size).  Moreover, it
# works only with fdisk from util-linux! Busybox's fdisk shows size in 1024
# bytes blocks instead.
get_part_col_kb() {
  local node=${1}
  local col=${2}
  local device=$(get_device "${node}")
  local fdisk=$(fdisk -u -l "${device}")
  local sector_size_b=$(
    printf '%s' "${fdisk}" |
      grep -i '^unit' |
      sed -e 's/.*\?= \([0-9]\+\) bytes$/\1/'
  )
  # Column 2 is the "Boot" column. It contains a "*" if the partition is
  # bootable, and is empty otherwise. That means that if the partition is *not*
  # bootable, the second word of the line is column *3*, which would mess
  # parsing up.
  local line=$(printf %s "${fdisk}" | grep "^${node}" | sed 's/*//' )
  local sectors=$(printf %s "${line}" | awk "{ print \$${col} }")
  echo "$(( ${sectors} * ${sector_size_b} / 1024 ))"
}

get_part_start_kb() { get_part_col_kb "${1}" 2; }
get_part_end_kb() { get_part_col_kb "${1}" 3; }
get_part_size_kb() { get_part_col_kb "${1}" 4; }


# From the proc(5) man page:
#
# /proc/partitions
#        Contains the major and minor numbers of each  partition  as
#        well  as  the  number of 1024-byte blocks and the partition
#        name.
get_device_size_kb() {
  local device=${1}
  local base_device=$(basename "${device}")
  awk "/\\<${base_device}\\>/ { print \$3 }" /proc/partitions
}

delete_part() {
  local node=${1}
  local device=$(get_device "${node}")
  local nr=$(get_part_nr "${node}")

  {
    echo d
    echo "${nr}"
    echo w
  } | fdisk -u "${device}" &>/dev/null
  sync
  delpart "${device}" "${nr}"
}

# check the partition
# @param image_size_kb   image size - minimal size of the partition - (in Kbyte)
# @param part_node       node of the new partition
# @param part_size_kb    size of the new partition (in Kbyte), optional
#
check_partition() {
  local node="${1}" image_raw_size_kb="${2}"
  local part_num=$(get_part_nr "${node}")
  elog "part_num       : ${part_num}"
  # sanity checks
  elog "node             : ${node}"
  elog "image_raw_size_kb: ${image_raw_size_kb}"
  [ -n "${image_raw_size_kb}" ] || die 140 "check_partition" "missing arguments" || return $?
  [ -n "${node}" ] || die 141 "check_partition" "missing arguments" || return $?

  local node_="$(basename ${node})"
  elog "node_            : ${node}"

  local device=$(get_device "${node}") || \
    die 142 "check_partition" "internal error" || return $?

  if grep -q "${node_}$" /proc/partitions ; then
    echo -n "--> checking if the partition is large enough... "
    check_size "${node}" "${image_raw_size_kb}"
    local ret=$?

    if [ "${ret}" -eq 0 ]; then
      echo "OK"
    fi

    if [ "${ret}" -ne "${CHECK_SIZE_PART_TOO_SMALL}" ]; then
      return "${ret}"
    fi

    # The partition is smaller than the image size
    # check if we can enlarge it.
    local part_cnt=$(count_part "${device}")
    local part_start_kb="$(get_part_start_kb "${node}")"

    if [ ${part_cnt} -gt ${part_num} ]; then
      # mm there is a fourth partition
      # let see if we have enough space to enlarge the 3rd one.
      local next_node=$(get_node "${device}" "$(( ${part_num} + 1 ))")
      local usable_zone_end_kb=$(get_part_start_kb "${next_node}")
    else
      # Now see if we have enough free space on the disk
      # to create a new part.
      usable_zone_end_kb=$(get_device_size_kb "${device}")
    fi
    local usable_size_kb=$(( ${usable_zone_end_kb} - ${part_start_kb} ))

    if [ ${usable_size_kb} -lt ${image_raw_size_kb} ]; then
      return ${ret}
    fi

    # So there is a part3 too small for the image but we can
    # delete it and create it back.
    delete_part "${node}"
  fi

  local part_cnt=$(count_part "${device}")
  elog "part_cnt       : ${part_cnt}"
  [ -n "${part_cnt}" ] || die 145 "check_partition" "internal error" || return $?

  # Two cases are possible here: Either the partition was too small and it has
  # been removed, or it did not exist in the first place.
  #
  # There are only two partitions "before" the system partition, and max one
  # partition afterwards, so there are either 2 or 3 partitions.
  #
  # If the system partition is not the last or the one before, there must be a
  # logic error. In that case, bail out.
  if [ ${part_cnt} -ne $(( ${part_num} - 1 )) ] &&
     [ ${part_cnt} -ne ${part_num} ]; then
    einfo "error: unconsistent partition layout"
    return 146
  fi
  echo "adding a new partition to the table ..."
  add_partition "${device}" "${part_num}" "${image_raw_size_kb}" "${part_start_kb}" || \
    die 147 "check_partition" "calling add_partition, exitcode: ${?}" || return $?
  [ -b "${node}" ] || die 148 "check_partition" "cannot find device ${node}" || return $?
  echo "the partition table is OK"

  return 0
}


#
# flashing utilities
#


# upgrade the system
# @param archive         image archive
# @param image_size_kb   image size - minimal size of the partition - (in Kbyte)
# @param dest_part       node of the partition on which the system will be upgraded
#
upgrade_system() {
  # get destination partition properties
  local image_file="${1}"
  local image_raw_size="${2}"
  local dest_part="${3}"

  local system_version_old="$(read_system_version "${dest_part}")"
  local system_version_new="$(get_image_version "${image_file}" | tail -n1)"

  local status= do_upgrade=true
  # sanity checks
  [ -n "${image_file}" ]     || die 130 "upgrade_system" "missing argument" || return $?
  [ -n "${image_raw_size}" ] || die 133 "upgrade_system" "missing argument" || return $?
  [ -n "${dest_part}" ]      || die 135 "upgrade_system" "missing argument" || return $?

  ## pre-upgrade checks:
  status=0
  if [ ${status} -eq 0 ] ; then
    einfo ">>> Checking the image integrity... "
    check_image "${image_file}" \
      "${ROOT_IMAGE_OFFSET}" "${IMAGE_CMP_SIZE}"
    status=$?
    if [ ${status} -ne 0 ] ; then
      enotify poll "${NOTIF_IMAGE_INTEGRITY}" "${system_version_old}" "${system_version_new}"
      eerror ${status} "Image integrity checks failed."
      return ${status}
    else
      einfo "  Image integrity checks passed."
    fi
  fi
  if [ ${status} -eq 0 ] ; then
    einfo ">>> Checking the partition layout..."
    check_partition "${dest_part}" "${image_raw_size}"
    status=$?
    if [ ${status} -ne 0 ] ; then
      enotify poll "${NOTIF_PART_LAYOUT}" "${system_version_old}" "${system_version_new}"
      eerror ${status} "Partition layout checks failed."
      return ${status}
    else
      einfo "  Partitions layout checks passed."
    fi
  fi
  # system upgrade
  local dd_args="bs=${SIZE_BASE}"
  dd_args="${dd_args} skip=${ROOT_IMAGE_OFFSET}"
  [ -b "${dest_part}" ] || return 137
  killall -q -USR1 setears
  einfo ">>> Upgrading the system..."
  local dd_progress_file="/tmp/dd.log"
  local dd_loop="/tmp/.loop"
  local dd_progress=
  local cmd="dd if=${image_file} ${dd_args} count=${IMAGE_CMP_SIZE} 2>/dev/null"
  local cmd="${cmd} | gunzip -c"
  local cmd="${cmd} | dd of=${dest_part} bs=4096 count=$(( ${IMAGE_RAW_SIZE} / 4 )) 2>${dd_progress_file}"
  local cmd="${cmd} ; sync ; sync"
  elog "cmd: ${cmd}"
  cat <<EOF >${dd_loop}
#!/bin/sh
killall -q -USR1 dd
percentage=\$(tail -n1 ${dd_progress_file} | grep -oE '^[0-9]+')
[ "x\${percentage}" != x ] || percentage=1
echo \${percentage} | grep -qE '^[0-9]+$' || percentage=1
percentage=\$(( \${percentage} * 100 / ( ${SIZE_BASE} * ${image_raw_size} ) ))
[ \${percentage} -eq 0 ] && percentage=1
printf "\rwriting progress: %3d%%" \${percentage}
for i in \$(seq 1 5 \${percentage}) ; do
  setears \${i} &>/dev/null
  usleep \$(( 3000000 / \${percentage} ))
done
EOF
  chmod +x ${dd_loop}
  rm -f "${dd_progress_file}"
  watchdog $((30*60)) ${dd_loop} "${cmd}" ; status=$?
  rm -f ${dd_loop}
  sleep 1
  watchdog $((15*60)) true sync
  einfo "  System has been upgraded."
  if [ ${status} -ne 0 ] ; then
    enotify unusable "${NOTIF_WRITING_ERROR}"
    eerror ${status} "System image ($(basename "${image_file}")) and hardware mismatch" || return $?
  fi
  enotify poll "${NOTIF_UPGRADE_SUCCESS}" "${system_version_old}" "${system_version_new}"
  return 0
}

create_user_partition() {
  killall -q -USR1 setears
  setears --anime --state PREPARE_UPGRADE &>/dev/null &
  local timeout= status= cmd= dd_size_kb= mkfs_args=
  echo "${LOG_FILE}" | grep -q "${USER_DIR}" && LOG_FILE="/dev/null"
  echo "${SAY_FILE}" | grep -q "${USER_DIR}" && SAY_FILE="/dev/null"
  echo "${DBG_FILE}" | grep -q "${USER_DIR}" && DBG_FILE="/dev/null"

  # Backup home archive because if the image is on the user partition, it will be wiped.
  dd bs=${SIZE_BASE} skip="${HOME_ARCHIVE_OFFSET}" if=${IMAGE_FILE} of=/tmp/home.tar.gz count=${ARCHIVE_CMP_SIZE} 2>/dev/null

  einfo ">>> Creating user partition..."
  mount | grep -q "^${USER_PART}" && \
    umount -f "${USER_PART}" &>/dev/null
  local device="$(get_device "${USER_PART}")"
  local part_count=$(count_part "${USER_PART}")
  [ ${part_count} -le 1 ] || device="${USER_PART}"
  local part_nr=$(get_part_nr "${USER_PART}")
  if [ ${part_count} -le 1 ] ; then
    add_partition "${device}" 1
  elif [ "$part_nr" -eq 4 ] ; then
    if [ -b "$USER_PART" ] ; then
      delete_part "$USER_PART"
    fi
    local disk="$(get_device "${USER_PART}")"
    local start_kb=$((4*1024*1024)) # use a 4GB offset.
    add_partition "$disk" "$part_nr" "" "$start_kb"
    status=$?
  else
    cmd="echo -e 't\n${part_nr}\n83\nw\n' | fdisk '${device}'"
    watchdog $((5*60)) true "${cmd}"
    status=$?
  fi
  [ ${status} -eq 0 ] && \
    enotify poll "${NOTIF_FACTORY_RESET}"
  # but always notify is case of error.
  [ ${status} -eq 0 ] || \
    enotify poll "${NOTIF_FACTORY_RESET_ERROR}"
  watchdog $((15*60)) true "sync ; sync"
  # For some reason, device has been overwritten in the meantime.
  local disk="$(get_device "${USER_PART}")"
  watchdog $((1*60)) true "sfdisk --part-uuid '${disk}' ${part_nr} 66666666-1120-1120-1120-666666666666"
  watchdog $((15*60)) true "sync ; sync"
  watchdog $((1*60)) true "mkfs.ext2 -L B-Human-data '$USER_PART'"
  watchdog $((15*60)) true "sync ; sync"
  killall -q -USR1 setears
}

setup_system() {
  einfo ">>> Configuring the system..."
  mkdir -p /tmp/userpart /tmp/rootpart
  watchdog $((1*60)) true "mount '$USER_PART' /tmp/userpart"
  watchdog $((1*60)) true "mount '$DEST_PART' /tmp/rootpart"

  # Extract archive to initialize the user partition.
  gunzip -c /tmp/home.tar.gz | tar -C /tmp/userpart -x

  # Copy skeleton files from root file system.
  if [ -d /tmp/userpart/nao ]; then
    for f in .bashrc .bash_logout .profile; do
      cp /tmp/rootpart/etc/skel/${f} /tmp/userpart/nao
    done
  fi

  # Change owner of all files on the user partition
  chown -R 1001:1001 /tmp/userpart/nao

  # Configure hostname and IPs if possible.
  local hostname="$(cat /tmp/userpart/nao/.config/hostname)"
  local wiredIp="$(cat /tmp/userpart/nao/.config/wiredIp)"
  local wirelessIp="$(cat /tmp/userpart/nao/.config/wirelessIp)"
  if [ -r /tmp/userpart/nao/Config/Robots/robots.cfg ]; then
    local headId="$(/tmp/get-head-id)"
    if [ -n "${headId}" ]; then
      local thisRobot="$(grep "headId = \"${headId}\";" /tmp/userpart/nao/Config/Robots/robots.cfg)"
      if [ -n "${thisRobot}" ]; then
        hostname="$(echo ${thisRobot} | sed "s%.*name[ ]*=[ ]*\"\([^\"]*\)\";.*%\1%")"
        if [ -n "${hostname}" ]; then
          local networkFile="/tmp/userpart/nao/Config/Robots/${hostname}/network.cfg"
          if [ -r "${networkFile}" ]; then
            wiredIp="$(grep "^lan" "${networkFile}" | sed "s%.*\"\([^\"]*\)\".*%\1%")"
            wirelessIp="$(grep "^wlan" "${networkFile}" | sed "s%.*\"\([^\"]*\)\".*%\1%")"
          fi
        fi
      fi
    fi
  fi
  if [ -n "${hostname}" -a -n "${wiredIp}" -a -n "${wirelessIp}" ]; then
    sed "s%HOSTNAME_PLACEHOLDER%${hostname}%g" /tmp/rootpart/etc/hosts >/tmp/hosts.tmp && mv /tmp/hosts.tmp /tmp/rootpart/etc/hosts
    sed "s%HOSTNAME_PLACEHOLDER%${hostname}%g" /tmp/rootpart/etc/hostname >/tmp/hostname.tmp && mv /tmp/hostname.tmp /tmp/rootpart/etc/hostname
    sed "s%WIRED_IP_PLACEHOLDER%${wiredIp}%g" /tmp/rootpart/etc/netplan/default.yaml.base >/tmp/default.yaml.base.tmp && mv /tmp/default.yaml.base.tmp /tmp/rootpart/etc/netplan/default.yaml.base
    sed "s%WIRELESS_IP_PLACEHOLDER%${wirelessIp}%g" /tmp/rootpart/etc/netplan/default.yaml.wifi >/tmp/default.yaml.wifi.tmp && mv /tmp/default.yaml.wifi.tmp /tmp/rootpart/etc/netplan/default.yaml.wifi
  else
    echo "No hostname and IP set. Things will break."
    # TODO: proper error handling
  fi

  # Create initial netplan configuration.
  mv /tmp/userpart/nao/Profiles/default /tmp/rootpart/etc/netplan/default.yaml.profile
  if [ -s /tmp/rootpart/etc/netplan/default.yaml.profile ]; then
    cat /tmp/rootpart/etc/netplan/default.yaml.base /tmp/rootpart/etc/netplan/default.yaml.wifi /tmp/rootpart/etc/netplan/default.yaml.profile >/tmp/rootpart/etc/netplan/default.yaml
  else
    cp /tmp/rootpart/etc/netplan/default.yaml.base /tmp/rootpart/etc/netplan/default.yaml
  fi

  # Spawn a terminal to see if everything is correct. -> Only for testing when connected via serial port, otherwise flashing will stop here.
  # getty -n -l /bin/sh ttyS0 115200

  watchdog $((1*60)) true "umount /tmp/userpart"
  watchdog $((1*60)) true "umount /tmp/rootpart"
  rmdir /tmp/userpart /tmp/rootpart
  watchdog $((15*60)) true "sync ; sync"
}

# ======================================
#                 MAIN
# ======================================

main() {
    killall -q -USR1 setears
    setears --anime --state UPGRADE_PRECHECK &>/dev/null &
    local system_version_old=$(read_system_version "${DEST_PART}")
    local system_version_new=$(get_image_version "${IMAGE_FILE}" | tail -n1)
    einfo "------------------------------------------------------------"
    einfo "Starting upgrade to version ${system_version_new}"
    einfo "$(date -u)"
    upgrade_system "${IMAGE_FILE}" \
      "${IMAGE_RAW_SIZE}" \
      "${DEST_PART}"
    local status=$?
    [ ${status} -eq 0 ] || einfo "System upgrade exited with error code: ${status}"
    create_user_partition
    setup_system
    killall -q -USR1 setears ; setears 25 &>/dev/null
    einfo ""
    if [ -n "${HALT_AFTER_UPGRADE}" ] ; then
      umount -fr $(mount | grep '^/dev' | cut -d' ' -f3)
      sleep 2 ; sync
      /usr/libexec/chest-harakiri
      halt -f
    else
      umount -fr $(mount | grep '^/dev' | cut -d' ' -f3)
      sleep 2 ; sync
      /usr/libexec/chest-harakiri -r
    fi
    exit ${status}
}


main
exit $?
