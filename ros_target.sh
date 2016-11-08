#!/bin/bash
#
# Copyright (c) 2016 Parrot S.A.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above copyright
#     notice, this list of conditions and the following disclaimer in the
#     documentation and/or other materials provided with the distribution.
#   * Neither the name of the Parrot Company nor the
#     names of its contributors may be used to endorse or promote products
#     derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE PARROT COMPANY BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
# THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

Usage()
{
  cat << EOF
Usage: $1 [ -h ] [ -c ] [ -g ] [ -u <user> ] [ -d <device> ] -p package_name [ args...]
  -h: this help
  -c: copy to remote device
  -g: generate on remote device
  -u: remote user (default: ${SLAMDUNK_USER})
  -d: remote device (default: ${SLAMDUNK_DEVICE})
  -p: package name

Before to be used, you have to:
* initialize a ROS workspace on SLAMdunk
* initialize a ROS workspace on your PC

It's recommended to copy your public SSH key on SLAMdunk.
EOF
}

display_packages()
{
  LIST=$( cd src ; /bin/ls | cat )
  echo -n "Available packages:"
  for x in ${LIST}; do
    [ -d "src/${x}" ] && echo -n " ${x}"
  done
  echo
}

REMOTE_COPY=
REMOTE_GENERATE=
SLAMDUNK_USER="slamdunk"
SLAMDUNK_DEVICE="slamdunk-usb.local"
PACKAGE_NAME=

while getopts "hcgu:d:p:" opt; do
  case $opt in
    h)
      Usage $0
      exit 0
      ;;
    c)
      REMOTE_COPY="1"
      ;;
    g)
      REMOTE_GENERATE="1"
      ;;
    u)
      SLAMDUNK_USER=${OPTARG}
      ;;
    d)
      SLAMDUNK_DEVICE=${OPTARG}
      ;;
    p)
      PACKAGE_NAME=${OPTARG}
      ;;
  esac
done
shift $((OPTIND-1))

ARGS=$*

# check that public key is used
ssh -o PasswordAuthentication=no ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} true >/dev/null 2>&1
if [ "$?" != "0" ]; then
  echo "Please install SSH Key"
  echo "ssh-copy-id ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE}"
  exit 1
fi

CURRENT_DATE=$(date '+%s')
DEVICE_DATE=$(ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} date '+%s')
DIFF_DATE=$((${CURRENT_DATE}-${DEVICE_DATE}))
if [ "${DEVICE_DATE}" -gt "${CURRENT_DATE}" ] || [ "${DIFF_DATE}" -gt "1" ]; then
  echo "Setting SLAMDunk date ..."
  ssh -t ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} "echo slamdunk | sudo -S date -s '@$(date +%s)'"
fi

if [ -n "${REMOTE_COPY}" ]; then
  if [ ! -d "src" ]; then
    echo "Cannot find directory src"
    exit 1
  fi

  if [ -z "${PACKAGE_NAME}" ]; then
    echo "Cannot find package name (option -p)"
    display_packages
    exit 1
  fi

  if [ ! -d "src/${PACKAGE_NAME}" ]; then
    echo "Cannot find directory src/${PACKAGE_NAME}"
    display_packages
    exit 1
  fi

  ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} "true"
  if [ "$?" != "0" ]; then
    echo "Cannot reach ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE}"
    exit 1
  fi
  ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} "cd slamdunk_catkin_ws/src"
  if [ "$?" != "0" ]; then
    echo "Cannot find slamdunk_catkin_ws/src in ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE}"
    echo "  Hint: create catkin workspace \"slamdunk_catkin_ws\" on ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE}"
    echo "ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} \"source /opt/ros/indigo/setup.bash && mkdir -p slamdunk_catkin_ws/src && catkin_init_workspace\""
    exit 1
  fi
  rsync -avz src/${PACKAGE_NAME} ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE}:slamdunk_catkin_ws/src/
fi

if [ -n "${REMOTE_GENERATE}" ]; then
  ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} "[ ! -r slamdunk_catkin_ws/devel/setup.bash ] && exit 1 || exit 0"
  [ "$?" != "0" ] && echo "Remote environment not ready" && echo "Please read /opt/ros-slamdunk/share/slamdunk_node/README.md" && exit 1
  ssh ${SLAMDUNK_USER}@${SLAMDUNK_DEVICE} "cd slamdunk_catkin_ws && source devel/setup.bash && catkin_make ${ARGS}"
fi

if [ -z "${ROS_MASTER_URI}" ]; then
  echo "Warning: ROS_MASTER_URI is not set"
fi
if [ -z "${ROS_HOSTNAME}" ]; then
  echo "Warning: ROS_HOSTNAME is not set"
fi
