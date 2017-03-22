#!/bin/sh

SCRIPT_DIR=`dirname $0`
YELLOW='\033[1;33m'
NOCOLOR='\033[0m'

echo
echo "${YELLOW}Downloading example bagfiles to $SCRIPT_DIR ...${NOCOLOR}"
echo

cd $SCRIPT_DIR
wget -O bagfiles.zip http://srl.informatik.uni-freiburg.de/downloadsdir/spencer_tracking_rgbd_2dlaser_example_bagfiles.zip

echo
echo "${YELLOW}Extracting bagfiles ...${NOCOLOR}"
echo
unzip bagfiles.zip

echo
echo "${YELLOW}Cleaning up ...${NOCOLOR}"
echo
rm bagfiles.zip
