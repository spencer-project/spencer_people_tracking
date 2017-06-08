#!/bin/sh

TARGET_DIR=$HOME/.ros/spencer_tracking_rgbd_2dlaser_example_bagfiles/
mkdir -p $TARGET_DIR

YELLOW='\033[1;33m'
NOCOLOR='\033[0m'

echo
echo "${YELLOW}Downloading example bagfiles to $TARGET_DIR ...${NOCOLOR}"
echo

cd $TARGET_DIR
wget -O bagfiles.zip http://srl.informatik.uni-freiburg.de/downloadsdir/spencer_tracking_rgbd_2dlaser_example_bagfiles.zip

echo
echo "${YELLOW}Extracting bagfiles ...${NOCOLOR}"
echo

# Not using unzip here as it may not be installed on all systems
python -c "import zipfile; zipfile.PyZipFile('bagfiles.zip').extractall()"

echo
echo "${YELLOW}Cleaning up ...${NOCOLOR}"
echo
rm bagfiles.zip
