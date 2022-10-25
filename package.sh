#!/bin/bash

set -e

root=$(pwd)

# Jenkins sets both of these but for non-jenkins builds try to do
# the right thing...
if [ -z "$WORKSPACE" ]; then
    WORKSPACE=$root
fi
build_num="$BUILD_NUMBER"
if [ -z "$build_num" ]; then
    build_num="0"
fi

rm -rf packages
mkdir -p packages

rosdep update

# Taken from https://stackoverflow.com/a/10467453
function sedeasy {
    sed -i "s/$(echo $1 | sed -e 's/\([[\/.*]\|\]\)/\\&/g')/$(echo $2 | sed -e 's/[\/&]/\\&/g')/g" $3
}

function build_and_install_package {
    cd $WORKSPACE/src/$1
    rosdep install --from-paths . --ignore-src -y
    sedeasy "$2" "$3" package.xml
    sedeasy "NO_URL" "$GIT_FULL_URL" package.xml
    rm -rf obj-* debian build
    echo "--------------------------------------"
    echo "Building $1"
    echo "--------------------------------------"
    bloom-generate rosdebian .
    fakeroot make -d -f debian/rules binary
    rm -rf obj-* debian build
    cd $WORKSPACE
    mv src/*.deb packages
    echo "--------------------------------------"
    echo "Installing $1 (and other packages)"
    echo "--------------------------------------"
    sudo dpkg -i packages/*.deb
}

build_and_install_package "bosch_radar_sda_msgs" "0.0.0" "0.1.$build_num"
build_and_install_package "bosch_radar_sda" "0.0.0" "0.1.$build_num"
