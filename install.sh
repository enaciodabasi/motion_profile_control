#!/bin/bash

#USERNAME='root'
#echo $USERNAME
while getopts h:u: option; 
do

	case "$option" in
        h)
            exit;;
        u)

            USERNAME=${OPTARG};;
    esac
done

export PROJECT_DIR="/home/$USERNAME/motion_profile_control"
echo $PROJECT_DIR

if [ -d ./build ]; then
    echo "Deleting build directory."
    rm -rf ${PROJECT_DIR}/build
fi

if [ -d /usr/local/lib/MotionProfileGenerators ]; then
    echo "Deleting shared library."
    rm -rf /usr/local/lib/MotionProfileGenerators
fi

if [ -d /usr/local/bin/MotionProfileGenerators ]; then
    echo "Deleting shared library."
    rm -rf /usr/local/bin/MotionProfileGenerators
fi

mkdir ./build

cd build

cmake ..

cmake --build . --target install
