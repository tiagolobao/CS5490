#!/bin/sh

# --------------------------------------
# Author: Tiago Lobao
# Description: Building all examples and unit tests based on arduino-cli
# Continous Development
#
# Argument [$1] - arduino cli directory
# e.g. bash build_examples.sh ./arduino-cli 
# or
# e.g. bash build_examples.sh arduino-cli 
# --------------------------------------

# -------------- Argument checks -----------------

if [ $# != 1 ]; then
    echo "Wrong parameters -- try bash build_examples.sh arduino-cli"
    exit
fi

$1 version
if [ $? != 0 ]; then
    echo "unable to find cli -- try bash build_examples.sh arduino-cli"
    exit 1
fi
# make sure the cores/boards are installed
$1 core install arduino:avr
if [ $? != 0 ]; then
    echo "Boards not installed correctly"
    exit 1
fi

# --------- Function Declarations -----------------


# Function - Fails execution if any build/test fails
# Arguments - $1 [ Output of last command $? ]
#           - $2 [ dir to blame ]
# Retruns   - N/A 
function check_cs5490_build {
    if [ $1 != 0 ]; then
        echo "!!FAILED!! By $2"
        sed -i '3s/.*/[![build](https:\/\/img.shields.io\/badge\/tests-0%20passed%2C%201%20failed-red)](https:\/\/github.com\/tiagolobao\/CS5490\/releases)/' README.md
        exit 1
    fi    
}


# ----------------- Main -------------------

uno_folders=( 
    examples/On_Chip_Calibration/
    examples/Manual_Offset_Calibration/
    examples/CS5490_Basics/
    examples/CS5490_AC_Current_Gain_Tuning_demo/
    examples/Change_Baud_Rate/ 
)

mega_folders=(
    examples/Change_Baud_Rate/ 
    examples/withoutLib_HS/
)

# arduino-cli compile 'directory to compile' --fqbn 'board to build' --libraries 'local library to add'

# AVR:UNO builds
for sketch in ${uno_folders[@]}; do
    $1 compile $sketch --fqbn arduino:avr:uno --libraries "."
    check_cs5490_build $? $sketch
done

# AVR:MEGA builds
# # Checking for other boards - ESP uses the same code as Mega (Serial 2)
for sketch in ${mega_folders[@]}; do
    $1 compile $sketch --fqbn arduino:avr:mega --libraries "."
    check_cs5490_build $? $sketch
done

echo "BUILD SUCCESSFUL. updating readme"

sed -i '3s/.*/[![build](https:\/\/img.shields.io\/badge\/build-passing-brightgreen)](https:\/\/github.com\/tiagolobao\/CS5490\/releases)/' README.md

exit 0


