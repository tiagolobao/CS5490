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

# --------- Function Declarations -----------------


# Function - Fails execution if any build/test fails
# Arguments - $1 [ Output of last command $? ]
#           - $2 [ dir to blame ]
# Retruns   - N/A 
function check_cs5490_build {
    if [ $1 != 0 ]; then
        echo "!!FAILED!! By $2"
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

echo "BUILD SUCCESSFUL"

exit 0


