#!/bin/sh
#
# Helper script to build ArduCopter for multiple FMUs
# 
# sitl -- Software-in-the-loop simulator
# Pixhawk1 -- general Pixhawk1-based drones
# cmcopter -- for our own drones, which need a hack in the parsing of the PPMSum signals
# CubeBlack -- for Sparkl One
# entron300 -- for the Entron 300, which is a Pixhawk1 with some special hacks to fix the GPS autoconfig issue
# fmuv2 -- for droneshow.mx
# fmuv4 -- for FlyFire
# fmuv5 -- for the Holybro Pixhawk 4
# luminuousbee5 -- for LuminousBee5 outdoor
# luminuousbee-mini2 -- for LuminousBee Mini indoor
# CUAVv5Nano -- for jonathangentile.it
#
# You may define the BOARDS= variable in the environment to override which
# boards to build for.

BOARDS=${BOARDS:-"sitl Pixhawk1 cmcopter CubeBlack entron300 fmuv4 fmuv5 luminousbee-mini2 luminousbee5"}
ARM_TOOLCHAIN=${ARM_TOOLCHAIN:-"${HOME}/opt/toolchains/ardupilot"}

set -e

cd "`dirname $0`"

mkdir -p dist/

if [ ! -d .venv ]; then
    python2.7 -m venv .venv
    .venv/bin/pip install -U pip wheel
    .venv/bin/pip install future empy
fi

export PATH=".venv/bin:$PATH"
if [ ! -d "${ARM_TOOLCHAIN}" ]; then
    echo "/!\\ ARM toolchain suggested by the ArduPilot developers is not installed."
	while true; do
		read -p "    Do you want to continue? [y/N] " yn
		case $yn in
			[Yy]* ) break;;
			[Nn]* ) exit;;
			* ) echo ""; echo "    Please respond with yes or no.";;
		esac
	done
fi

if [ -d "${ARM_TOOLCHAIN}" ]; then
	export PATH="${ARM_TOOLCHAIN}/bin:$PATH"
fi

DATE=`date +%Y%m%d`

for BOARD in $BOARDS; do
    BOARD_LOWER=`echo $BOARD | tr [[:upper:]] [[:lower:]]`

    echo "Starting build for $BOARD..."
	echo ""

    rm -rf build/$BOARD

    if [ x$BOARD = xcmcopter ]; then
        # The IOFMU code has to be rebuilt
       ./waf configure --board=$BOARD
        CXXFLAGS=-DCOLLMOT_HACKS_FRSKY_SHORTER_FINAL_PPMSUM_PULSE Tools/scripts/build_iofirmware.py
    else
        # Discard any changes made to the vendored IOFMU
        git restore Tools/IO_Firmware/*.bin
    fi

    ./waf configure --board=$BOARD && ./waf copter
    if [ -f build/$BOARD/bin/arducopter.apj ]; then
        cp build/$BOARD/bin/arducopter.apj dist/arducopter-skybrush-${BOARD_LOWER}-$DATE.apj
    fi

    if [ x$BOARD = xcmcopter ]; then
        # Discard any changes made to the vendored IOFMU
        git restore Tools/IO_Firmware/*.bin
    fi

	echo "Finished build for $BOARD."
	echo "====================================================================="
done

echo ""
echo "Compiled firmwares are in dist/"

