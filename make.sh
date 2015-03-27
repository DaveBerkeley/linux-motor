#!/bin/bash

make ARCH=arm CROSS_COMPILE=arm-linux- -C../../gumstix-buildroot/build_arm_nofpu/linux-2.6.21gum/ M=`pwd` motors.ko
scp motors.ko root@gumstix:/tmp/

# FIN
