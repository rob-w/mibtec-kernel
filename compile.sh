#!/bin/sh
export ARCH=arm
export CROSS_COMPILE=arm-poky-linux-gnueabi-

make $1 CC="arm-poky-linux-gnueabi-gcc" $2
