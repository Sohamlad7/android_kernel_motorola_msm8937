#!/bin/bash

### Prema Chand Alugu (premaca@gmail.com)
### Shivam Desai (shivamdesaixda@gmail.com)
### Soham Lad (sohamlad7@gmail.com)
### A custom build script to build zImage,DTB & Wlan module(Anykernel2 method)

set -e

## Copy this script inside the kernel directory
KERNEL_DIR=$PWD
KERNEL_TOOLCHAIN=$HOME/work/arm-eabi-4.9/bin/arm-eabi-
KERNEL_DEFCONFIG=nebula_defconfig
DTBTOOL=$KERNEL_DIR/Dtbtool/
JOBS=8
ANY_KERNEL2_DIR=$KERNEL_DIR/AnyKernel2/
FINAL_KERNEL_ZIP=Nebula-R1-Cedric.zip

# Export User & Host
export KBUILD_BUILD_USER=CodeZero
export KBUILD_BUILD_HOST=root

# Clean build always lol
echo "**** Cleaning ****"
make clean && make mrproper

# The MAIN Part
echo "**** Setting Toolchain ****"
export CROSS_COMPILE=$KERNEL_TOOLCHAIN
export ARCH=arm
echo "**** Kernel defconfig is set to $KERNEL_DEFCONFIG ****"
make $KERNEL_DEFCONFIG
make -j$JOBS

# Time for dtb
echo "**** Generating DT.IMG ****"
$DTBTOOL/dtbToolCM -2 -o $KERNEL_DIR/arch/arm/boot/dtb -s 2048 -p $KERNEL_DIR/scripts/dtc/ $KERNEL_DIR/arch/arm/boot/dts/qcom/

echo "**** Verify zImage,dtb & wlan****"
ls $KERNEL_DIR/arch/arm/boot/zImage
ls $KERNEL_DIR/arch/arm/boot/dtb
ls $KERNEL_DIR/drivers/staging/prima/wlan.ko

#Anykernel 2 time!!
echo "**** Verifying Anyernel2 Directory ****"
ls $ANY_KERNEL2_DIR
echo "**** Removing leftovers ****"
rm -rf $ANY_KERNEL2_DIR/dtb
rm -rf $ANY_KERNEL2_DIR/zImage
rm -rf $ANY_KERNEL2_DIR/modules/wlan.ko
rm -rf $ANY_KERNEL2_DIR/$FINAL_KERNEL_ZIP

echo "**** Copying zImage ****"
cp -rf $KERNEL_DIR/arch/arm/boot/zImage $ANY_KERNEL2_DIR/
echo "**** Copying dtb ****"
cp -rf $KERNEL_DIR/arch/arm/boot/dtb $ANY_KERNEL2_DIR/
echo "**** Copying modules ****"
cd $ANY_KERNEL2_DIR/
mkdir modules
cd ../
cp -rf $KERNEL_DIR/drivers/staging/prima/wlan.ko $ANY_KERNEL2_DIR/modules/

echo "**** Time to zip up! ****"
cd $ANY_KERNEL2_DIR/
zip -r9 $FINAL_KERNEL_ZIP * -x README $FINAL_KERNEL_ZIP
rm -rf $HOME/work/$FINAL_KERNEL_ZIP
cp -rf $HOME/work/android_kernel_motorola_msm8937/AnyKernel2/$FINAL_KERNEL_ZIP $HOME/work/$FINAL_KERNEL_ZIP

echo "**** Good Bye!! ****"
cd $KERNEL_DIR
rm -rf arch/arm/boot/dtb
rm -rf $ANY_KERNEL2_DIR/$FINAL_KERNEL_ZIP
rm -rf AnyKernel2/zImage
rm -rf AnyKernel2/dtb
rm -rf AnyKernel2/modules
