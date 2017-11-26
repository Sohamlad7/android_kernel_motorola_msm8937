#!/bin/bash

### Prema Chand Alugu (premaca@gmail.com)
### Shivam Desai (shivamdesaixda@gmail.com)
### Soham Lad (sohamlad7@gmail.com)
### A custom build script to build zImage,DTB & Wlan module(Anykernel2 method)

set -e

## Copy this script inside the kernel directory
git clone https://bitbucket.org/UBERTC/arm-eabi-4.9.git prebuilts/gcc/linux-x86/arm/arm-linux-eabi-UB-4.9
KERNEL_DIR=$PWD
KERNEL_TOOLCHAIN=$HOME/flyhigh/prebuilts/gcc/linux-x86/arm/arm-linux-eabi-UB-4.9/bin/arm-eabi-
KERNEL_DEFCONFIG=cedric_defconfig
DTBTOOL=$KERNEL_DIR/Dtbtool/
JOBS=8
ANY_KERNEL2_DIR=$KERNEL_DIR/AnyKernel2/
FINAL_KERNEL_ZIP=FlyHigh-R1-Cedric.zip

# Export User & Host
export KBUILD_BUILD_USER=Infixremix
export KBUILD_BUILD_HOST=FlyHigh

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


echo "**** Verify zImage
ls $KERNEL_DIR/arch/arm/boot/zImage


#Anykernel 2 time!!
echo "**** Verifying Anyernel2 Directory ****"
ls $ANY_KERNEL2_DIR
echo "**** Removing leftovers ****"
rm -rf $ANY_KERNEL2_DIR/zImage
rm -rf $ANY_KERNEL2_DIR/$FINAL_KERNEL_ZIP

echo "**** Copying zImage ****"
cp -rf $KERNEL_DIR/arch/arm/boot/zImage $ANY_KERNEL2_DIR/

cd ../
cp -rf $KERNEL_DIR/drivers/staging/prima/wlan.ko $ANY_KERNEL2_DIR/modules/

echo "**** Time to zip up! ****"
cd $ANY_KERNEL2_DIR/
zip -r9 $FINAL_KERNEL_ZIP * -x README $FINAL_KERNEL_ZIP
rm -rf $HOME/flighhigh/$FINAL_KERNEL_ZIP
cp -rf $HOME/flighhigh/AnyKernel2/$FINAL_KERNEL_ZIP $HOME/flighhigh/$FINAL_KERNEL_ZIP

echo "**** Good Bye!! ****"
cd $KERNEL_DIR
rm -rf arch/arm/boot/dtb
rm -rf $ANY_KERNEL2_DIR/$FINAL_KERNEL_ZIP
rm -rf AnyKernel2/zImage

