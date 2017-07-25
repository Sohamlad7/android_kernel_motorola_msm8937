#!/bin/bash
# simple script for executing menuconfig

# root directory of Moto G5 msm8937 git repo (default is this script's location)
RDIR=$(pwd)

# directory containing cross-compile arm toolchain
TOOLCHAIN=$HOME/work/arm-eabi-4.9

export ARCH=arm
export CROSS_COMPILE=$TOOLCHAIN/bin/arm-eabi-

ABORT() {
	[ "$1" ] && echo "Error: $*"
	exit 1
}

[ -x "${CROSS_COMPILE}gcc" ] ||
ABORT "Unable to find gcc cross-compiler at location: ${CROSS_COMPILE}gcc"

while [ $# != 0 ]; do
	if [ ! "$TARGET" ]; then
		TARGET=$1
	else
		echo "Too many arguments!"
		echo "Usage: ./menuconfig.sh [target defconfig]"
		ABORT
	fi
	shift
done

[ "$TARGET" ] || TARGET=nebula

DEFCONFIG=${TARGET}_defconfig
DEFCONFIG_FILE=$RDIR/arch/$ARCH/configs/$DEFCONFIG

[ -f "$DEFCONFIG_FILE" ] ||
ABORT "Config $DEFCONFIG not found in $ARCH configs!"

cd "$RDIR" || ABORT "Failed to enter $RDIR!"

echo "Cleaning build..."
rm -rf build
mkdir build
make -s -i -C "$RDIR" O=build "$DEFCONFIG" menuconfig
echo "Showing differences between old config and new config"
echo "-----------------------------------------------------"
if command -v colordiff >/dev/null 2>&1; then
	diff -Bwu --label "old config" "$DEFCONFIG_FILE" --label "new config" build/.config | colordiff
else
	diff -Bwu --label "old config" "$DEFCONFIG_FILE" --label "new config" build/.config
	echo "-----------------------------------------------------"
	echo "Consider installing the colordiff package to make diffs easier to read"
fi
echo "-----------------------------------------------------"
echo -n "Are you satisfied with these changes? Y/N: "
read -r option
case $option in
y|Y)
	cp build/.config "$DEFCONFIG_FILE"
	echo "Copied new config to $DEFCONFIG_FILE"
	;;
*)
	echo "That's unfortunate"
	;;
esac
echo "Cleaning build..."
rm -rf build
echo "Done."
