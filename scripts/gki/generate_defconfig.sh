#!/bin/bash
# SPDX-License-Identifier: GPL-2.0-only
# Copyright (c) 2020, The Linux Foundation. All rights reserved.

# Script to generate a defconfig variant based on the input

usage() {
	echo "Usage: $0 <platform_defconfig_variant>"
	echo "Variants: <platform>-gki_defconfig, <platform>-qgki_defconfig, <platform>-consolidate_defconfig and <platform>-qgki-debug_defconfig"
	echo "Example: $0 lahaina-gki_defconfig"
	exit 1
}

if [ -z "$1" ]; then
	echo "Error: Failed to pass input argument"
	usage
fi

SCRIPTS_ROOT=$(readlink -f $(dirname $0)/)

TEMP_DEF_NAME=`echo $1 | sed -r "s/_defconfig$//"`
DEF_VARIANT=`echo ${TEMP_DEF_NAME} | sed -r "s/.*-//"`
PLATFORM_NAME=`echo ${TEMP_DEF_NAME} | sed -r "s/-.*$//"`

PLATFORM_NAME=`echo $PLATFORM_NAME | sed "s/vendor\///g"`

REQUIRED_DEFCONFIG=`echo $1 | sed "s/vendor\///g"`

# We should be in the kernel root after the envsetup
if [[  "${REQUIRED_DEFCONFIG}" != *"gki"* ]]; then
	source ${SCRIPTS_ROOT}/envsetup.sh $PLATFORM_NAME generic_defconfig
else
	source ${SCRIPTS_ROOT}/envsetup.sh $PLATFORM_NAME
fi

echo "" >> generate_defconfig.txt
echo "================ begin generate_defconfig.sh ================" >> generate_defconfig.txt
echo "" >> generate_defconfig.txt
echo "[generate_defconfig.sh] generate_defconfig.sh $1 " >> generate_defconfig.txt

KERN_MAKE_ARGS="ARCH=$ARCH \
		CROSS_COMPILE=$CROSS_COMPILE \
		REAL_CC=$REAL_CC \
		CLANG_TRIPLE=$CLANG_TRIPLE \
		HOSTCC=$HOSTCC \
		HOSTLD=$HOSTLD \
		HOSTAR=$HOSTAR \
		LD=$LD \
		"

# Allyes fragment temporarily created on GKI config fragment
QCOM_GKI_ALLYES_FRAG=${CONFIGS_DIR}/${PLATFORM_NAME}_ALLYES_GKI.config

echo "[generate_defconfig.sh] QCOM_GKI_ALLYES_FRAG=$QCOM_GKI_ALLYES_FRAG" >> generate_defconfig.txt

if [[ "${REQUIRED_DEFCONFIG}" == *"gki"* ]]; then
if [ ! -f "${QCOM_GKI_FRAG}" ]; then
	echo "Error: Invalid input"
	usage
fi
fi

FINAL_DEFCONFIG_BLEND=""

echo "[generate_defconfig.sh] REQUIRED_DEFCONFIG=$REQUIRED_DEFCONFIG" >> generate_defconfig.txt

case "$REQUIRED_DEFCONFIG" in
	${PLATFORM_NAME}-qgki-debug_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_DEBUG_FRAG"
		;&	# Intentional fallthrough
	${PLATFORM_NAME}-qgki-consolidate_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_CONSOLIDATE_FRAG"
		;&	# Intentional fallthrough
	${PLATFORM_NAME}-qgki_defconfig )
		# DEBUG_FS fragment.
		FINAL_DEFCONFIG_BLEND+=" $QCOM_DEBUG_FS_FRAG"

		FINAL_DEFCONFIG_BLEND+=" $QCOM_QGKI_FRAG"
        	echo "[generate_defconfig.sh] fragment_allyesconfig.sh QCOM_GKI_FRAG:$QCOM_GKI_FRAG QCOM_GKI_ALLYES_FRAG:$QCOM_GKI_ALLYES_FRAG" >> generate_defconfig.txt
		${SCRIPTS_ROOT}/fragment_allyesconfig.sh $QCOM_GKI_FRAG $QCOM_GKI_ALLYES_FRAG
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GKI_ALLYES_FRAG "
		;;
	${PLATFORM_NAME}-gki_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GKI_FRAG "
		;;
	${PLATFORM_NAME}-debug_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GENERIC_DEBUG_FRAG "
		;&
	${PLATFORM_NAME}_defconfig )
		FINAL_DEFCONFIG_BLEND+=" $QCOM_GENERIC_PERF_FRAG "
		;;
esac

echo "[generate_defconfig.sh] Step 1: FINAL_DEFCONFIG_BLEND: $FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
FINAL_DEFCONFIG_BLEND+=${BASE_DEFCONFIG}
echo "[generate_defconfig.sh] Step 2: FINAL_DEFCONFIG_BLEND: $FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt

# Reverse the order of the configs for the override to work properly
# Correct order is base_defconfig GKI.config QGKI.config consolidate.config debug.config
FINAL_DEFCONFIG_BLEND=`echo "${FINAL_DEFCONFIG_BLEND}" | awk '{ for (i=NF; i>1; i--) printf("%s ",$i); print $1; }'`
echo "" >> generate_defconfig.txt
echo "+++[generate_defconfig.sh]+++" >> generate_defconfig.txt
echo "[generate_defconfig.sh] after awk FINAL_DEFCONFIG_BLEND: $FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
echo "[generate_defconfig.sh] ONLY_GKI=$ONLY_GKI" >> generate_defconfig.txt

ASUS_CONFIGS_DIR="${KERN_SRC}/arch/${ARCH}/configs/vendor/"
echo "[generate_defconfig.sh] ASUS_CONFIGS_DIR=$ASUS_CONFIGS_DIR" >> generate_defconfig.txt
echo "[generate_defconfig.sh] ASUS_BUILD_PROJECT=$ASUS_BUILD_PROJECT" >> generate_defconfig.txt
echo "[generate_defconfig.sh] TARGET_BUILD_VARIANT=$TARGET_BUILD_VARIANT" >> generate_defconfig.txt

if [ "$ONLY_GKI" == "0" ]; then
if [ "$ASUS_BUILD_PROJECT" == "ZS673KS" ] ; then
	if [ "$TARGET_BUILD_VARIANT" == "userdebug" ]; then
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"_defconfig"
		echo "[generate_defconfig.sh] userdebug : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	else
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"-perf_defconfig"
		echo "[generate_defconfig.sh] user : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	fi
fi #end of if [ "$ASUS_BUILD_PROJECT" == "ZS673KS" ] ; then

if [ "$ASUS_BUILD_PROJECT" == "PICASSO" ]; then
	if [ "$TARGET_BUILD_VARIANT" == "userdebug" ]; then
		ASUS_DEFCINFIG="PICASSO_defconfig"
		echo "[generate_defconfig.sh] userdebug : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	else
		ASUS_DEFCINFIG="PICASSO-perf_defconfig"
		echo "[generate_defconfig.sh] user : ASUS_DEFCINFIG=$ASUS_DEFCINFIG" >> generate_defconfig.txt
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	fi
fi #end of if [ "$ASUS_BUILD_PROJECT" == "PICASSO" ]; then

if [ "$ASUS_BUILD_PROJECT" == "SAKE" ] || [ "$ASUS_BUILD_PROJECT" == "VODKA" ]; then
	if [ "$TARGET_BUILD_VARIANT" == "userdebug" ]; then
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"_defconfig"
		echo "[generate_defconfig.sh] userdebug : ASUS_DEFCINFIG=$ASUS_DEFCINFIG"
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	else
		echo "[generate_defconfig.sh] user : ASUS_DEFCINFIG=$ASUS_DEFCINFIG"
		ASUS_DEFCINFIG=$ASUS_BUILD_PROJECT"-perf_defconfig"
		FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_DEFCINFIG"
	fi
fi #endof if [ "$ASUS_BUILD_PROJECT" == "SAKE" ] || [ "$ASUS_BUILD_PROJECT" == "VODKA" ]; then

else #else of if [ "$ONLY_GKI" == "0" ]; then
echo "[generate_defconfig.sh] ONLY_GKI=$ONLY_GKI" >> generate_defconfig.txt
echo "[generate_defconfig.sh] ASUS_GKI_BUILD=$ASUS_GKI_BUILD" >> generate_defconfig.txt

if [ "$ASUS_BUILD_PROJECT" == "PICASSO" ]; then
	ASUS_GKI_CONFIG="PICASSO_GKI.config"
	echo "[generate_defconfig.sh] user : ASUS_GKI_CONFIG=$ASUS_GKI_CONFIG" >> generate_defconfig.txt
	FINAL_DEFCONFIG_BLEND+=" $ASUS_CONFIGS_DIR$ASUS_GKI_CONFIG"
fi #end of if [ "$ASUS_BUILD_PROJECT" == "PICASSO" ]; then

fi #end of if [ "$ONLY_GKI" == "0" ]; then

#REQUIRED_DEFCONFIG    =lahaina-qgki-debug_defconfig
echo "[generate_defconfig.sh] REQUIRED_DEFCONFIG: $REQUIRED_DEFCONFIG" >> generate_defconfig.txt
echo "[generate_defconfig.sh] defconfig blend for FINAL_DEFCONFIG_BLEND: $FINAL_DEFCONFIG_BLEND" >> generate_defconfig.txt
echo "---[generate_defconfig.sh]---" >> generate_defconfig.txt
echo "" >> generate_defconfig.txt

MAKE_ARGS=$KERN_MAKE_ARGS \
MAKE_PATH=${MAKE_PATH} \
	${KERN_SRC}/scripts/kconfig/merge_config.sh $FINAL_DEFCONFIG_BLEND
${MAKE_PATH}make $KERN_MAKE_ARGS savedefconfig
mv defconfig $CONFIGS_DIR/$REQUIRED_DEFCONFIG

# Cleanup the allyes config fragment and other generated files
#umask for debug: cp $QCOM_GKI_ALLYES_FRAG -f $QCOM_GKI_ALLYES_FRAG.backup
rm -rf $QCOM_GKI_ALLYES_FRAG .config include/config/ include/generated/ arch/$ARCH/include/generated/

echo "================ end generate_defconfig.sh ================" >> generate_defconfig.txt
echo "" >> generate_defconfig.txt
