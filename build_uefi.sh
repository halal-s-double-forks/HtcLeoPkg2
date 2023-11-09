#!/bin/bash

# Function to display Help Message
function _help(){
	echo "Usage: ./build_uefi.sh -d <Codename> [-r <Build Mode>]"
	echo
	echo "Build Project Mu UEFI for Apple Devices."
	echo
	echo "Options:"
	echo "	--device <Codename>, -d <Codename>:         Build a Device."
	echo "	--release <Build Mode>, -r <Build Mode>:    Release mode for building, 'RELEASE' is the default or use 'DEBUG' alternatively."
	echo "	--help, -h:                                 Shows this Help."
	echo
	echo "MainPage: https://github.com/HTC-Leo-Revival-Project/HtcLeoPkg"
	exit 1
}

# Functions to display the Message Type (Error or Warning)
function _error(){ echo -e "\033[1;31m${@}\033[0m" >&2;exit 1; }
function _warn(){ echo -e "\033[0;33m${@}\033[0m" >&2; }

# Set Default Defines
TARGET_BUILD_MODE="DEBUG"

# Check if any args were given
OPTS="$(getopt -o d:hfabcACDO:r: -l device:,help,release:: -n 'build_uefi.sh' -- "$@")"||exit 1
eval set -- "${OPTS}"
while true
do	case "${1}" in
		-d|--device) TARGET_DEVICE="${2}";shift 2;;
		-h|--help) _help 0;shift;;
		-r|--release) TARGET_BUILD_MODE="${2}";shift 2;;
		--) shift;break;;
		*) _help 1;;
	esac
done

# If no Device arg is present, Display the Help Message
if [ -z ${TARGET_DEVICE} ]
then _help
fi

# Set Release Type of UEFI
case "${TARGET_BUILD_MODE}" in
	RELEASE) _TARGET_BUILD_MODE=RELEASE;;
	*) _TARGET_BUILD_MODE=DEBUG;;
esac

# Include Device Config if it exists
if [ -f "configs/${TARGET_DEVICE}.conf" ]
then source "configs/${TARGET_DEVICE}.conf"
else _error "\nDevice configuration not found!\nCheck if your .conf File is in the 'configs' Folder\n"
fi

# Delete Output Files if present
rm ./BootShim/BootShim.bin &> /dev/null
rm ./BootShim/BootShim.elf &> /dev/null
rm ./ImageResources/Tools/bootpayload.bin &> /dev/null

# Compile BootShim
cd BootShim
make UEFI_BASE=${TARGET_FD_BASE} UEFI_SIZE=${TARGET_FD_SIZE}||_error "\nFailed to Compile BootShim!\n"
cd ..

# Setup and Update UEFI workspace
stuart_setup -c "HtcLeoPkg/PlatformBuild.py" TOOL_CHAIN_TAG=CLANG38||_error "\nFailed to Setup UEFI Env!\n"
stuart_update -c "HtcLeoPkg/PlatformBuild.py" TOOL_CHAIN_TAG=CLANG38||_error "\nFailed to Update UEFI Env!\n"

# Copy fixed tools_def.txt to Build Folder
rm ./Conf/tools_def.txt &> /dev/null
cp ./configs/tools_def.txt ./Conf/ &> /dev/null

# Start the UEFI Build "FD_BASE=${TARGET_FD_BASE}" "FD_SIZE=${TARGET_FD_SIZE}" "FD_BLOCKS=${TARGET_FD_BLOCKS}"||
stuart_build -c "HtcLeoPkg/PlatformBuild.py" TOOL_CHAIN_TAG=CLANG38 "TARGET=${_TARGET_BUILD_MODE}"||_error "\nFailed to Compile UEFI!\n"
# Copy UEFI FD File to a Payload File
cat ./BootShim/BootShim.bin "./Build/${TARGET_DEVICE}Pkg/${_TARGET_BUILD_MODE}_CLANG38/FV/${TARGET_DEVICE^^}_UEFI.fd" > "Mu-${TARGET_DEVICE}.bin"||exit 1
