#!/bin/bash

show_help() {
    echo 'Usage:'
    echo "-f platform of project <cl|bb>"
    echo "-p project <yv35|gt>"
    echo "-v version number of fw version"
    echo "-w week of fw version"
    echo "-y year of fw version"
    echo "-s stage <evt|dev|pvt|mp>"
    echo "-i original unsinged image file name (please put in the same place)"
    echo "-o name of the output signed image"
}

while getopts "f:p:s:i:v:w:y:o:h?" opt;
do
    case "$opt" in
    h|\?)
        show_help
        exit 1
        ;;
    f)  PLATFORM=${OPTARG}
        ;;
    p)  PROJECT_NAME=${OPTARG}
        ;;
    v)  VERSION=${OPTARG}
        ;;
    w)  WEEK=${OPTARG}
        ;;
	  y)  YEAR=${OPTARG}
        ;;
	  s)  STAGE=${OPTARG}
        ;;
    i)  BIN_NAME=${OPTARG}
        ;;
    o)  OUTPUT_SIGN_IMAGE=${OPTARG}
        ;;
    esac
done


if [ -z "$PLATFORM" ] || [ -z "$PROJECT_NAME" ] || [ -z "$STAGE" ] || [ -z "$OUTPUT_SIGN_IMAGE" ] || [ -z "$BIN_NAME" ]; then
        echo 'Missing -f or -p or -s or -o or -i' >&2
        show_help
        exit 1
fi

# year week version project platform
if [ -z $VERSION ]; then
VERSION="01"
fi
if [ -z $WEEK ]; then
WEEK=$(date +%U)
fi
if [ -z $YEAR ]; then
YEAR=$(date +"%Y")
fi

# board: server_board="00001", base_board="00010", nic_exp_board="00011", rainbow_falls="00100", waimano_falls="00101"
# get board from bin name
if [ $PLATFORM = "cl" ]
then
  board="00001"
elif [ $PLATFORM = "bb" ]
then
  board="00010"
else
   echo "Error : -f fail, only support <cl|bb>"
   exit 1;
fi

# Project name
if [ $PROJECT_NAME = "yv35" ]
then
  PROJECT_NAME="Yosemite V3.5   "
elif [ $PROJECT_NAME = "gt" ]
then
  PROJECT_NAME="Grand Teton     "
else
   echo "Error : -p fail, only support <yv35|gt>"
   exit 1;
fi

echo "Project code = ${PROJECT_NAME}"

# Stage
if [ $STAGE = "evt" ]
then
  STAGE="001"
elif [ $STAGE = "dev" ]
then
  STAGE="010"
elif [ $STAGE = "pvt" ]
then
  STAGE="011"
elif [ $STAGE = "mp" ]
then
  STAGE="100"
else
  echo "Error : -s fail, only support <evt|dev|pvt|mp>"
  exit 1;
fi

if [ ! -f $BIN_NAME ]; then
    echo "Error : File $BIN_NAME not found!"
    show_help
    exit 1
fi


# Remove signed image created before
echo $BIN_NAME
if [[ -f "$OUTPUT_SIGN_IMAGE" ]]; then
    rm $OUTPUT_SIGN_IMAGE
fi
cp $BIN_NAME $OUTPUT_SIGN_IMAGE

# cal md5 and put into binary
md5=$(md5sum $OUTPUT_SIGN_IMAGE)
byte16_md5=$(echo $md5 | cut -c1-32)
echo $byte16_md5 > md5_tmp
xxd -r -p md5_tmp >> $OUTPUT_SIGN_IMAGE

# change prj code to hex and put into binary
echo $(xxd -pu <<< "$PROJECT_NAME") | cut -c 1-32 > prj_code_tmp
xxd -r -p prj_code_tmp >> $OUTPUT_SIGN_IMAGE

# FW file verison 13 byte
echo fw_version = ${YEAR}.${WEEK}.${VERSION}
byte13_fw_version=(${YEAR}.${WEEK}.${VERSION}"      ")
echo $(xxd -pu <<< "$byte13_fw_version") | cut -c 1-26 > fw_code_tmp
xxd -r -p fw_code_tmp >> $OUTPUT_SIGN_IMAGE


# component: cpld=001, bic=010, bios=011
component="010"

# Reserved for future use. if used 000000000001
INSTANCE_ID="000000000000"
echo "${INSTANCE_ID}${component}${STAGE}${board}"
errorPoof=$(echo ${INSTANCE_ID}${component}${STAGE}${board})
byte3_errorPoof=$(echo "obase=16; ibase=2; ${errorPoof}" | bc | awk '{ printf "%06d\n", $0 }')
# LSB first
echo ${byte3_errorPoof:4:2}${byte3_errorPoof:2:2}${byte3_errorPoof:0:2} > error_poof_tmp
xxd -r -p error_poof_tmp >> $OUTPUT_SIGN_IMAGE

# cal md5_2
xxd -r -p md5_tmp > md5_2_tmp
xxd -r -p prj_code_tmp >> md5_2_tmp
xxd -r -p fw_code_tmp >> md5_2_tmp
xxd -r -p error_poof_tmp >> md5_2_tmp
md5_2=$(md5sum md5_2_tmp)
byte16_md5_2=$(echo $md5_2 | cut -c1-32)
echo $byte16_md5_2 > md5_2_tmp

xxd -r -p md5_2_tmp >> $OUTPUT_SIGN_IMAGE

rm md5_tmp prj_code_tmp fw_code_tmp error_poof_tmp md5_2_tmp

echo "Signed image build success"