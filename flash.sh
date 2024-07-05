#!/usr/bin/bash
if [ $# -lt 1 ]
then
  >&2 printf "Missing robot id\nUsage : %s ROBOT_ID\n" "${0}"
  exit 1
fi

ROBOT_ID=$1
if ! [[ ${ROBOT_ID} =~ ^[0-9]+$ ]]
then
  >&2 printf "Robot id provided is invalid : %s\n" "${ROBOT_ID}"
  exit 2
fi
# edit macro with new robot id
# note: this is better than defining the macro with `mbed compile`,
# otherwise it will recompile all of the files
sed -r -i "s/#define ROBOT_ID [0-9]+/#define ROBOT_ID ${ROBOT_ID}/g" src/motor/brushless_board.h

# compile & flash
mbed compile
sixtron_flash stm32l4a6rg BUILD/ZEST_CORE_STM32L4A6RG/GCC_ARM/robot.elf
