#!/bin/bash
# arg1: duration to run test in seconds
source $(dirname "$0")/utils.sh

check_psrecord

for FILE in $(find build/autoware_reference_system/ -maxdepth 1 -type f -executable); do
    generate_memory_data $FILE $1
done
