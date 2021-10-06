#!/bin/bash
# arg1: duration to run test in seconds
# arg2: name of target to test
# arg3: path to store log data (directory only)
# arg4: log filename (including filetype extension, e.g. .txt)
source $(dirname "$0")/utils.sh

check_psrecord

generate_memory_data $1 $2 $3 $4
