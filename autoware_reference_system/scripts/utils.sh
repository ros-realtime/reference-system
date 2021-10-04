#!/bin/bash

function check_psrecord {
if ! [ -x "$(command -v psrecord)" ]; then
  echo psrecord is not installed, please install it with pip install -U psrecord
  exit 1
fi
}

function generate_memory_data {
  [ ! -d ${3} ] && mkdir -p ${3}/
  full_log_name="${3}"
  [ ! "${full_log_name: -1}" == "/" ] && full_log_name+="/"
  full_log_name+="${4}"
  plot_name="${3}"
  plot_name="${4:-4}"
  plot_name+=".png"
  echo "Benchmarking: $1"
  echo "Duration: $2"
  echo "writing logs to: ${full_log_name}"
  psrecord $1 --include-children --log ${full_log_name} --plot ${plot_name} --duration $2
  killall $1
}