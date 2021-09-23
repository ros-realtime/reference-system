#!/bin/bash

BENCHMARK_DURATION_IN_SECONDS=10

if ! [ -x "$(command -v psrecord)" ]; then
  echo psrecord is not installed, please install it with pip install -U psrecord
  exit 1
fi

# colcon build --packages-up-to reference_system

for FILE in $(find build/autoware_reference_system/ -maxdepth 1 -type f -executable); do
    BASE_FILE=$(basename ${FILE})
    echo benchmarking: ${BASE_FILE}
    psrecord ./${FILE} --include-children --plot ${BASE_FILE}_benchmark.png --duration ${BENCHMARK_DURATION_IN_SECONDS} >/dev/null 2>/dev/null
    killall ${BASE_FILE}
done
