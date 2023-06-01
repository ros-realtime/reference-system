# syntax=docker/dockerfile:1.4
# Build with 
# TODO: Not use root! 
# Add cache for apt update.xº
ARG image_base=registry.klepsydra.io/amd64/galactic:20.04.0

ARG registry=registry.klepsydra.io/repository
ARG board=amd64
ARG osrelease=focal

FROM $image_base as system_dep
ENV BUILD_MODE=Release

USER root 

# Install kpe dependencies. VPN required.
# COPY public.gpg.key /tmp/public.gpg.key 

RUN <<EOF
# echo "deb https://${registry}/${board}/ ${osrelease} main" >> /etc/apt/sources.list.d/klepsydra.list 
# apt-key add /tmp/public.gpg.key 
# rm /tmp/public.gpg.key 
# apt update

pip3 config set global.extra-index-url https://registry.klepsydra.io/repository/kpe-pypi/simple
pip3 install kpsr-codegen
EOF

RUN mkdir -p -m 0700 ~/.ssh && ssh-keyscan github.com >> ~/.ssh/known_hosts

FROM system_dep as kpe-core
ENV KPE_CORE_TAG=v7.11.0
RUN --mount=type=ssh git clone --branch ${KPE_CORE_TAG} --recurse-submodules git@github.com:klepsydra-technologies/kpe-core

RUN --mount=type=cache,target=/home/kpsruser/.ccache <<EOF
cd /home/kpsruser/kpe-core
# Patch to make it work. 
sed -i 's/set(_SPDLOG_VERSION "1.5")/set(_SPDLOG_VERSION "1.5.0")/g' CMakeLists.txt
mkdir build 
cd build 
cmake -DKPSR_WITH_YAML=false -DKPSR_WITH_SOCKET=true -DCMAKE_BUILD_TYPE=${BUILD_MODE} -DKPSR_WITH_ROS_HUMBLE_OR_ABOVE=true \
-DKPSR_TEST_PERFORMANCE=false ../ 
make -j
sudo make install
EOF


FROM kpe-core as kpe-admin
ENV KPE_ADMIN_TAG=v6.0.1
RUN --mount=type=ssh git clone --branch ${KPE_ADMIN_TAG} --recurse-submodules git@github.com:klepsydra-technologies/kpe-admin
RUN --mount=type=cache,target=/home/kpsruser/.ccache <<EOF
cd /home/kpsruser/kpe-admin
mkdir build && cd build &&
cmake -DKPSR_WITH_SOCKET=true -DKPSR_NO_LICENSE=true -DCMAKE_BUILD_TYPE=${BUILD_MODE} ../ &&
make -j$(nproc) &&
sudo make install
EOF


FROM kpe-admin as kpe-streaming
ENV KPE_STREAMING_VER=v9.1.1
RUN --mount=type=ssh git clone --branch ${KPE_STREAMING_VER} --recurse-submodules git@github.com:klepsydra-technologies/kpe-streaming
RUN --mount=type=cache,target=/home/kpsruser/.ccache <<EOF
cd  /home/kpsruser/kpe-streaming
mkdir build && cd build &&
cmake -DCMAKE_BUILD_TYPE=${BUILD_MODE} ../ &&
make -j$(nproc) &&
sudo make install
EOF

# Install colcon, rosdep and dependencies
FROM kpe-streaming as kpe-ros2-core
ENV KPE_ROS2_CORE_VER=v1.0.0
RUN --mount=type=ssh git clone --branch ${KPE_ROS2_CORE_VER} --recurse-submodules git@github.com:klepsydra-technologies/kpe-ros2-core.git
ENV SSL_CERT_FILE=/etc/ssl/certs/ca-certificates.crt 

RUN --mount=type=cache,target=/home/kpsruser/.ccache <<EOF
pip install -U rosdep
rosdep init && rosdep update --include-eol-distros

. /opt/ros/galactic/local_setup.sh
mkdir -p /home/kpsruser/kpe-ros2-core-install/src 
ln -s /home/kpsruser/kpe-ros2-core/ /home/kpsruser/kpe-ros2-core-install/src/kpe-ros2-core
cd /home/kpsruser/kpe-ros2-core-install/ 
rosdep install --from-paths src --ignore-src -r -y  && 
colcon build --cmake-args -DCMAKE_BUILD_TYPE=${BUILD_MODE}
EOF

FROM kpe-ros2-core as reference_sytem
ENV KPE_REF_SYSTEM=update_klepsydra_executor
RUN --mount=type=ssh git clone --branch ${KPE_REF_SYSTEM} --recurse-submodules git@github.com:klepsydra-technologies/reference-system.git

RUN --mount=type=cache,target=/home/kpsruser/.ccache <<EOF
mkdir -p /home/kpsruser/colcon_reference-system/src
 
ln -s /home/kpsruser/reference-system /home/kpsruser/colcon_reference-system/src/reference-system
cd /home/kpsruser/colcon_reference-system/

. /opt/ros/galactic/local_setup.sh
colcon build --cmake-args -DCMAKE_PREFIX_PATH="/home/kpsruser/kpe-ros2-core-install/install" -DCMAKE_BUILD_TYPE=${BUILD_MODE} -DRUN_BENCHMARK=ON
EOF

# ToDO: This is not ideal. 
# Necessary.
ENV LD_LIBRARY_PATH=/home/kpsruser/kpe-ros2-core-install/install/kpsr_ros2_executor/lib:/usr/local/lib:/home/kpsruser/colcon_reference-system/install/reference_interfaces/lib:/opt/ros/galactic/lib/x86_64-linux-gnu:/opt/ros/galactic/lib
ENV PYTHONPATH=/opt/ros/galactic/lib/python3.8/site-packages:/home/kpsruser/colcon_reference-system/install/reference_system/lib/python3.8/site-packages/

# Modify entrypoint
RUN sed -i '/^exec */i source /home/kpsruser/colcon_reference-system/install/setup.sh --' /opt/ros_entrypoint.sh 

# Install dependencies.
RUN pip3 install pandas==2.0.1 bokeh==2.4.1 psrecord==1.2 numpy==1.24.3 

RUN chmod +x /home/kpsruser/reference-system/autoware_reference_system/scripts/benchmark.py
WORKDIR /home/kpsruser/reference-system/
CMD autoware_reference_system/scripts/benchmark.py 30 autoware_default_custom