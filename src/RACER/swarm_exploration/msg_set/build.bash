#!/bin/bash
cd spdlog
mkdir build && cd build
cmake .. && make -j

cd ../../..
catkin_make -DCATKIN_WHITELIST_PACKAGES="msg_set"
