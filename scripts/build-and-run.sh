#!/bin/bash

DORA_PRE_BUILD_PWD=$PWD

source /root/dora-ros/scripts/build.sh &&
source /root/dora-ros/scripts/run.sh

cd $DORA_PRE_RUN_PWD