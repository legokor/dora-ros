#!/bin/bash

DORA_PRE_PULL_AND_RUN_PWD=$PWD

# build our packages
cd /root/dora-ros &&
git pull &&
source /root/dora-ros/scripts/run.sh

cd DORA_PRE_PULL_AND_RUN_PWD

