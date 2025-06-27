#!/bin/bash

SIM_ROOT="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"

docker build -t thaison/mobile_robots:nvidia -f ${SIM_ROOT}/docker/Dockerfile ${SIM_ROOT}