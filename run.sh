#!/bin/bash


SHELL_FOLDER=$(cd "$(dirname "$0")";pwd)

${SHELL_FOLDER}/build/lidar_obstacle $@
