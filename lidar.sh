#!/bin/bash

# 定义可执行文件的路径
EXEC_PATH="./build/sample/pointcloud/pointcloud_demo"
COMP_PATH="./doc/nz1_a2_angle_comp.csv"
# 执行可执行文件，并传递参数
"$EXEC_PATH" -online -ip 192.168.10.31 -p 2368 -nz1_a2 

