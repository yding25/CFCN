#!/bin/bash

N=100  # 要运行的次数
script="kuka_with_a_sucker.py"  # 要运行的Python脚本的名称

for ((i=1; i<=N; i++))
do
    echo "Running iteration $i"
    python3 "$script"
done