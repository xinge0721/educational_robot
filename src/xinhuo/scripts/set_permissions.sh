#!/bin/bash

# 设置Python脚本的执行权限
echo "设置Python脚本的执行权限..."

# 获取脚本所在目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# 设置各目录下所有Python脚本的权限
chmod +x "$SCRIPT_DIR"/*.py
chmod +x "$SCRIPT_DIR"/http/*.py
chmod +x "$SCRIPT_DIR"/websocket/*.py
chmod +x "$SCRIPT_DIR"/common/*.py

echo "权限设置完成！" 