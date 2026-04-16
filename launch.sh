#!/bin/bash

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}   启动 Vision Kalman Filter 项目     ${NC}"
echo -e "${GREEN}========================================${NC}"

# 回到工作空间根目录
cd ~/vision_kalaman_filter_ws || exit

# 清理旧文件
echo -e "${YELLOW}清理 build/install/log 目录...${NC}"
rm -rf build/ install/ log/

# 编译
echo -e "${YELLOW}开始编译 vision_kalman_filter 包...${NC}"
colcon build --packages-select vision_kalman_filter

# 检查编译是否成功
if [ $? -ne 0 ]; then
    echo -e "${RED}编译失败！${NC}"
    exit 1
fi

echo -e "${GREEN}编译成功！${NC}"

# 设置环境变量
echo -e "${YELLOW}设置环境变量...${NC}"
source install/setup.bash

# 运行程序
echo -e "${GREEN}启动 vision_kalman_filter 节点...${NC}"
ros2 run vision_kalman_filter kalman_filter_node
