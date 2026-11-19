#!/bin/bash
# 保存点云地图的脚本
# 使用方法: ./save_map.sh [保存路径]

# 默认保存路径
DEFAULT_PATH="/home/g/Projects/fast-lio-loop/PCD/map.pcd"

# 获取保存路径
if [ -z "$1" ]; then
    SAVE_PATH="$DEFAULT_PATH"
    echo "未指定保存路径，使用默认路径: $SAVE_PATH"
else
    SAVE_PATH="$1"
fi

# 确保目录存在
SAVE_DIR=$(dirname "$SAVE_PATH")
if [ ! -z "$SAVE_DIR" ] && [ ! -d "$SAVE_DIR" ]; then
    echo "创建目录: $SAVE_DIR"
    mkdir -p "$SAVE_DIR"
fi

# 调用 ROS 服务保存地图
echo "正在保存地图到: $SAVE_PATH"
rosservice call /save_map "save_path: '$SAVE_PATH'
resolution: 0.0"

# 检查结果
if [ $? -eq 0 ]; then
    echo "✓ 地图保存完成!"
    echo "  路径: $SAVE_PATH"
else
    echo "✗ 地图保存失败!"
    exit 1
fi

