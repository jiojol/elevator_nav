#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
保存点云地图的脚本
使用方法: python save_map.py [保存路径]
"""

import rospy
import sys
import os
from fastlio.srv import SaveMap

def save_map_client(save_path):
    """
    调用保存地图服务
    
    Args:
        save_path: 保存文件的完整路径
    """
    rospy.wait_for_service('save_map')
    try:
        save_map = rospy.ServiceProxy('save_map', SaveMap)
        print("正在保存地图到: {}".format(save_path))
        response = save_map(save_path, 0.0)
        if response.status:
            print("✓ 地图保存成功!")
            print("  路径: {}".format(save_path))
            print("  消息: {}".format(response.message))
            return True
        else:
            print("✗ 地图保存失败!")
            print("  错误: {}".format(response.message))
            return False
    except rospy.ServiceException as e:
        print("✗ 服务调用失败: %s" % e)
        return False

if __name__ == "__main__":
    rospy.init_node('save_map_client')
    
    # 获取保存路径
    if len(sys.argv) > 1:
        save_path = sys.argv[1]
    else:
        # 默认保存路径
        home_dir = os.path.expanduser("/home/g/Projects/fast-lio-loop/PCD/")
        save_path = os.path.join(home_dir, "map.pcd")
        print("未指定保存路径，使用默认路径: {}".format(save_path))
    
    # 确保目录存在
    save_dir = os.path.dirname(save_path)
    if save_dir and not os.path.exists(save_dir):
        print("创建目录: {}".format(save_dir))
        os.makedirs(save_dir)
    
    # 保存地图
    success = save_map_client(save_path)
    
    if success:
        sys.exit(0)
    else:
        sys.exit(1)

