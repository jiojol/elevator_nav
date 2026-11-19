# Mid360 回环建图

当前保留 `map_builder_node`、`mapping_mid360_loop.launch` 和对应配置。使用 `/livox/lidar`、`/livox/imu`、`/go2/odom`，GTSAM 负责回环优化。

从仓库根目录运行：

```bash
bash system/scripts/slam/start_slam_odom.sh
```

结束时按 Ctrl-C，由外层入口保存本轮 PCD 和 TUM，并调用 `map_for_plan` 生成 FREE 可通行体素。配置位于 `config/mapping_mid360_loop.yaml`。GTSAM 库及头文件保留在根目录 `third_party/gtsam`。

保留 `/save_map` 服务及 `save_map.py` 维护工具，并保留 TUM 闭环误差统计、录包轨迹提取工具。旧隧道定位、其他雷达和离线对比入口已移出当前工作区。
