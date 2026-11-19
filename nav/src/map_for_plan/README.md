# TUM FREE 体素规划

`system/scripts/build_planning_map.sh` 从建图 TUM 轨迹生成 FREE 体素和规划参考路径。`global_planner.py` 在体素上执行 A*，将结果发布到 `/plan_map/global_path`，交给 SCAN 局部规划。

默认体素分辨率为 0.20 m，沿轨迹左右各扩展 0.50 m（名义总宽 1.0 m）；实际边界按体素网格离散。可通过 `build_planning_map.sh --half-width` 覆盖默认值。

输入 `/initialpose`、`/move_base_simple/goal` 和真实 `/dog_nav/odometry`。显示话题为 `/plan_map/free`、`/plan_map/path`。取消使用 `/map_manager/cancel`；状态和结果使用 `/map_manager/status`、`/map_manager/result`。

完整导航的全局起点与局部规划共用 `/planning/body_odometry` 机身参考，避免从雷达位置吸附后产生起步短折返。仅显示规划时仍允许原始定位或手工起点。

完整导航启动与自动录包由根目录 `system/scripts/start_nav.sh` 负责。仅查看规划可传 `planning_only:=true`；起终点往返可传 `loop_nav:=true`。FREE 通行带只覆盖采集轨迹及膨胀范围，实时碰撞避障由 SCAN 完成。

全局 A* 路线发布前按不超过半体素的间距重采样，并在 FREE 体素约束内平滑 XY；起终点固定，Z 按原路线进度插值，保留楼梯/楼层高度。平滑的每条边都采样检查 FREE 支持，局部规划仍负责完整机身和实时障碍校验。坐标适配保留中间点的几何位置，仅在末端平滑衔接固定目标偏移，避免按每个栅格转角重复旋转雷达偏移造成锯齿。
