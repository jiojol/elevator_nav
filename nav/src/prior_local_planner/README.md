# 基于先验地图的局部规划器

当前工作区唯一保留的局部规划实现。先验 FREE 与实时障碍占据共同约束 A* 搜索、Rebound B 样条优化和整条发布曲线；跟踪器与 Go2 执行适配器负责速度衔接、停车确认、接触输入和最终朝向。

- 启动：`system/scripts/start_nav_standalone.sh` → `dog_nav_bringup/launch/prior_local.launch`。
- 参数：`dog_nav_bringup/config/prior_local.yaml` 与 `config/go2_geometry.yaml`。
- 构建：`system/scripts/build_current.sh nav`。
- 结构：`plan_env` 占据/先验，`path_searching` 搜索，`bspline_opt` 优化，`traj_utils` 轨迹工具，`plan_manage` 状态机/跟踪。
- 没有原版对照入口，也没有学习规划扩展；FREE 在公开启动入口中强制启用，缺失时保持等待。

为兼容已有 rosbag、控制桥和诊断工具，内部 ROS 包、消息类型、节点和话题仍沿用 `scan_planner` 等旧名字。这是同一份先验改进实现，不是另外启动原版。

核心派生自 [SCAN-Planner](https://github.com/wuyi2121/SCAN-Planner) 提交 `348e8a590a50a5a6bbab8d8c6dcfd171f009be26`；来源与 Apache-2.0 许可证保留，见 [LICENSE](LICENSE)、`native_core_manifest.json` 和 [设计说明](LOCAL_PLANNER_DESIGN.md)。本次整理不宣称算法从零实现或已经完成实机通行验收。
