# Go2 三维导航

当前实机链路：Mid360 回环建图 → TUM 轨迹膨胀生成 FREE 体素 → FAST-LIO 定位 → FREE 体素 A* 全局路线 → 基于先验地图的局部重规划 → 前进/转向控制 → 速度检查 → 共用底盘桥。

当前只保留 TUM 规划地图和 FAST-LIO 定位。无碰撞时沿原全局路线执行；局部轨迹分段衔接仍会生成下一段轨迹。临时障碍阻断时保留目标并等待恢复。控制输出是机身坐标系的 `vx / yaw_rate`，横向速度为零，默认前进上限 0.8 m/s。

## 目录

| 路径 | 用途 |
|---|---|
| `system/scripts/slam/start_slam_odom.sh` | 回环建图，退出时保存 PCD、TUM 并生成规划地图 |
| `system/scripts/build_planning_map.sh` | 从建图 TUM 生成 FREE 可通行体素 |
| `system/scripts/start_nav.sh` | 导航入口，自动录制本轮数据 |
| `system/scripts/build_current.sh` | 构建当前保留的建图/导航包 |
| `system/scripts/rebuild_scan.sh` | SCAN 清理编译并更新源码/产物摘要 |
| `nav/src/map_for_plan` | FREE 地图生成、显示及 A* 全局规划 |
| `nav/src/prior_local_planner/src/planner` | 先验与实时占据、搜索、B 样条、重规划、控制器 |
| `nav/src/dog_nav_bringup` | FAST-LIO 地图定位、路径变换、速度检查及启动配置 |
| `nav/src/dog_nav_rviz_plugins` | RViz 3D 起点和目标工具 |
| `nav/src/fast_gicp` | 定位所需 CPU 点云配准 |
| `slam/src/fastlio` | 带 GTSAM 回环及轮速融合的建图程序 |
| `slam/src/FAST_LIO` | 导航时的激光惯性里程计 |
| `third_party/gtsam` | 本机 aarch64 GTSAM 4.1.1 依赖、头文件及许可证 |
| `third_party/go2_geometry.urdf` | 硬件几何及外参核对参考 |
| `system/tests` | 当前链路的开发测试，实机启动不会执行 |
| `system/map` | 定位点云、建图轨迹、FREE 图及地图元数据 |
| `system/bags/navigation_latest` | 自动保留的最近一轮导航录包和诊断信息 |

建图和导航使用的 Livox 驱动有配置与发布实现差异，分别保留在两个工作空间。底盘桥由已有外部服务提供；两个流程共用同一桥，不能重复启动速度接收者。

## 构建

本机使用 ROS Noetic、系统 Python 3 和已安装的 Livox-SDK2：

```bash
cd /home/nvidia/dog_nav_3d
export DOG_NAV_BUILD_JOBS=2
bash system/scripts/build_current.sh all
```

只重建一侧可用 `build_current.sh nav` 或 `build_current.sh slam`。`nav/rebuild_all.sh` 和 `slam/rebuild_all.sh` 转到同一构建入口。构建前关闭本工作区运行中的建图和导航节点。

GTSAM 的保留库为本机 aarch64 版本；换架构时安装或编译目标架构的 GTSAM，并通过 `GTSAM_DIR` 指定其 CMake 配置目录。不要复制本机 `devel` 目录到其他架构。

## 建图

先运行已有的底盘桥并确认 `/go2/odom` 有数据。以下命令在仓库根目录执行：

```bash
bash system/scripts/slam/start_slam_odom.sh
```

走完需要覆盖的路线后按 Ctrl-C，等待保存完成。默认生成：

```text
system/map/dog.pcd        定位点云
system/map/dog.tum        建图机身轨迹
system/map/dog_free.pcd   FREE 可通行体素
system/map/dog_path.tum   规划参考轨迹
system/map/dog_plan.json  规划地图元数据
```

新建地图默认沿轨迹左右各扩展 0.50 m，名义总宽 1.0 m，体素分辨率 0.20 m。需要重新调整通行带时：

```bash
bash system/scripts/build_planning_map.sh --half-width 0.50
```

FREE 图来自建图轨迹及其膨胀范围。实时障碍由 SCAN 占据地图处理；FREE 体素本身不能保证范围内每处都有地面。

局部规划器按 SCAN-Planner 原版流程重构：实时占据建图与双圆柱障碍膨胀 → A* 碰撞引导 → Rebound B 样条优化 → 原版时间重分配 → 预测碰撞时先重规划，失败且危险临近才急停。保存的 FREE 通行体素作为独立先验约束，通过统一占用查询进入搜索和优化，整条发布曲线额外检查先验范围。先验不清除实时障碍，也不自动扩图。全局路线用于局部目标与曲线初值，保留楼梯高度；移除了独立的参考走廊距离/路线投影否决规则。

执行适配器负责 Go2 速度转发、定位/点云/控制器超时、可靠接触输入、停车确认及最终朝向；不再读取 FREE 地图或规划器日志决定路线，也不再单独判断当前位置/转向扫掠的占据状态。停车后等待 SDK 回执、实际静止反馈及新数据，再请求规划器重新生成轨迹；失败时保持停车等待，不能反复执行相同的停车确认循环。

默认保留现有雷达参考定位、0.40 m 规划参考高度及原 site1 数据，未切换成 0.32 m 真实机身高度。已保存的安装平移在输入适配器中换算为机身中心 XY，规划、碰撞查询和跟踪共用该中心，不再随候选路径方向移动雷达杠杆臂；该水平适配并非完整三维载荷/步态标定。动态障碍采用原版双圆柱默认半径 0.25 m、偏移 ±0.18 m、上下膨胀各 0.10 m；35 cm 宽的矩形仅用于先验支撑检查，不能混称为动态障碍模型。Go2 前进/转向上限继续为 0.8 m/s、0.8 rad/s，保留速度平滑及实测进度跟踪。

实现与原版的逐项对照、保留的适配差异及操作者对比验收条件见 [局部规划器设计](nav/src/prior_local_planner/LOCAL_PLANNER_DESIGN.md)。正常导航强制启用先验，缺失 FREE 时不规划。重构只完成源码静态检查和编译，不代表已完成窄门、避障或上楼实机验收。

当前只保留先验改进版。GAT/时序图网络、训练代码、学习候选话题和软代价已从当前工作树移除，先前实验保存在 `feat/dynamic-graph-planner` 分支和整理前备份。公开入口为 `dog_nav_bringup/launch/prior_local.launch`，不再提供原版或关闭先验的对照入口。

为兼容现有 rosbag、控制桥和诊断工具，保留 `scan_planner` ROS 消息包名、节点与话题名，以及 `rebuild_scan.sh` / `verify_scan_build.py` 构建检查接口。它们运行的是同一份先验改进版。核心派生自 SCAN-Planner，来源与许可证留在 `nav/src/prior_local_planner`，没有另起一套原版导航。

## 导航

导航与建图共用 `system/map/dog.*`：定位读取 `dog.pcd`，规划读取 `dog_plan.json` 指定的 FREE 图和 TUM 参考轨迹。每次建图成功保存后，下一次导航自动使用刚生成的这套地图；新终端也采用该默认，不再默认选用 site1。缺少新地图时会报错，不自动回退旧地图。原 site1 文件保留，只有明确指定时才使用。

```bash
bash system/scripts/start_nav_standalone.sh
```

独立导航入口会临时阻止旧网页导航服务启动、停止其导航和雷达，重新加载共用底盘桥配置，并等待运行限速与配置一致后启动 FAST-LIO + 先验局部规划器。退出后解除本轮限制，不自动启动旧导航。需要 sudo 权限；只查看规划地图时不切换服务。`start_nav.sh` 保留为已完成服务切换后的底层入口。

RViz 使用 **3D Pose Estimate** 初始化定位，等待 `/dog_nav/localization_status` 为 `TRACKING`，再用 **3D Nav Goal** 设置目标。可通行地图话题是 `/plan_map/free`，全局路线是 `/plan_map/global_path`，局部 B 样条是 `/planning/bspline`。

到点默认水平容差为 0.30 m。朝向误差进入约 10° 时停止微调，停稳确认期间允许约 20° 内的朝向波动；转向期间离目标超过 0.45 m 才触发位置恢复。完成导航仍需实际静止反馈。

初始箭头只作为点云配准提示，完整导航不会把手工选点直接当作已验证的起点。初次配准失败时，会在提示附近搜索 XYZ/朝向候选，并要求连续两帧达到至少 65% 重叠、RMSE 不超过 0.18 m，且位姿相差不超过 0.10 m / 2°。地图与本轮 LIO 重力坐标允许最多 12° 差异，跟踪阶段原有的单次修正和变化速率限制保持不变。这是人工粗选点辅助初始化，不是全地图自动重定位。

静止起步时，局部曲线初值会在先验和实时占据检查下衔接当前机身朝向，避免全局路径第一段斜切通行带边缘；没有通过整条优化曲线校验的候选仍不会执行。

已有雷达驱动时入口自动复用。显式复用雷达并保持完整导航：

```bash
bash system/scripts/start_nav.sh start_lidar:=false planning_only:=false
```

仅查看规划地图和全局路径：

```bash
bash system/scripts/start_nav.sh planning_only:=true
```

起点和终点往返可加 `loop_nav:=true`。取消当前导航：

```bash
source system/scripts/env.sh
dog_nav_source_nav
rostopic pub -1 /map_manager/cancel std_msgs/Empty '{}'
```

更换地图时，启动前在当前终端设置同一组路径：

```bash
export DOG_NAV_BUILDING_PCD=system/map/site1.pcd
export DOG_NAV_MAP_PREFIX=system/map/site1
export DOG_NAV_PLAN_META=system/map/site1_plan.json
bash system/scripts/start_nav.sh
```

## 自动录包与追溯

每次经 `start_nav.sh` 启动，都会先启动 rosbag，保存到 `system/bags/navigation_latest/`。下一次通过启动检查后，只删除本工作区拥有标记的上一轮自动录包目录；手动保存的历史包不受影响。Ctrl-C 关闭导航后会等待 bag 完成索引。

目录包含传感器、里程计、全局路线、B 样条、速度与状态消息，以及 `launch.log`、`recorder.log`、参数和配置快照、地图元数据、源码/产物摘要。录包进程意外退出会终止本轮导航。

速度诊断看 `/planning/tracking_status` 中的计划速度、请求速度、转弯限速、制动限速与实际发送命令。`system/scripts/analyze_nav_recording.py`、`audit_nav_scene.py` 用于已有记录分析；`record_nav.sh` 保留手动录制和可选视频用途。

## 代码整理与备份（2026-10-29）

保留建图、FAST-LIO 定位、原 site1 地图、先验全局/局部规划、Go2 执行适配器、自动录包和本机仿真。局部实现集中在 `nav/src/prior_local_planner`；原来的 `SCAN-Planner` 目录、`scan.launch` 启动入口和学习扩展已移除。悬浮仿真仅供本电脑测试，不在机器狗上运行。

机器狗整理前的实际源码和原 Git 历史分别备份在 `/home/nvidia/dog_nav_backups/prior_cleanup_20261029_044358/source_before_cleanup.tar.gz` 与 `old_repository.bundle`。备份不移动地图、录包或运行数据；本机也有完整 Git bundle。GitHub 保留原基线标签和实验分支。

## Go2 外包络与碰撞停车

机身横向模型统一读取 `nav/src/dog_nav_bringup/config/go2_geometry.yaml`：当前长度暂按 0.70 m，宽度按用户提供的约 0.35 m，加每侧 0.03 m 余量；直行横向包络为 0.41 m。它不是 URDF 中较小的躯干盒，也不代表已测量当前机器的附件或腿部步态外缘。`geometry_verified: false` 保留未实测状态；长度、宽度、中心偏移及相对机身的上下碰撞高度需要按实物核对。矩形随朝向旋转，用分离轴判定检查每个相交栅格；先验支撑检查使用矩形；实时障碍仍使用前述双圆柱模型。RViz 的 `Go2 Body Envelope` 显示配置包络；通道是否可通行还受栅格分辨率、定位误差和 FREE 范围影响。

规划曲线预测碰撞时先重规划，危险临近且重规划失败时阻断旧速度并请求底盘 `StopMove`；外部接触信号及数据超时也会停车。SDK 成功回执之后，还要求新鲜底盘反馈连续 0.30 秒满足线速度 ≤0.03 m/s、角速度 ≤0.08 rad/s，才允许从实测位姿重新规划。停车与允许重规划走同一 FIFO 控制通道；等待期间忽略旧轨迹。找不到经过 FREE、外包络和碰撞检查的新路线时继续停车，不强行恢复或自动退让。

`/dog_nav/contact_stop` 是为已验证硬件反馈预留的 `std_msgs/Bool` 输入：true 表示机身接触仍在持续，false 表示解除；不会把普通足端接地力或含义不明的 SDK 状态位当作机身碰撞。当前没有确认自动机身触碰反馈来源，因此只能声称已实现 LiDAR 几何防碰撞及触碰输入处理，不能声称已验证真实撞击检测。若接入周期反馈，可启用 `require_contact_feedback`；信号过期也会保持停车。过程记录在 `/dog_nav/collision_status`，随本轮 bag 保存。


## 无进展恢复

控制器在有前进或转向指令、且里程计新鲜时观察实际轨迹进度和朝向。连续 3 秒未达到 5 厘米前进或 0.05 弧度转向进展，会先停止旧轨迹并记录 `/planning/execution_stall`，由执行检查请求 StopMove、等待实测停稳，再从当前位置重规划。连续恢复失败最多 3 次，随后明确失败并保持停车；实际前进超过 0.30 米或新目标会重置计数。该机制不把 SDK 成功回执当作实际运动，也不自动增加速度。

A* 保留当前地图分辨率，按查询跨度和绕行余量设置有界搜索范围，节点按需分配。被占用的碰撞分段端点只在 0.75 米邻域寻找可支撑的空闲足迹，保留起终点分离与沿线包围关系；没有合法初值时退出该轮优化，保留目标等待新的地图或路线，不继续使用缺失的碰撞引导。

## 新目标起步与转弯修正（2026-10-29）

`reference_path_transform` 发布 `/planning/body_odometry`，局部规划和跟踪的路径 XY 统一为机身中心；原始定位、全局路线、地图及垂直参考保持原契约。原地转向不再被解释成机身平移。最终输出在平滑后仍严格满足转弯/制动上限，不能让平滑过程覆盖限速。当前自动录包额外保留规划机身里程计和实时原始占据图。此修复需要新一轮实机验收，不能将离线回归或悬浮仿真视为实际防碰撞验收。

## 全局路径平滑与窄通道跟踪（2026-10-29）

全局 FREE 图路线在发布前进行受约束的 XY 平滑，保持端点和原高度进度；每条平滑边必须得到 FREE 支持。坐标适配使用实测起点，并在末端平滑衔接原始目标的固定偏移，不再将每个格点边的方向解释成传感器姿态。控制器保留平滑后的最终转弯/制动上限，但移除了用瞬时微小转向速度反推前进速度的耦合，避免直线微调引起零速。局部整条轨迹的碰撞和先验校验保持启用。


## 团队分支与联调交接（2026-11-09）

团队仓库统一使用 `main` 保存稳定基线，`feature/robot-lab` 保存当前平台联调开发；本轮不合入 main。旧分支先以 `archive/2026-11-09/<原分支名>` 标签保存再删除，全部提交历史保留。跨仓库运行以 robot-lab 的 sources.lock.json 精确 SHA 为准，不混用 main 与开发版接口。完整物理抓取及独立机器部署尚待验收。


2026-11-09 团队源码发布：当前开发统一使用 feature/robot-lab，main 保持稳定基线；旧分支以 archive/2026-11-09/<原分支名> 标签保留。Keep terrain support, terminal segment and planar progress checks consistent; retain requested map labels and restore control only for active stopped goals. Preserve default non-terrain behavior.

Checks: five terrain/controller C++ executables; cloud contract, start-height, recovery and odometry receipt Python checks passed; ROS1 integration 7 passed. No online motion or new hardware acceptance.
本轮只进行源码发布及隔离检查，不部署或运动。
