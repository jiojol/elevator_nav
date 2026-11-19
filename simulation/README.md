# Go2 纯本机导航仿真

**当前入口只在本电脑运行。** 先验局部规划器、闭环控制器、执行适配器和 MuJoCo 都是本机进程。
不使用 SSH，不连接机器狗，不复制源码到机器狗，也不启动真实 Livox、FAST-LIO、Unitree SDK 或底盘导航服务。
ROS master 和节点使用 `127.0.0.1`；入口拒绝远端地址以及正常硬件导航使用的 11311 端口。

默认使用悬浮机身模式：70 × 35 × 20 cm 的物体按导航速度命令运动，中心位于支撑面上方 40 cm。
楼梯支撑高度在踏面中心之间连续插值，实际台阶碰撞几何和先验 FREE 高度保持不变。每次位移和转动均分段检查 MuJoCo 机身碰撞；碰撞时阻止穿透并发布接触停止信号。该模式用于验证规划、先验约束和避障，不验证真实步态或爬楼动力学。
使用 `--motion dynamic` 可运行原自由基座、关节执行器和 Go2 ONNX 步态；该模式仅在初始复位或显式重置时设置根位姿，运动由关节力驱动。
原 site1 地图未替换，各个仿真场景只写入忽略的 artifacts 目录。

## 自己选点并实时观看

```bash
cd "/home/xzq/文档/ChatGPT/xie zq/replan_in_global_map"
python3 simulation/run_interactive.py --case large_obstacle
```

会打开本机页面 `http://127.0.0.1:11882`：实时 Go2 画面、绿色先验 FREE 区、全局路线、局部曲线和实际轨迹。
在绿色区域单击指定机身目标位置，拖动指定最终朝向。到达后可继续选点；支持停止、重置和结束测试。
默认会话最长一小时，可用 `--duration` 调整；`--no-browser` 只打印地址，不自动打开浏览器。
楼梯悬浮测试使用 `python3 simulation/run_interactive.py --case stairs --motion hover`；可加 `--hover-height 0.4` 设置机身中心离支撑面的高度，单位为米。
雷达参考高度同步设为悬浮高度加标定安装高度，避免将悬浮高度变化误认为先验地形变化。页面和结果记录均标明当前运动模式。
更换场景使用 `--case stairs`、`door_065` 等，先结束当前页面的测试再启动新场景。
管理进程收到终止或挂断信号时会清理自己的子进程；即使被强制杀死，Linux 也会通知对应启动器退出。
若旧版本留下了已结束任务的节点，入口仅在进程参数、回环 master、工作区日志目录及结束记录都匹配时自动清理。活动或归属不明的节点保持不动，无需手动按名字批量杀进程。

**先验来源：** 当前箱体/门洞/楼梯仿真使用与几何场景匹配的固定 FREE 体素，不把箱体或未知地面标为可通行。
箱体场景包含一条已知通行走廊，展示先验内绕行；原 site1 地图文件保持不变，这不是把 site1 实景重建成了箱体。
复用 `map_for_plan/plan_graph.py` 的 `FreeGraph` 全局搜索，再将路线交给 SCAN；不会直接连接起终点。
目标在先验之外或机身净空不足会明确拒绝，不静默吸附到其他区域。
楼梯 FREE 位于各级踏面和上方平台的实际高度；地图用绿色覆盖通行区，并保留台阶分界和高度标注。
路线的 XY 参考点是雷达，高度按标定偏移后的机身中心所处踏面取值，避免前置雷达跨到下一台阶时让机身路径提前抬高。原始 FREE 高度和机身支撑容差不变。

界面渲染使用独立模型的状态快照；隐藏外墙不影响碰撞与雷达。悬浮模式不加载 ONNX 步态和外部 Go2 模型，雷达使用 MuJoCo 几何射线检测。

## 自动测试入口

本机依赖与编译结果已准备好。**无需额外启动 roscore，也不会询问远端密码。**

```bash
cd "/home/xzq/文档/ChatGPT/xie zq/replan_in_global_map"
python3 simulation/run_case.py --case stairs --mode native --duration 65 \
  --output "simulation/artifacts/my_stairs_$(date +%Y%m%d_%H%M%S)"
```

入口自动启动本机 `http://127.0.0.1:11791` 独立 master，结束时清理本次进程。
若操作者已经在这个回环地址启动了空 master，入口可以复用，并在结束后保留它。
先前的 `192.168.31.9:11781` 窗口与本次无关，不会被自动关闭。

可选场景：

| `--case` | 场景 |
|---|---|
| `straight` | 直行 |
| `final_yaw` | 最终朝向 |
| `large_obstacle` | 大箱体绕行 |
| `door_065` / `door_055` | 65 / 55 cm 门洞 |
| `stairs` | 12 级楼梯 |
| `dynamic_crossing` | 障碍横穿 |
| `blocking_then_clear` | 走廊挡路后移开 |

只保留 `native`（先验改进版）模式；批量入口按上表八个场景串行运行，不包含学习模型对照。

## 编译与本机依赖

仿真使用当前仓库的完整 SCAN C++ 源码，不复用远端 ARM64 可执行文件。
为兼容本机 Ubuntu 24.04，独立 CMake 入口链接本机 PCL/OpenCV 和现有 ROS Noetic 库，编译到 `simulation/artifacts/local_build`。
它不修改 `nav/devel`，不编译硬件驱动或定位模块。

```bash
python3 simulation/build_local.py
python3 simulation/build_local.py --verify
```

源码、生成消息、动态库和可执行文件使用摘要校验，源码变化后须重新编译。
`cv_bridge` 从官方 `vision_opencv` 1.16.2 固定提交 `08b012c038e575d7fe1d538f11235a994159dc93` 构建；只在生成的副本中改用本机 Boost endian 头文件，未改转换算法。
TF 库/头文件取自本机 `/opt/ros/noetic` 或现有 `~/ros/noetic_overlay/install`；PCL 转换头取自本机 Noetic 或已缓存的 RoboStack 包。
首次取依赖可能下载官方源码，**测试过程不访问远端机器人**。

物理环境保存于 `simulation/artifacts/physics_env`，继承现有 `~/miniconda3/envs/mujoco-lidar` 的 MuJoCo/ONNX 依赖。
如果该环境缺失，运行 `bash simulation/setup_local.sh`，可用 `GO2_SIM_UV` 指定已安装的 uv。
外部资产默认在 `~/代码合集/MuJoCo-LiDAR`，用 `--assets` 可指定其他位置。
其 `examples/cpp_deploy_controller.py` 还依赖本机 `unitree_cpp_deploy` 的 Go2 ONNX 策略。
迁移到另一台电脑时需要同时准备这些资产，仓库不分发外部模型。

## 结果与画面

每次保存结果、事件、运动轨迹、实际 qpos、动态障碍状态、场景、日志和本机节点 URI。
`result.json` 中的 `reached` / `timeout` / `fell` 分别表示导航到达、超时和失稳。
接触计数是采样到的接触对累计数，不是独立碰撞次数。

自动批量入口不打开实时页面。离线画面可从保存状态生成；自行选点请使用上面的交互入口。将路径换成自己的输出目录：

```bash
simulation/artifacts/physics_env/bin/uv run --no-project \
  simulation/artifacts/physics_env/bin/python simulation/render_replay.py \
  simulation/artifacts/local_only_20261028/native_stairs \
  --times 2 8 14 22 --output simulation/artifacts/stairs_replay.png
xdg-open simulation/artifacts/stairs_replay.png
```

## 测试边界与历史记录

位姿来自真值，未测试 FAST-LIO、真实噪声或丢帧。测试驱动提供全局参考路径，位置恢复最多重发两次，未运行真实全局图搜索。
使用现有 ONNX 步态，不是 Go2 Sport 控制器。动态障碍是按轨迹移动、碰撞后仍继续推挤的 mocap 刚体。
本机结果不能替代机器狗验收；在本机问题解决前不部署到机器狗。

[先前混合运行记录](reports/20261028/REPORT.md) 使用了本机物理仿真与远端规划器，仅作为历史证据。
[纯本机运行记录](reports/20261028/LOCAL_ONLY.md) 与其分开保存，不能把两组结果混算。
