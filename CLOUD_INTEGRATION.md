# 云平台关联导航接口（2026-11-03）

在原 FREE 全局规划和 SCAN 局部规划之上增加 `/map_manager/goal_request` 关联请求；状态/结果保留 request_id、instance_id、asset_id、map_request_id。实例 ID 和地图请求身份随进程启动变化，旧指令不能被新进程继续认领。请求校验地图、坐标系、有限位姿和单位四元数。web 所有权模式拒绝手工目标。

`/integration/navigation_asset_id` 为当前已加载的导航资产身份，`/3dnav/task_owner=web` 表示 Agent 管理任务。必须有新鲜里程计和新鲜 TRACKING 状态；仅显式 `/integration/simulation=true` 可以使用初期真值定位。实际地图装载/切换还需和云平台现有地图管理控制器对接，不能只改参数就宣称完成地图切换。

集成部署必须把原 scan_command_gate 最终 `/cmd_vel` 重映射到 `/mission/global_cmd_vel`，再由 A2 统一 mux 发布到底盘。不要把 `/planning/body_odometry` 重映射回原始 `/a2/odom`：reference_path_transform 会发布前者，这样会造成回灌，导致大量消息挤压和定位反馈过期。

MuJoCo 调用适配固定版本 3.3.7 的 mj_multiRay 签名。原独立运动学导航烟测通过；带 Piper 的共享世界中，经关联 ROS 请求、真实 FREE/SCAN、mux 和 MuJoCo 反馈完成到达。不是全身 A2 动力学，也未包含 MQTT 或抓取。导航 C++ 构建与 cloud_contract 离线测试通过。未连接实机。

## 2026-11-05: expired replan seed termination

A live stack captured during consecutive cloud tasks was stuck in
`SCANPlannerManager::reboundReplan`'s old-curve resampling: one segment point,
`pseudo_arc_length={0}`, and spacing at `denorm_min`. Unsigned `size()-2`
underflow and an unbounded minimum-seven-points retry blocked the single ROS
callback thread, including cancellation and later goals.

Only the seed boundary is changed: empty/degenerate/nonfinite old curves ask the
existing polynomial initialization to regenerate; ordinary interpolation stays
the same. Resampling has a finite retry/point bound and uses `id+1<size()`.
Native optimization, FREE, collision, dynamics and publication checks remain
mandatory. No new execution path, controller gain, or grasp algorithm change.

Regression: `system/tests/replan_seed_sampling_test.cpp` reproduces the frozen
one-point input, checks invalid/repeated vertices and compares ordinary seeds
against the previous sampler. Compile with Eigen and the plan_manage include
directory; run with an external three-second deadline. Build scan_planner with
Noetic/catkin. Integration acceptance and stack evidence are recorded separately
in the integration repository; unit/build success alone is not robot acceptance.

Rollback: revert this isolated fix, rebuild scan_planner, and keep the affected
consecutive-task feature disabled until the blocking case is addressed. This
restores the known dead-loop risk; never restart and automatically replay a
physical grasp to recover it.
