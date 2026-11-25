# 动态 TF、等待队列与回放时间

默认入口仍使用系统时间、静态 TF 演示和零等待查询。
动态运动、原始 `/tf` 录制、模拟时间和等待队列都通过显式参数开启。

## 完整演示步骤

每个终端先加载环境：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

终端 A 开启模拟运动和固定安装关系：

```bash
ros2 launch ros_gateserver mvp.launch.py \
    demo_tf:=true demo_motion:=true target_frame:=odom tf_wait_timeout_sec:=0.5
```

mock_sensors 使用同一份 Odometry 位姿和时间戳广播 `odom → base_link`，
独立静态节点提供 `base_link → laser_link`。这是可复现的演示轨迹，不是定位算法。
demo_motion 要求 mock=true；真实机器人保持关闭，由实际里程计或状态估计节点提供 TF。

终端 B 手动短时录制，两个选项分别检查实际写入的直接坐标关系：

```bash
ros2 run ros_gateserver record_bag \
    --with-static-tf --with-dynamic-tf --output bags/motion_demo --duration 10
```

录制仍为三路原始传感器加可选 TF，默认不常驻。
动态关系默认 odom → base_link，可用 `--dynamic-parent-frame`、
`--dynamic-child-frame` 修改。关系缺失返回失败，保留录包供排查。
检查只验证指定直接关系存在，不保证整个录制时段的 TF 覆盖完整。

关闭终端 A 中的实时演示，再在终端 B 回放：

```bash
ros2 launch ros_gateserver replay.launch.py \
    bag_path:=bags/motion_demo target_frame:=odom \
    use_sim_time:=true tf_wait_timeout_sec:=0.5
```

回放根据元数据自动选择包内 `/tf` 和 `/tf_static`，不会启动模拟运动或演示安装关系。
use_sim_time 同时开启播放器 `--clock`，并配置 Gateway 和可选自检节点。
外部算法节点仍需自行加载相同时间设置，不能假设 Launch 能配置其他进程。

需要循环时加 `loop:=true`，需要调速时加 `rate:=0.5` 或其他正数。
loop 要求 use_sim_time=true，否则启动前报错，避免历史 TF 在系统时间模式下反复倒序输入。
`clock_rate` 默认 50 Hz，须为有限正数；发布频率不改变 bag 消息内部的采集时间戳。
暂停或播放完毕后，ROS 时间可保持不变，但网关继续输出状态。

## 三类时间各管什么

| 时间 | 用途 | 暂停回放时 |
| --- | --- | --- |
| 消息 header.stamp | 对应采集时刻的 TF 查询 | 保留原始值 |
| ROS 时钟 now() | 顶层 stamp_ns，使用系统时间或播放器 /clock | 模拟时间模式下可停住 |
| steady_clock / wall timer | 新鲜度、QoS 频率估计、TF 等待上限、自检时长和心跳 | 继续计时 |

因此暂停或很慢的回放可能触发 fresh=false，自检的十秒仍指实际经过十秒。
当前不提供按模拟时间暂停自检观察窗口的模式。
启用模拟时间但尚未收到首个非零 /clock 时，clock.initialized=false，网关不提交传感器摘要。

状态新增 `clock.source`（system / ros）、`clock.initialized`、`clock.epoch`，
以及进程内递增的 `sequence`。时间暂停时 stamp_ns 可以相同；匹配具体发布应使用 sequence。
sequence 和 epoch 都不跨进程重启持久化，不能替代稳定实例标识。

## 时间回跳与并发边界

tf2::Buffer 自带时钟跳变回调，在时间源切换或回跳时清理 TF 缓存。
Gateway 的 `trackClockChanges()` 注册同样条件，只递增原子 epoch，
不在时钟内部锁下访问缓存锁或再次查询时钟。
状态输出会隐藏其他 epoch 的缓存；处理中的旧消息也不能重新提交为新时间线的数据。
摘要提交计数保持进程内累计，不随回跳归零；接收频率使用真实时间，不因 ROS 时间回跳清空。

这处理应用已缓存和正在处理的数据；不能识别所有在传输层排队、回跳后才到达的旧消息。
不应混用实时发布者与回放源。循环测试在独立 Domain 中运行，避免现场数据掩盖录包缺失。

自检仍使用真实时间计时，但观察阶段遇到 clock.epoch 变化会返回 ABORTED，
失败项为 gateway_clock_changed，避免跨时间线生成有效报告。需要检查新循环时重新提交目标。

## 非阻塞 TF 等待

tf_wait_timeout_sec 默认 0，保持零等待行为；范围为 0～5 秒，启动时只读。
大于零时，扫描先提取摘要和最近点，最多缓存 32 份，不复制原始 ranges 数组。
传感器组内的 20 ms wall timer 重试队头查询，TF 到达后按 FIFO 提交。
所有查询仍为零阻塞，没有在回调中 sleep，也不以最新 TF 替代历史查询。

超过真实等待上限时保留原始摘要，transform_status=timeout；
队列满时先提交最旧摘要并标记 queue_overflow；时间回跳时丢弃旧 epoch 的待处理扫描。
等待会增加延迟，调度和发布周期也影响外部观察到的时间，不提供硬实时截止保证。
默认关闭等待时，失败仍为 unavailable；geometry 无效和无有效点沿用原有状态。

验证：[time_tf_test.py](../tests/time_tf_test.py)，默认受控 Domain 136，隔离回放使用 137。
覆盖 TF 晚到、历史变换而非最新变换、超时、时钟未初始化、暂停时看门狗、回跳失效、
Action 中止，以及真实动态 TF 录制、数值回查、模拟时间循环恢复和 ROS / protobuf 一致性。
