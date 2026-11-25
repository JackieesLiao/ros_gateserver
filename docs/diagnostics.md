# 传感器 QoS 异常诊断

网关为三路输入分别输出 `sensors.<name>.diagnostics`，ROS JSON 和 WebSocket
JSON / protobuf 使用同一份诊断快照。转发关闭时隐藏业务 data，诊断和计数仍然可见。

## 数据流和线程边界

`createInput()` 注册订阅及 QoS 不兼容事件；事件更新累计计数和最后失败策略。
`inspectPublishers()` 在控制组中最多每秒查询一次 ROS 图，对同类型发布端的实际 QoS
执行 `rclcpp::qos_check_compatible()`。图查询在缓存锁外，随后只提交端点检查结果，
不能覆盖查询期间新增的接收记录和事件。

订阅入口在处理和 TF 排队前记录本机时间；InputDiagnostics::report 在局部缓存副本上
生成状态与频率。JSON 和 protobuf 编码仍在缓存锁外执行，网络线程继续只读取载荷快照。

| 状态 | 含义 |
| --- | --- |
| `no_publisher` | 当前本地 ROS 图未发现发布端，不能直接判定设备硬件离线 |
| `type_mismatch` | 发现的发布端全部使用其他消息类型 |
| `qos_incompatible` | 所有同类型端点都已确认不兼容 |
| `qos_uncertain` | 没有已确认兼容端点，且存在无法确定的 QoS |
| `waiting_for_data` | 存在可能可用的端点，但当前时间线尚无已提交摘要 |
| `receiving` | 当前存在新鲜摘要，且端点检查没有上述阻断条件 |
| `stale` | 已有摘要超过 stale_after_sec，当前端点仍可能存在 |
| `graph_error` | 查询图失败，graph_error 字段保留错误信息 |

混合端点时，兼容发布端可以正常供数；不兼容端点数量和历史事件仍保留。
图有发现延迟，状态最多受一秒检查周期及发现延迟影响，不能作为精确网络故障定位结论。

## 字段约定

- `topic`：解析命名空间和 remap 后的订阅 Topic。
- `reliability`：当前启动选择的可靠性。
- `publishers`、`compatible_publishers`、`incompatible_publishers`：发现及兼容性统计。
- `uncertain_publishers`、`type_mismatches`：不确定及类型不同的端点数量。
- `reasons`：最多八条不兼容或不确定原因。
- `incompatible_events`、`last_policy`：RMW 累计事件数和最后失败策略；不是丢包数。
- `event_supported`：RMW 是否支持该事件；不支持时保留订阅与图检查。
- `receive_hz`：最近两秒内、最多 256 次订阅回调到达的频率估计。

频率没有样本时为 0，仅一个样本时为 null；它不是设备发布频率或丢包率。
计时发生在 TF 等待前，不把等待后批量提交误当成接收突发；执行器调度仍会影响该估计。
比较时使用订阅的 `get_actual_qos()`，避免把已经由 RMW 解析的 system_default
继续当成未知配置。图中的兼容不保证数据已被应用消费。

## 复现一次不兼容并恢复

各终端先加载 ROS 和工作区环境，并使用相同的 CycloneDDS 设置。
模拟发布者仍使用 BestEffort；将网关扫描订阅改为 Reliable：

```bash
ros2 launch ros_gateserver mvp.launch.py scan_reliability:=reliable
```

另一个终端检查：

```bash
ros2 topic info /scan --verbose
ros2 topic echo /gateway/status --field data --once
```

预期扫描没有接收数据，诊断逐步显示 qos_incompatible，事件支持时计数增长。
停止演示后移除该参数重新启动，模拟扫描恢复接收。
也支持 `image_reliability`、`odom_reliability`，合法值只有 best_effort / reliable。
这三个参数只在启动时生效，并声明为只读；不会在运行中自动重建订阅。

实现：[gateway.cpp](../src/ros_gateserver/src/gateway.cpp)、
[input_diagnostics.hpp](../src/ros_gateserver/include/ros_gateserver/input_diagnostics.hpp)。
验证：[qos_test.py](../tests/qos_test.py)，默认 Domain 135，覆盖类型错误、真实 DDS
不兼容事件、混合端点、频率、过期与恢复、转发关闭，以及 ROS / protobuf 逐字段一致性。
