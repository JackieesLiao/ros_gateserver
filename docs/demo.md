# 构建、演示与配置

## 构建与启动

构建验证使用 Ubuntu 24.04、ROS 2 Jazzy 和 CycloneDDS。以下命令从仓库根目录执行。

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 首次使用可通过 rosdep 安装 package.xml 声明的依赖
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to ros_gateserver --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch ros_gateserver mvp.launch.py
```

默认启动模拟传感器和网关。模拟数据为 10 Hz，网关状态为 5 Hz。
WebSocket 地址：`ws://127.0.0.1:8765`，默认仅监听本机。

另开终端，加载 ROS 与 `install/setup.bash`，同样设置 `RMW_IMPLEMENTATION` 后检查状态：

```bash
ros2 topic echo /gateway/status --once
ros2 service call /gateway/set_enabled std_srvs/srv/SetBool '{data: false}'
ros2 service call /gateway/set_enabled std_srvs/srv/SetBool '{data: true}'
```

也可以在浏览器空白页的开发者控制台连接：

```javascript
const ws = new WebSocket('ws://127.0.0.1:8765');
ws.onmessage = event => console.log(JSON.parse(event.data));
// 结束时：ws.close();
```

接入真实传感器时关闭模拟节点，并指定已有 Topic：

```bash
ros2 launch ros_gateserver mvp.launch.py mock:=false \
  image_topic:=/camera/image_raw scan_topic:=/scan odom_topic:=/odom
```

Launch 还支持 `ws_host`、`ws_port`、`ws_format`（默认 `json`）、
`self_check`（默认 `false`）、`target_frame`（默认 `base_link`）参数。
新增 `image_reliability`、`scan_reliability`、`odom_reliability`，默认 best_effort，
也可选择 reliable。诊断输出位于 `sensors.<name>.diagnostics`，区分发布者缺失、
类型错误、QoS 不兼容、不确定和数据过期，并包含接收频率与事件计数。
复现和字段说明见 [QoS 异常诊断](diagnostics.md)。
网关节点支持 `publish_period_ms`（默认 200）、
`stale_after_sec`（默认 2.0），可在 `ros2 run` 时通过 `--ros-args -p` 设置。

## 自检 Action 与 protobuf

现在包含两个 ROS 包：`ros_gateserver_interfaces` 生成自检 Action 类型，
`ros_gateserver` 提供节点、protobuf 协议及解码客户端。构建使用上面的
`--packages-up-to ros_gateserver`，会先构建接口包。
新增依赖由 package.xml 声明，包含 rclcpp_action、rosidl、protobuf 编译器、开发库和 Python 运行库。

一条命令开启模拟传感器、静态 TF、自检 Action 和 protobuf WebSocket：

```bash
ros2 launch ros_gateserver mvp.launch.py \
    demo_tf:=true self_check:=true ws_format:=protobuf
```

另一个已加载环境的终端提交自检目标，观察反馈和结果：

```bash
ros2 action send_goal /gateway/check_sensors \
    ros_gateserver_interfaces/action/CheckSensors \
    '{duration_sec: 10.0, require_scan_tf: true}' --feedback
```

自检观察 1～60 秒，提供进度、取消、检查报告和网关失联检测。
`SUCCEEDED` 表示检查执行完成，是否通过必须读取 `result.passed`。
正常检查也可能返回 `passed=false`；心跳超时等执行故障返回 `ABORTED`。
观察时间与等待网关准备时间分开计算，参数和失败项见 [Action 说明](actions.md)。

解码 WebSocket 二进制状态并显示为可读 JSON：

```bash
ros2 run ros_gateserver protobuf_client --count 3
```

默认 `ws_format:=json` 继续兼容原客户端。protobuf 模式中 `/gateway/status` 仍发布 JSON，
自检节点照常工作；每个网关实例的全部 WebSocket 连接使用同一种格式。
`replay.launch.py` 也支持 `ws_format` 和 `self_check`。
协议定义、字段缺失语义和客户端实现见 [protobuf 说明](protocol.md)。

## gRPC 状态订阅

构建依赖新增 `libgrpc++-dev`、`protobuf-compiler-grpc`，演示客户端使用 `python3-grpcio`。
可通过上面的 `rosdep install` 安装。

```bash
ros2 launch ros_gateserver mvp.launch.py demo_tf:=true grpc_enabled:=true
ros2 run ros_gateserver grpc_client --count 3
```

分别在两个已加载环境的终端运行。`GatewayStatusService.WatchStatus` 返回当前快照，
随后流式发送更新。可用 `grpc_host` 和 `grpc_port` 修改监听地址，默认 `127.0.0.1:50051`。
gRPC 默认关闭，启用后使用明文连接，与 `ws_format` 独立配置。
`replay.launch.py` 接受同样的 gRPC 参数。接口和推送语义见 [状态协议](protocol.md#grpc-状态流)。

## 固定雷达 TF 演示

显式开启演示安装关系，一条命令启动模拟传感器、网关和静态 TF 发布器：

```bash
ros2 launch ros_gateserver mvp.launch.py demo_tf:=true
```

演示关系固定为 `base_link -> laser_link`，默认雷达在车体前方 0.2 米、朝向相同。
安装参数为 `laser_x`、`laser_y`、`laser_z`（米）及
`laser_roll`、`laser_pitch`、`laser_yaw`（弧度）。例如增加绕 Z 轴 90 度旋转：

```bash
ros2 launch ros_gateserver mvp.launch.py \
    demo_tf:=true laser_x:=0.2 laser_yaw:=1.5707963267948966
```

真实机器人已有 TF 时保持 `demo_tf:=false`，避免重复提供同一安装关系。
修改 `target_frame` 仅改变网关的转换目标，不会创建额外 TF；对应链路须由外部提供。
启动初期 TF 尚未到达时允许短暂 `unavailable`，后续扫描收到后自动恢复。

## OpenCV 图像质量诊断

默认只提取元信息，quality.status=disabled。显式开启后，在原元信息之外输出质量摘要：

```bash
ros2 launch ros_gateserver mvp.launch.py image_quality_enabled:=true
```

查看 sensors.image.data.quality，其中包含亮度均值、像素标准差、Laplacian 方差及阈值提示。
分析支持 mono8、rgb8、bgr8、rgba8、bgra8，默认将最长边限制为 320 后计算，
不修改原始像素，不向 WebSocket 发送图像。坏帧以明确状态报告，后续合法帧可恢复。
这是一组可标定的启发式指标，不直接判断硬件故障；status=ok 表示计算完成。
回放入口支持相同参数。配置、数据流和边界见 [图像质量说明](image_quality.md)。

## 演示时录制与回放

录制由用户单独启动，默认录制 10 秒。`mvp.launch.py` 和回放入口都不会自动录制。
以下命令均在工作区根目录执行，每个终端先加载 ROS、`install/setup.bash`，并设置
`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`。

1. 终端 A 启动模拟传感器和网关：

   ```bash
   ros2 launch ros_gateserver mvp.launch.py
   ```

2. 终端 B 录制三路原始消息，等脚本输出每路消息计数后再继续：

   ```bash
   ros2 run ros_gateserver record_bag --output bags/demo --duration 10
   ros2 bag info bags/demo
   ```

   输出为 `bags/demo/metadata.yaml` 和 MCAP 数据文件。原始图像像素、扫描数组、
   里程计消息及其时间戳都会保存。已有目录会被拒绝，重复演示时使用新的目录名；
   省略 `--output` 会在 `bags/` 下自动创建带时间戳的目录。`bags/` 已加入 `.gitignore`。
   Ctrl+C 可提前结束，脚本仍会等待录制器完成文件收尾。任一路没有消息时返回失败。

3. 在终端 A 按 Ctrl+C 关闭原演示，再启动回放：

   ```bash
   ros2 launch ros_gateserver replay.launch.py bag_path:=bags/demo
   ```

   回放入口启动网关和播放器，模拟节点关闭；确认其他终端也没有运行模拟发布者。
   仍可通过 `ws://127.0.0.1:8765` 和 `/gateway/status` 观察输出。
   播放完成后网关继续运行，超过新鲜度阈值后各路 `fresh` 变为 `false`，最后按 Ctrl+C 退出。

回放参数：`rate` 为速度倍数，默认 `1.0`；`start_delay` 默认 `2.0` 秒，为启动和
DDS 发现留出时间。两者也可与 `ws_host`、`ws_port`、三路 Topic 参数一起传入。
录制脚本的 `--topics` 支持指定完整 Topic 名称；录制自定义 Topic 后，回放时须传入
对应的 `image_topic`、`scan_topic`、`odom_topic`。

需要离线复现雷达坐标转换时，终端 A 使用上面的 `demo_tf:=true` 启动，终端 B 显式录入静态 TF：

```bash
ros2 run ros_gateserver record_bag --with-static-tf --output bags/tf_demo --duration 10
```

此选项在三路传感器之外录入 `/tf_static`，结束后确认实际消息包含
`base_link -> laser_link`。录制其他直接安装关系时可指定
`--tf-parent-frame` 和 `--tf-child-frame`；当前检查不支持用多段链路替代指定的直接关系。
缺少消息或关系不匹配时命令返回失败，保留文件便于诊断，不宣称录制成功。

在终端 A 按 Ctrl+C 关闭实时演示和 TF 发布器，再回放：

```bash
ros2 launch ros_gateserver replay.launch.py bag_path:=bags/tf_demo
```

回放自动选择包内的 `/tf_static`，关闭模拟节点和演示 TF，不重新生成安装参数。
录制与回放共用 `config/rosbag_qos.yaml`，保证晚加入者可以接收保留的静态 TF。
旧包不含 TF 时会提示；没有外部 TF 的情况下，摘要照常输出、转换状态为 `unavailable`。
演示应关闭其他实时发布者；自动测试使用另一 ROS Domain 回放，以排除现场数据干扰。
以上命令保持默认系统时间模式。动态 `/tf` 录放和模拟时间由下方选项显式开启。

这是一项短时演示工具，默认只处理当前三路消息。它保留消息中的原始时间戳，
默认网关输出时间仍为当前系统时间；新鲜度按本机接收间隔计算。低速回放中的长间隔也可能
触发过期状态。实现与底层机制见 [录制回放代码说明](recording.md)。

## 动态 TF 和回放时间

启动演示运动，模拟节点使用原始 Odometry 位姿和时间戳广播 odom → base_link：

```bash
ros2 launch ros_gateserver mvp.launch.py \
    demo_tf:=true demo_motion:=true target_frame:=odom tf_wait_timeout_sec:=0.5
```

另一个已加载环境的终端手动录制：

```bash
ros2 run ros_gateserver record_bag \
    --with-static-tf --with-dynamic-tf --output bags/motion_demo --duration 10
```

关闭实时演示后，用录包中的动态与静态 TF 回放：

```bash
ros2 launch ros_gateserver replay.launch.py \
    bag_path:=bags/motion_demo target_frame:=odom \
    use_sim_time:=true tf_wait_timeout_sec:=0.5
```

use_sim_time 同时配置网关时钟和播放器 /clock；循环可加 `loop:=true`，
loop 要求模拟时间。默认不启用这两项，沿用已有回放行为。
ROS 时间回跳使旧缓存失效；状态包含 clock 和进程内递增 sequence，便于识别时间线和暂停时的不同帧。
新鲜度、自检和等待上限仍用实际经过时间，暂停回放后仍可过期。
配置、代码流和边界见 [动态 TF 与回放时间](replay_time.md)。

## VS Code 头文件解析

在 VS Code 中打开整个 `ros_gateserver` 工作区，并先执行上面的 colcon 构建。
CMake 会生成 `build/ros_gateserver/compile_commands.json`。
`.vscode/c_cpp_properties.json` 中的 `ROS 2 Jazzy` 配置指定编译器、C++17 和编译数据库，
不依赖 CMake Tools 提供配置。匹配到编译单元时，扩展读取数据库中的宏定义和头文件路径；
未匹配到时，使用配置中逐项列出的项目 include 和当前依赖包的 ROS include 目录。
这些备用目录取自当前构建数据库；新增依赖后，应同步更新备用目录。
Jazzy 的头文件目录包含包名层级，例如 `rclcpp/rclcpp.hpp` 位于
`/opt/ros/jazzy/include/rclcpp/rclcpp/rclcpp.hpp`，只添加 `/opt/ros/jazzy/include` 并不足够。
`websocket_server.hpp` 由 `gateway.cpp` 包含，可使用对应编译单元的配置解析。

若编辑器在 `#pragma once` 处显示“检测到 #include 错误，请更新 includePath”，
先确认上述编译数据库存在。该诊断通常表示编辑器没有正确解析依赖头文件，
需要结合具体缺失的头文件排查，不能通过删除 `#pragma once` 解决。
配置更新后若仍保留旧诊断，在命令面板执行 `C/C++: Select a Configuration`，
选择 `ROS 2 Jazzy`；必要时执行 `C/C++: Reset IntelliSense Database` 和
`Developer: Reload Window`。通过 `C/C++: Log Diagnostics` 可检查当前编译器、
编译数据库和实际头文件搜索路径。若日志中没有本项目配置，确认打开的是最外层
`ros_gateserver` 目录，而不是 `src/ros_gateserver` 子目录，并确认窗口连接到 WSL 环境。

## 输出约定

ROS `/gateway/status` 与默认 WebSocket 文本输出使用相同的 JSON 结构；
protobuf 解码后恢复同一组字段：

- `schema_version`、`enabled`、`stamp_ns`：版本、转发开关、ROS 时间。
- `sequence`、`clock`：进程内发布序号、时间来源、初始化状态和回跳代号。
- sensors.image：宽高、编码、字节数，以及 data.quality；不传图像像素。
- `sensors.scan`：采样数、有效采样数、雷达原点的最近有效距离、原始点与转换点；不传完整扫描数组。
- `sensors.odom`：原始坐标系下的位置、四元数、前向速度、角速度。
- 每路传感器附带 `received`、`fresh`、`age_sec`，以及 `data` 内的原始时间戳和 `frame_id`。
- 每路 diagnostics 保留端点兼容性、QoS 事件、状态和应用接收频率估计。

尚未收到数据时 `data`、`age_sec` 为 `null`。超过 2 秒无新消息时 `fresh` 为 `false`，
保留最近数据。没有有效雷达距离时 `nearest_m` 为 `null`。
雷达 `data.nearest_point_sensor` 带原始坐标系，`data.nearest_point` 带目标坐标系。
`transform_status` 为 `ok`、`unavailable`、`no_valid_point` 或 `invalid_scan_geometry`。
开启等待队列时，还可能出现 `timeout` 或 `queue_overflow`。
坐标转换只处理每帧中离雷达最近的一个点，不转换整帧扫描，也不重新计算离车体最近的点。
关闭转发后仍发送状态心跳，`data` 为 `null`；传感器订阅和计数继续运行，重新开启后恢复最近数据。
此 Service 只控制网关数据转发，不控制机器人运动。
