# interaction_data_recorder

ROS 2 Humble（`rclpy`）功能包：在仿真或实机运行时，**按 YAML 播放指令序列**（与 `teleop_command` 相同的话题约定），并在可配置的采样率下将**关节状态、IMU、指令相关话题**导出为 **CSV**。定时器使用节点时钟，在 **`use_sim_time:=true`** 时与 **`/clock`** 对齐。

---

## 依赖与编译

- ROS 2 Humble
- `sensor_msgs`、`geometry_msgs`、`std_msgs`、`python3-yaml`、`ament_index_python`

```bash
cd ~/ddt_ros2_ws
colcon build --packages-select interaction_data_recorder --symlink-install
source install/setup.bash
```

---

## 快速启动

```bash
ros2 launch interaction_data_recorder recorder.launch.py
```

默认：

- 读取包内 `config/sequence_example.yaml` 作为指令序列；
- 读取 `config/recorder_params.yaml` 作为节点参数（`sample_rate_hz`、`output_csv_filename` 等以 yaml 为准，除非在命令行显式覆盖，见下文）；
- `use_sim_time` 默认为 `true`，**请与发布 `/clock` 的仿真/实机一起使用**；若仅用系统时间，可传 `use_sim_time:=false`；
- CSV 默认写到工作空间根目录下的 **`data/`**（launch 会解析含有 `src/` 的目录为工作空间）。

常用命令行参数：

```bash
ros2 launch interaction_data_recorder recorder.launch.py \
  sequence_file:=/path/to/my_sequence.yaml \
  output_directory:=/home/user/ddt_ros2_ws/data \
  use_sim_time:=true
```

仅在命令行**传入非空**时才会覆盖 yaml 中的对应项（避免空字符串盖掉 yaml）：

```bash
ros2 launch interaction_data_recorder recorder.launch.py \
  output_csv_filename:=run_01.csv \
  sample_rate_hz:=200.0
```

---

## 订阅与发布的话题

与仓库 `ros_utils/include/ros_utils/topic_names.hpp` 一致：

| 方向 | 话题 | 消息类型 | 说明 |
|------|------|----------|------|
| 订阅 | `joint_states` | `sensor_msgs/JointState` | 关节位置/速度/力矩 |
| 订阅 | `imu_sensor_broadcaster/imu` | `sensor_msgs/Imu` | IMU |
| 订阅 | `command/cmd_twist` | `geometry_msgs/Twist` | 速度指令（含遥控回读） |
| 订阅 | `command/cmd_pose` | `geometry_msgs/PoseStamped` | 位姿指令 |
| 订阅 | `command/cmd_key` | `std_msgs/String` | 字符串指令（如 FSM） |
| 发布 | 同上 cmd_* | 同上 | 按序列播放指令 |

指令话题的 QoS 为 **RELIABLE + TRANSIENT_LOCAL**，与 `teleop_command` 发布端兼容。

---

## 参数说明（`config/recorder_params.yaml`）

| 参数 | 说明 |
|------|------|
| `sequence_file` | 由 launch 传入，yaml 里通常不写 |
| `sample_rate_hz` | CSV 采样频率（Hz），与节点时钟一致 |
| `command_publish_rate_hz` | 序列中指令发布频率 |
| `output_directory` | **空字符串**时：解析为「含 `src/` 的工作空间根」下的 `data/` |
| `output_csv_filename` | 见下文「输出文件与路径」 |
| `joint_states_topic` / `imu_topic` / `cmd_*_topic` | 可按实机/仿真 remap |
| `subscribe_cmd_topics` | 是否订阅 cmd_*（用于把遥控指令写入 CSV） |
| `start_delay_sec` | 开始等待后再起序列（秒） |
| `clear_commands_on_wait` | `wait` 类步骤是否清空内部缓存的 twist/key |
| `wait_for_joint_state_sec` | 等待 `joint_states` 以确定 CSV 表头的超时 |

---

## 输出文件与路径

- **仅文件名或相对路径**（**不以 `/` 开头**）：与 `output_directory` 拼接，例如 `test/run.csv` → `{output_directory}/test/run.csv`，父目录不存在时会自动创建（无权限则打日志并跳过写文件，节点不崩溃）。
- **绝对路径**（以 `/` 开头）：**不再拼接** `output_directory`，必须对父目录有写权限。

**默认文件名**：若 `output_csv_filename` 为空，则使用 `record_{节点当前时刻纳秒}.csv`（仿真模式下为仿真时钟）。

**显式采集多段**：序列中使用多对 `record_start` / `record_stop` 时，第二段及以后文件名为 `{主名}_part1.csv`、`{主名}_part2.csv` …

---

## 指令序列 YAML（`config/sequence_example.yaml`）

- 根键：`sequence`（步骤列表）。
- 每步至少包含：`type`、`duration_sec`（仿真秒）。

步骤类型摘要：

| `type` | 作用 |
|--------|------|
| `twist` | 发布 `Twist`，字段 `linear` / `angular` 的 `x,y,z` |
| `key` | 发布 `String`，字段 `data` |
| `pose` | 发布 `PoseStamped`：`frame_id`、`position`、`orientation_rpy` 或 `orientation_quat` |
| `wait` / `none` / `idle` | 不发指令（可选清空内部指令） |
| `record_start` | **开始**向 CSV 写行（需在序列中出现才会进入「显式采集」模式） |
| `record_stop` | **停止**写行并关闭当前 CSV 文件 |

**显式采集模式**：只要序列里出现 `record_start` 或 `record_stop` 任一，则**仅**在 `record_start`～`record_stop` 之间写入采样；未出现这两项时，行为为**整条序列从头到尾都写入**（与旧版一致）。

更完整的字段说明见 `config/sequence_example.yaml` 顶部注释。

---

## CSV 列说明

- `sim_time_s`、`step_index`、`step_type`
- 每个关节：`joint_<name>_pos/vel/effort`（关节顺序由首次收到的 `joint_states` 固定）
- IMU：四元数、角速度、线加速度
- 指令：`cmd_twist_*`、`cmd_pose_*`、`cmd_key`

**缺省填充**：无关节/IMU数据时对应格可为空；**无指令数据时**，所有 `cmd_*` 列导出为 **0**（数值为 `0.0`，`cmd_key` 为 `0`），便于下游数值处理。

---

## 与仿真配合

1. 先启动仿真与 `ros2_control`/控制器（保证存在 `joint_states`、IMU、`/clock`）。
2. 再 `ros2 launch interaction_data_recorder recorder.launch.py`。
3. 若时间不前进，检查全局是否使用 `use_sim_time` 且 **`/clock` 是否在发布**。

---

## 常见问题

- **QoS 警告 / 收不到指令**：本包已与 `teleop_command` 的 durability 对齐；若仍异常，用 `ros2 topic info -v` 核对发布/订阅 QoS。
- **`PermissionError` 写盘**：勿使用无写权限的绝对路径（例如 Linux 根下 `/test/...`）；改用工作空间 `data/` 或 `~/...`。
- **`output_csv_filename` 在 yaml 里不生效**：请确认 launch **未**传入空的 `output_csv_filename:=` 覆盖；当前 launch 仅在**非空** CLI 时覆盖 yaml。

---

## 许可证

Apache-2.0（见 `package.xml`）。
