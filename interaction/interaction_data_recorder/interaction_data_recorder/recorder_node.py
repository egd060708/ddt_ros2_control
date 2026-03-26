#!/usr/bin/env python3
# Copyright 2025
# SPDX-License-Identifier: Apache-2.0

"""ROS 2 node: play a YAML command sequence and record JointState, Imu, commands to CSV."""

from __future__ import annotations

import csv
import math
import os
import threading
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import String


def default_workspace_data_directory() -> str:
    """解析工作空间根目录（含 src/ 的目录）下的 data/，兼容 install 与 merge-install 布局。"""
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory('interaction_data_recorder')).resolve()
        for up in share.parents:
            if (up / 'src').is_dir():
                return str((up / 'data').resolve())
    except Exception:
        pass
    return str((Path.cwd() / 'data').resolve())


@dataclass
class LatestJointState:
    names: List[str] = field(default_factory=list)
    position: Dict[str, float] = field(default_factory=dict)
    velocity: Dict[str, float] = field(default_factory=dict)
    effort: Dict[str, float] = field(default_factory=dict)
    stamp_sec: float = 0.0


class RecorderNode(Node):
    def __init__(self) -> None:
        super().__init__('interaction_data_recorder')

        self.declare_parameter('sequence_file', '')
        self.declare_parameter('sample_rate_hz', 100.0)
        self.declare_parameter('command_publish_rate_hz', 50.0)
        # 空字符串：使用 <工作空间>/data（见 default_workspace_data_directory）
        self.declare_parameter('output_directory', '')
        self.declare_parameter('output_csv_filename', '')
        self.declare_parameter('joint_states_topic', 'joint_states')
        self.declare_parameter('imu_topic', 'imu_sensor_broadcaster/imu')
        self.declare_parameter('cmd_twist_topic', 'command/cmd_twist')
        self.declare_parameter('cmd_pose_topic', 'command/cmd_pose')
        self.declare_parameter('cmd_key_topic', 'command/cmd_key')
        self.declare_parameter('subscribe_cmd_topics', True)
        self.declare_parameter('start_delay_sec', 2.0)
        self.declare_parameter('clear_commands_on_wait', True)
        self.declare_parameter('wait_for_joint_state_sec', 30.0)

        seq_file = self.get_parameter('sequence_file').get_parameter_value().string_value
        if not seq_file:
            self.get_logger().fatal('Parameter sequence_file must be set to a YAML path')
            raise RuntimeError('sequence_file empty')

        self._sample_hz = max(1e-6, self.get_parameter('sample_rate_hz').value)
        self._cmd_pub_hz = max(1e-6, self.get_parameter('command_publish_rate_hz').value)
        out_dir = self.get_parameter('output_directory').get_parameter_value().string_value.strip()
        self._output_dir = (
            os.path.expanduser(out_dir) if out_dir else default_workspace_data_directory()
        )
        self.get_logger().info(f'CSV output directory: {self._output_dir}')
        self._output_name = self.get_parameter('output_csv_filename').get_parameter_value().string_value
        self._clear_on_wait = self.get_parameter('clear_commands_on_wait').value
        self._subscribe_cmds = self.get_parameter('subscribe_cmd_topics').value
        self._start_delay = max(0.0, float(self.get_parameter('start_delay_sec').value))
        self._wait_js = max(0.0, float(self.get_parameter('wait_for_joint_state_sec').value))

        with open(os.path.expanduser(seq_file), 'r', encoding='utf-8') as f:
            raw = yaml.safe_load(f)
        self._sequence: List[Dict[str, Any]] = list(raw.get('sequence', []))
        if not self._sequence:
            raise RuntimeError(f'No steps in sequence file: {seq_file}')
        self._manual_csv_window = any(
            str(s.get('type', '')).lower() in ('record_start', 'record_stop')
            for s in self._sequence
        )

        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        # 与 teleop_command 一致：RELIABLE + TRANSIENT_LOCAL，否则与遥控器发布端 durability 不匹配，收不到指令也无法正确发布
        qos_cmd = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        jt = self.get_parameter('joint_states_topic').value
        imu_t = self.get_parameter('imu_topic').value
        ct = self.get_parameter('cmd_twist_topic').value
        cp = self.get_parameter('cmd_pose_topic').value
        ck = self.get_parameter('cmd_key_topic').value

        self._lock = threading.Lock()
        self._latest_js = LatestJointState()
        self._latest_imu: Optional[Imu] = None
        self._latest_twist: Optional[Twist] = None
        self._latest_pose: Optional[PoseStamped] = None
        self._latest_key: str = ''

        self.create_subscription(JointState, jt, self._cb_joint_state, qos_sensor)
        self.create_subscription(Imu, imu_t, self._cb_imu, qos_sensor)
        if self._subscribe_cmds:
            self.create_subscription(Twist, ct, self._cb_twist, qos_cmd)
            self.create_subscription(PoseStamped, cp, self._cb_pose, qos_cmd)
            self.create_subscription(String, ck, self._cb_key, qos_cmd)

        self._pub_twist = self.create_publisher(Twist, ct, qos_cmd)
        self._pub_pose = self.create_publisher(PoseStamped, cp, qos_cmd)
        self._pub_key = self.create_publisher(String, ck, qos_cmd)

        self._recording = False
        self._finished = False
        self._step_idx = 0
        self._step_start: Optional[rclpy.time.Time] = None
        self._csv_path: Optional[str] = None
        self._csv_file = None
        self._csv_writer: Optional[csv.DictWriter] = None
        self._fieldnames: Optional[List[str]] = None
        self._csv_logging = False
        self._csv_stem_full: str = ''
        self._csv_ext: str = '.csv'
        self._csv_segment_idx = 0
        self._timer_arm: Optional[Any] = None
        self._timer_wait_js: Optional[Any] = None

        self._step_twist = Twist()
        self._step_pose = PoseStamped()
        self._step_key = String()
        self._step_publish_mode = 'wait'  # twist | key | pose | wait

        period_sample = 1.0 / self._sample_hz
        period_cmd = 1.0 / self._cmd_pub_hz
        self._timer_sample = self.create_timer(period_sample, self._on_sample)
        self._timer_cmd = self.create_timer(period_cmd, self._on_cmd_publish)
        self._timer_step = self.create_timer(0.05, self._on_step_tick)

        if self._start_delay > 0.0:
            self._timer_arm = self.create_timer(self._start_delay, self._arm_recording)
        else:
            self._arm_recording()

        self.get_logger().info(
            f'Loaded {len(self._sequence)} sequence steps from {seq_file}; '
            f'sample_rate={self._sample_hz} Hz (node clock / sim time)'
        )
        if self._manual_csv_window:
            self.get_logger().info(
                'Manual CSV window enabled: use type record_start / record_stop in sequence '
                '(only rows between them are written; sequence running time still ends at last step).'
            )

    def _cb_joint_state(self, msg: JointState) -> None:
        t = self.get_clock().now()
        sec = t.nanoseconds / 1e9
        with self._lock:
            self._latest_js.names = list(msg.name)
            self._latest_js.position = {n: float(p) for n, p in zip(msg.name, msg.position)}
            self._latest_js.velocity = {
                n: float(v) for n, v in zip(msg.name, msg.velocity)
            } if msg.velocity else {}
            self._latest_js.effort = {n: float(e) for n, e in zip(msg.name, msg.effort)} if msg.effort else {}
            self._latest_js.stamp_sec = sec

    def _cb_imu(self, msg: Imu) -> None:
        with self._lock:
            self._latest_imu = msg

    def _cb_twist(self, msg: Twist) -> None:
        with self._lock:
            self._latest_twist = msg

    def _cb_pose(self, msg: PoseStamped) -> None:
        with self._lock:
            self._latest_pose = msg

    def _cb_key(self, msg: String) -> None:
        with self._lock:
            self._latest_key = msg.data

    def _arm_recording(self) -> None:
        if self._timer_arm is not None:
            self._timer_arm.cancel()
            self._timer_arm = None
        self.get_logger().info('Waiting for joint_states to fix column layout...')
        self._wait_js_end = self.get_clock().now() + Duration(seconds=self._wait_js)
        self._timer_wait_js = self.create_timer(0.1, self._try_begin_recording)

    def _try_begin_recording(self) -> None:
        with self._lock:
            has_names = len(self._latest_js.names) > 0
        if has_names:
            if self._timer_wait_js is not None:
                self._timer_wait_js.cancel()
                self._timer_wait_js = None
            self._begin_recording()
            return
        if self.get_clock().now() > self._wait_js_end:
            self.get_logger().error('Timeout waiting for joint_states; recording aborted')
            if self._timer_wait_js is not None:
                self._timer_wait_js.cancel()
                self._timer_wait_js = None
            self._finished = True
            self._recording = False

    def _ensure_csv_parent_directory(self, path: str) -> bool:
        """确保 CSV 路径的父目录存在（不存在则创建）。失败时打日志并返回 False。"""
        parent = os.path.dirname(os.path.abspath(os.path.expanduser(path)))
        if not parent:
            return True
        try:
            os.makedirs(parent, exist_ok=True)
        except OSError as e:
            self.get_logger().error(
                f'无法创建或访问 CSV 所在目录 [{parent}]: {e}。 '
                f'请改用当前用户可写的路径（例如仅填文件名以写入 CSV output_directory，'
                f'或 ~/data/run.csv），勿使用系统根目录下无权限的路径（如 /test/）。'
            )
            return False
        return True

    def _resolve_csv_stem_and_ext(self) -> None:
        """根据 output_csv_filename 与 output_directory 解析首段文件的 stem 与扩展名。"""
        out_dir = os.path.expanduser(self._output_dir)
        if self._output_name:
            name = self._output_name.strip()
            if not name.lower().endswith('.csv'):
                name += '.csv'
            name_exp = os.path.expanduser(name)
            if os.path.isabs(name_exp):
                full_first = name_exp
            else:
                os.makedirs(out_dir, exist_ok=True)
                full_first = os.path.normpath(os.path.join(out_dir, name_exp))
        else:
            os.makedirs(out_dir, exist_ok=True)
            t = self.get_clock().now().nanoseconds
            full_first = os.path.join(out_dir, f'record_{t}.csv')
        self._csv_stem_full = os.path.splitext(full_first)[0]
        self._csv_ext = os.path.splitext(full_first)[1] or '.csv'

    def _begin_recording(self) -> None:
        os.makedirs(os.path.expanduser(self._output_dir), exist_ok=True)
        self._resolve_csv_stem_and_ext()
        self._csv_segment_idx = 0

        with self._lock:
            joint_order = list(self._latest_js.names)

        if self._manual_csv_window:
            self._fieldnames = None
            self._csv_logging = False
            self._csv_path = None
        else:
            self._fieldnames = self._build_fieldnames(joint_order)
            self._open_csv_writer(self._csv_path_for_segment(0))

        self._recording = True
        self._step_idx = 0
        self._step_start = self.get_clock().now()
        self._apply_step(self._sequence[0])
        if self._manual_csv_window:
            self.get_logger().info(
                'Sequence started (manual CSV: write begins at record_start, stops at record_stop).'
            )
        else:
            self.get_logger().info(f'Sequence started; logging to {self._csv_path}')

    def _csv_path_for_segment(self, idx: int) -> str:
        if idx == 0:
            return f'{self._csv_stem_full}{self._csv_ext}'
        return f'{self._csv_stem_full}_part{idx}{self._csv_ext}'

    def _open_csv_writer(self, path: str) -> None:
        self._close_csv_writer(increment_segment=False)
        self._fieldnames = None
        with self._lock:
            joint_order = list(self._latest_js.names)
        if not joint_order:
            self.get_logger().error('Cannot open CSV: joint_states names empty')
            return
        path_resolved = os.path.normpath(os.path.expanduser(path))
        if not self._ensure_csv_parent_directory(path_resolved):
            return
        try:
            csv_file = open(path_resolved, 'w', newline='', encoding='utf-8')
        except OSError as e:
            self.get_logger().error(f'无法打开 CSV 文件 [{path_resolved}]: {e}')
            return
        self._fieldnames = self._build_fieldnames(joint_order)
        self._csv_path = path_resolved
        self._csv_file = csv_file
        self._csv_writer = csv.DictWriter(
            self._csv_file, fieldnames=self._fieldnames, extrasaction='ignore'
        )
        self._csv_writer.writeheader()
        self._csv_file.flush()
        self._csv_logging = True
        self.get_logger().info(f'CSV logging started: {path_resolved}')

    def _close_csv_writer(self, increment_segment: bool = True) -> None:
        had_file = self._csv_file is not None
        self._csv_logging = False
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None
        self._csv_writer = None
        if increment_segment and self._manual_csv_window and had_file:
            self._csv_segment_idx += 1

    def _on_record_start_step(self) -> None:
        if not self._manual_csv_window:
            return
        path = self._csv_path_for_segment(self._csv_segment_idx)
        self._open_csv_writer(path)

    def _on_record_stop_step(self) -> None:
        if not self._manual_csv_window:
            return
        self.get_logger().info(
            f'CSV logging stopped. File: {self._csv_path}' if self._csv_path else 'CSV logging stopped.'
        )
        self._close_csv_writer(increment_segment=True)

    def _build_fieldnames(self, joint_order: List[str]) -> List[str]:
        cols = ['sim_time_s', 'step_index', 'step_type']
        for j in joint_order:
            cols += [f'joint_{j}_pos', f'joint_{j}_vel', f'joint_{j}_effort']
        cols += [
            'imu_orient_x', 'imu_orient_y', 'imu_orient_z', 'imu_orient_w',
            'imu_gyro_x', 'imu_gyro_y', 'imu_gyro_z',
            'imu_accel_x', 'imu_accel_y', 'imu_accel_z',
        ]
        cols += [
            'cmd_twist_linear_x', 'cmd_twist_linear_y', 'cmd_twist_linear_z',
            'cmd_twist_angular_x', 'cmd_twist_angular_y', 'cmd_twist_angular_z',
            'cmd_pose_px', 'cmd_pose_py', 'cmd_pose_pz',
            'cmd_pose_qx', 'cmd_pose_qy', 'cmd_pose_qz', 'cmd_pose_qw',
            'cmd_key',
        ]
        return cols

    def _csv_row_base(self) -> Dict[str, Any]:
        """非指令列默认空串；指令列默认 0（无指令时导出为 0）。"""
        out: Dict[str, Any] = {}
        assert self._fieldnames is not None
        for k in self._fieldnames:
            if k.startswith('cmd_twist_') or k.startswith('cmd_pose_'):
                out[k] = 0.0
            elif k == 'cmd_key':
                out[k] = 0
            else:
                out[k] = ''
        return out

    def _normalize_command_csv_cells(self, full: Dict[str, Any]) -> None:
        """将仍为空的指令单元格统一为 0。"""
        assert self._fieldnames is not None
        for k in self._fieldnames:
            if k.startswith('cmd_twist_') or k.startswith('cmd_pose_'):
                v = full.get(k, '')
                if v == '' or v is None:
                    full[k] = 0.0
            elif k == 'cmd_key':
                v = full.get(k, '')
                if v == '' or v is None:
                    full[k] = 0

    def _apply_step(self, step: Dict[str, Any]) -> None:
        stype = str(step.get('type', 'wait')).lower()
        self._step_publish_mode = stype
        if stype == 'record_start':
            self._step_publish_mode = 'wait'
            self._on_record_start_step()
            return
        if stype == 'record_stop':
            self._step_publish_mode = 'wait'
            self._on_record_stop_step()
            return
        if stype == 'twist':
            lin = step.get('linear', {}) or {}
            ang = step.get('angular', {}) or {}
            self._step_twist.linear.x = float(lin.get('x', 0.0))
            self._step_twist.linear.y = float(lin.get('y', 0.0))
            self._step_twist.linear.z = float(lin.get('z', 0.0))
            self._step_twist.angular.x = float(ang.get('x', 0.0))
            self._step_twist.angular.y = float(ang.get('y', 0.0))
            self._step_twist.angular.z = float(ang.get('z', 0.0))
        elif stype == 'key':
            self._step_key.data = str(step.get('data', ''))
        elif stype == 'pose':
            pos = step.get('position', {}) or {}
            self._step_pose.header.frame_id = str(step.get('frame_id', 'map'))
            self._step_pose.pose.position.x = float(pos.get('x', 0.0))
            self._step_pose.pose.position.y = float(pos.get('y', 0.0))
            self._step_pose.pose.position.z = float(pos.get('z', 0.0))
            rpy = step.get('orientation_rpy', {}) or {}
            if rpy:
                roll = float(rpy.get('roll', 0.0))
                pitch = float(rpy.get('pitch', 0.0))
                yaw = float(rpy.get('yaw', 0.0))
                cy = math.cos(yaw * 0.5)
                sy = math.sin(yaw * 0.5)
                cp = math.cos(pitch * 0.5)
                sp = math.sin(pitch * 0.5)
                cr = math.cos(roll * 0.5)
                sr = math.sin(roll * 0.5)
                qw = cr * cp * cy + sr * sp * sy
                qx = sr * cp * cy - cr * sp * sy
                qy = cr * sp * cy + sr * cp * sy
                qz = cr * cp * sy - sr * sp * cy
                self._step_pose.pose.orientation.x = qx
                self._step_pose.pose.orientation.y = qy
                self._step_pose.pose.orientation.z = qz
                self._step_pose.pose.orientation.w = qw
            else:
                q = step.get('orientation_quat', {}) or {}
                self._step_pose.pose.orientation.x = float(q.get('x', 0.0))
                self._step_pose.pose.orientation.y = float(q.get('y', 0.0))
                self._step_pose.pose.orientation.z = float(q.get('z', 0.0))
                self._step_pose.pose.orientation.w = float(q.get('w', 1.0))
        elif stype in ('wait', 'none', 'idle'):
            self._step_publish_mode = 'wait'
            if self._clear_on_wait:
                self._step_twist = Twist()
                self._step_key = String()
        else:
            self.get_logger().warn(f'Unknown step type "{stype}", treating as wait')
            self._step_publish_mode = 'wait'

    def _on_step_tick(self) -> None:
        if not self._recording or self._finished:
            return
        now = self.get_clock().now()
        step = self._sequence[self._step_idx]
        dur = float(step.get('duration_sec', 0.0))
        if self._step_start is None:
            self._step_start = now
        elapsed = (now - self._step_start).nanoseconds / 1e9
        if elapsed >= dur:
            self._step_idx += 1
            if self._step_idx >= len(self._sequence):
                self._finish_recording()
                return
            self._step_start = now
            self._apply_step(self._sequence[self._step_idx])

    def _on_cmd_publish(self) -> None:
        if not self._recording or self._finished:
            return
        now = self.get_clock().now()
        self._step_pose.header.stamp = now.to_msg()
        mode = self._step_publish_mode
        if mode == 'twist':
            self._pub_twist.publish(self._step_twist)
        elif mode == 'pose':
            self._pub_pose.publish(self._step_pose)
        elif mode == 'key':
            self._pub_key.publish(self._step_key)

    def _finish_recording(self) -> None:
        self._finished = True
        self._recording = False
        if self._csv_file is not None:
            self._close_csv_writer(increment_segment=False)
        self._csv_logging = False
        if self._csv_path:
            self.get_logger().info(f'Sequence finished. Last CSV: {self._csv_path}')
        else:
            self.get_logger().info('Sequence finished (no CSV was written).')

    def _on_sample(self) -> None:
        if not self._recording or self._finished or not self._csv_logging:
            return
        if self._csv_writer is None or self._fieldnames is None:
            return

        now = self.get_clock().now()
        sim_time_s = now.nanoseconds / 1e9
        row: Dict[str, Any] = {
            'sim_time_s': sim_time_s,
            'step_index': self._step_idx,
            'step_type': str(self._sequence[self._step_idx].get('type', '')),
        }

        tw: Optional[Twist] = None
        ps: Optional[PoseStamped] = None
        key = ''

        with self._lock:
            names = list(self._latest_js.names)
            for j in names:
                row[f'joint_{j}_pos'] = self._latest_js.position.get(j, '')
                row[f'joint_{j}_vel'] = self._latest_js.velocity.get(j, '')
                row[f'joint_{j}_effort'] = self._latest_js.effort.get(j, '')

            imu = self._latest_imu
            if imu is not None:
                row['imu_orient_x'] = imu.orientation.x
                row['imu_orient_y'] = imu.orientation.y
                row['imu_orient_z'] = imu.orientation.z
                row['imu_orient_w'] = imu.orientation.w
                row['imu_gyro_x'] = imu.angular_velocity.x
                row['imu_gyro_y'] = imu.angular_velocity.y
                row['imu_gyro_z'] = imu.angular_velocity.z
                row['imu_accel_x'] = imu.linear_acceleration.x
                row['imu_accel_y'] = imu.linear_acceleration.y
                row['imu_accel_z'] = imu.linear_acceleration.z

            tw = self._latest_twist
            ps = self._latest_pose
            key = self._latest_key

        if tw is None and self._step_publish_mode == 'twist':
            tw = self._step_twist
        if ps is None and self._step_publish_mode == 'pose':
            ps = self._step_pose
        if not key and self._step_publish_mode == 'key':
            key = self._step_key.data

        if tw is not None:
            row['cmd_twist_linear_x'] = tw.linear.x
            row['cmd_twist_linear_y'] = tw.linear.y
            row['cmd_twist_linear_z'] = tw.linear.z
            row['cmd_twist_angular_x'] = tw.angular.x
            row['cmd_twist_angular_y'] = tw.angular.y
            row['cmd_twist_angular_z'] = tw.angular.z
        if ps is not None:
            row['cmd_pose_px'] = ps.pose.position.x
            row['cmd_pose_py'] = ps.pose.position.y
            row['cmd_pose_pz'] = ps.pose.position.z
            row['cmd_pose_qx'] = ps.pose.orientation.x
            row['cmd_pose_qy'] = ps.pose.orientation.y
            row['cmd_pose_qz'] = ps.pose.orientation.z
            row['cmd_pose_qw'] = ps.pose.orientation.w
        row['cmd_key'] = key

        full = self._csv_row_base()
        full.update(row)
        self._normalize_command_csv_cells(full)
        self._csv_writer.writerow(full)
        if self._csv_file is not None:
            self._csv_file.flush()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[RecorderNode] = None
    try:
        node = RecorderNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        print(e)
    finally:
        if node is not None:
            if node._csv_file is not None:
                node._csv_file.close()
                node._csv_file = None
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
