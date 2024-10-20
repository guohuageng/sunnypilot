# flake8: noqa

import unittest
import numpy as np
from cereal import car, custom
from common.realtime import DT_DMON
from selfdrive.controls.lib.events import Events
from selfdrive.monitoring.hands_on_wheel_monitor import HandsOnWheelStatus, _PRE_ALERT_THRESHOLD, \
                                  _PROMPT_ALERT_THRESHOLD, _TERMINAL_ALERT_THRESHOLD, \
                                  _MIN_MONITORING_SPEED

EventName = car.CarEvent.EventName
HandsOnWheelState = custom.DriverMonitoringStateSP.HandsOnWheelState

_TEST_TIMESPAN = 120  # 测试时间跨度，单位为秒

# 一些常见的状态向量
test_samples = int(_TEST_TIMESPAN / DT_DMON)  # 计算测试样本数量
half_test_samples = int(test_samples / 2.)  # 计算一半的测试样本数量
always_speed_over_threshold = [_MIN_MONITORING_SPEED + 1.] * test_samples  # 始终高于监控速度的速度向量
always_speed_under_threshold = [_MIN_MONITORING_SPEED - 1.] * test_samples  # 始终低于监控速度的速度向量
always_true = [True] * test_samples  # 始终为真的状态向量
always_false = [False] * test_samples  # 始终为假的状态向量
true_then_false = [True] * half_test_samples + [False] * (test_samples - half_test_samples)  # 前半部分为真，后半部分为假


def run_HOWState_seq(steering_wheel_interaction, openpilot_status, speed_status):
  # 输入数据均为10Hz
  HOWS = HandsOnWheelStatus()  # 创建方向盘状态对象
  events_from_HOWM = []  # 存储事件
  hands_on_wheel_state_from_HOWM = []  # 存储方向盘状态

  for idx in range(len(steering_wheel_interaction)):
    e = Events()  # 创建事件对象
    # 在10Hz下评估事件
    HOWS.update(e, steering_wheel_interaction[idx], openpilot_status[idx], speed_status[idx])  # 更新方向盘状态
    events_from_HOWM.append(e)  # 添加事件到列表
    hands_on_wheel_state_from_HOWM.append(HOWS.hands_on_wheel_state)  # 添加当前方向盘状态到列表

  assert len(events_from_HOWM) == len(steering_wheel_interaction), '发生错误'
  assert len(hands_on_wheel_state_from_HOWM) == len(steering_wheel_interaction), '发生错误'
  return events_from_HOWM, hands_on_wheel_state_from_HOWM  # 返回事件和状态


class TestHandsMonitoring(unittest.TestCase):
  # 0. 操作参与且超过监控速度，驾驶员始终将手放在方向盘上
  def test_hands_on_all_the_time(self):
    events_output, state_output = run_HOWState_seq(always_true, always_true, always_speed_over_threshold)
    self.assertTrue(np.sum([len(event) for event in events_output]) == 0)  # 确保没有事件
    self.assertEqual(state_output, [HandsOnWheelState.ok for x in range(len(state_output))])  # 确保状态为正常

  # 1. 操作参与且低于监控速度，方向盘交互无关
  def test_monitoring_under_threshold_speed(self):
    events_output, state_output = run_HOWState_seq(true_then_false, always_true, always_speed_under_threshold)
    self.assertTrue(np.sum([len(event) for event in events_output]) == 0)  # 确保没有事件
    self.assertEqual(state_output, [HandsOnWheelState.none for x in range(len(state_output))])  # 确保状态为无

  # 2. 操作参与且超过监控速度，驾驶员始终不将手放在方向盘上
  def test_hands_off_all_the_time(self):
    events_output, state_output = run_HOWState_seq(always_false, always_true, always_speed_over_threshold)
    # 在_PRE_ALERT_THRESHOLD之前的正确性检查
    self.assertTrue(np.sum([len(event) for event in events_output[:_PRE_ALERT_THRESHOLD - 1]]) == 0)
    self.assertEqual(state_output[:_PRE_ALERT_THRESHOLD - 1],
                     [HandsOnWheelState.minor for x in range(_PRE_ALERT_THRESHOLD - 1)])  # 确保状态为轻微
    # 在_PROMPT_ALERT_THRESHOLD之前的正确性检查
    self.assertEqual([event.names[0] for event in events_output[_PRE_ALERT_THRESHOLD:_PROMPT_ALERT_THRESHOLD - 1]],
                     [EventName.preKeepHandsOnWheel for x in range(_PROMPT_ALERT_THRESHOLD - 1 - _PRE_ALERT_THRESHOLD)])  # 确保事件为预警
    self.assertEqual(state_output[_PRE_ALERT_THRESHOLD:_PROMPT_ALERT_THRESHOLD - 1],
                     [HandsOnWheelState.warning for x in range(_PROMPT_ALERT_THRESHOLD - 1 - _PRE_ALERT_THRESHOLD)])  # 确保状态为警告
    # 在_TERMINAL_ALERT_THRESHOLD之前的正确性检查
    self.assertEqual(
        [event.names[0] for event in events_output[_PROMPT_ALERT_THRESHOLD:_TERMINAL_ALERT_THRESHOLD - 1]],
        [EventName.promptKeepHandsOnWheel for x in range(_TERMINAL_ALERT_THRESHOLD - 1 - _PROMPT_ALERT_THRESHOLD)])  # 确保事件为提示
    self.assertEqual(
        state_output[_PROMPT_ALERT_THRESHOLD:_TERMINAL_ALERT_THRESHOLD - 1],
        [HandsOnWheelState.critical for x in range(_TERMINAL_ALERT_THRESHOLD - 1 - _PROMPT_ALERT_THRESHOLD)])  # 确保状态为关键
    # 在_TERMINAL_ALERT_THRESHOLD之后的正确性检查
    self.assertEqual([event.names[0] for event in events_output[_TERMINAL_ALERT_THRESHOLD:]],
                     [EventName.keepHandsOnWheel for x in range(test_samples - _TERMINAL_ALERT_THRESHOLD)])  # 确保事件为保持手在方向盘上
    self.assertEqual(state_output[_TERMINAL_ALERT_THRESHOLD:],
                     [HandsOnWheelState.terminal for x in range(test_samples - _TERMINAL_ALERT_THRESHOLD)])  # 确保状态为终极

  # 3. 操作参与且超过监控速度，当速度低于监控速度时，警报状态重置为无
  def test_status_none_when_speeds_goes_down(self):
    speed_vector = always_speed_over_threshold[:-1] + [_MIN_MONITORING_SPEED - 1.]  # 速度向量
    events_output, state_output = run_HOWState_seq(always_false, always_true, speed_vector)
    # 在_TERMINAL_ALERT_THRESHOLD之后的正确性检查
    self.assertEqual([event.names[0] for event in events_output[_TERMINAL_ALERT_THRESHOLD:test_samples - 1]],
                     [EventName.keepHandsOnWheel for x in range(test_samples - 1 - _TERMINAL_ALERT_THRESHOLD)])  # 确保事件为保持手在方向盘上
    self.assertEqual(state_output[_TERMINAL_ALERT_THRESHOLD:test_samples - 1],
                     [HandsOnWheelState.terminal for x in range(test_samples - 1 - _TERMINAL_ALERT_THRESHOLD)])  # 确保状态为终极
    # 在最后一个样本中，速度低于监控阈值的正确性检查
    self.assertEqual(len(events_output[-1]), 0)  # 确保没有事件
    self.assertEqual(state_output[-1], HandsOnWheelState.none)  # 确保状态为无

  # 4. 操作参与且超过监控速度，当用户与方向盘交互时，警报状态重置为正常，
  # 一旦手离开方向盘，过程重复。
  def test_status_ok_after_interaction_with_wheel(self):
    interaction_vector = always_false[:_TERMINAL_ALERT_THRESHOLD] + [True
                                                                     ] + always_false[_TERMINAL_ALERT_THRESHOLD + 1:]  # 交互向量
    events_output, state_output = run_HOWState_seq(interaction_vector, always_true, always_speed_over_threshold)
    # 在_TERMINAL_ALERT_THRESHOLD之后的正确性检查
    self.assertEqual(events_output[_TERMINAL_ALERT_THRESHOLD - 1].names[0], EventName.keepHandsOnWheel)  # 确保事件为保持手在方向盘上
    self.assertEqual(state_output[_TERMINAL_ALERT_THRESHOLD - 1], HandsOnWheelState.terminal)  # 确保状态为终极
    # 当用户与方向盘交互时的正确性检查
    self.assertEqual(len(events_output[_TERMINAL_ALERT_THRESHOLD]), 0)  # 确保没有事件
    self.assertEqual(state_output[_TERMINAL_ALERT_THRESHOLD], HandsOnWheelState.ok)  # 确保状态为正常
    # 第二次运行的过程正确性检查
    offset = _TERMINAL_ALERT_THRESHOLD + 1
    self.assertTrue(np.sum([len(event) for event in events_output[offset:offset + _PRE_ALERT_THRESHOLD - 1]]) == 0)  # 确保没有事件
    self.assertEqual(state_output[offset:offset + _PRE_ALERT_THRESHOLD - 1],
                     [HandsOnWheelState.minor for x in range(_PRE_ALERT_THRESHOLD - 1)])  # 确保状态为轻微
    self.assertEqual(
        [event.names[0] for event in events_output[offset + _PRE_ALERT_THRESHOLD:offset + _PROMPT_ALERT_THRESHOLD - 1]],
        [EventName.preKeepHandsOnWheel for x in range(_PROMPT_ALERT_THRESHOLD - 1 - _PRE_ALERT_THRESHOLD)])  # 确保事件为预警
    self.assertEqual(state_output[offset + _PRE_ALERT_THRESHOLD:offset + _PROMPT_ALERT_THRESHOLD - 1],
                     [HandsOnWheelState.warning for x in range(_PROMPT_ALERT_THRESHOLD - 1 - _PRE_ALERT_THRESHOLD)])  # 确保状态为警告
    self.assertEqual([
        event.names[0]
        for event in events_output[offset + _PROMPT_ALERT_THRESHOLD:offset + _TERMINAL_ALERT_THRESHOLD - 1]
    ], [EventName.promptKeepHandsOnWheel for x in range(_TERMINAL_ALERT_THRESHOLD - 1 - _PROMPT_ALERT_THRESHOLD)])  # 确保事件为提示
    self.assertEqual(
        state_output[offset + _PROMPT_ALERT_THRESHOLD:offset + _TERMINAL_ALERT_THRESHOLD - 1],
        [HandsOnWheelState.critical for x in range(_TERMINAL_ALERT_THRESHOLD - 1 - _PROMPT_ALERT_THRESHOLD)])  # 确保状态为关键
    self.assertEqual([event.names[0] for event in events_output[offset + _TERMINAL_ALERT_THRESHOLD:]],
                     [EventName.keepHandsOnWheel for x in range(test_samples - offset - _TERMINAL_ALERT_THRESHOLD)])  # 确保事件为保持手在方向盘上
    self.assertEqual(state_output[offset + _TERMINAL_ALERT_THRESHOLD:],
                     [HandsOnWheelState.terminal for x in range(test_samples - offset - _TERMINAL_ALERT_THRESHOLD)])  # 确保状态为终极

  # 5. 操作未参与，始终不将手放在方向盘上
  #  - 当未参与时，监控应保持安静
  def test_pure_dashcam_user(self):
    events_output, state_output = run_HOWState_seq(always_false, always_false, always_speed_over_threshold)
    self.assertTrue(np.sum([len(event) for event in events_output]) == 0)  # 确保没有事件
    self.assertEqual(state_output, [HandsOnWheelState.none for x in range(len(state_output))])  # 确保状态为无


if __name__ == "__main__":
  unittest.main()  # 运行单元测试
