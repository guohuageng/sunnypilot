import numpy as np

from cereal import car, log
from openpilot.common.realtime import DT_DMON
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring, DRIVER_MONITOR_SETTINGS

EventName = car.CarEvent.EventName
dm_settings = DRIVER_MONITOR_SETTINGS()

TEST_TIMESPAN = 120  # seconds  # 测试时间跨度，单位为秒
DISTRACTED_SECONDS_TO_ORANGE = dm_settings._DISTRACTED_TIME - dm_settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL + 1  # 从分心到橙色警告的时间
DISTRACTED_SECONDS_TO_RED = dm_settings._DISTRACTED_TIME + 1  # 从分心到红色警告的时间
INVISIBLE_SECONDS_TO_ORANGE = dm_settings._AWARENESS_TIME - dm_settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL + 1  # 从不可见到橙色警告的时间
INVISIBLE_SECONDS_TO_RED = dm_settings._AWARENESS_TIME + 1  # 从不可见到红色警告的时间

def make_msg(face_detected, distracted=False, model_uncertain=False):
  ds = log.DriverStateV2.new_message()  # 创建新的驾驶员状态消息
  ds.leftDriverData.faceOrientation = [0., 0., 0.]  # 面部朝向
  ds.leftDriverData.facePosition = [0., 0.]  # 面部位置
  ds.leftDriverData.faceProb = 1. * face_detected  # 面部检测概率
  ds.leftDriverData.leftEyeProb = 1.  # 左眼检测概率
  ds.leftDriverData.rightEyeProb = 1.  # 右眼检测概率
  ds.leftDriverData.leftBlinkProb = 1. * distracted  # 左眼眨眼概率
  ds.leftDriverData.rightBlinkProb = 1. * distracted  # 右眼眨眼概率
  ds.leftDriverData.faceOrientationStd = [1.*model_uncertain, 1.*model_uncertain, 1.*model_uncertain]  # 面部朝向标准差
  ds.leftDriverData.facePositionStd = [1.*model_uncertain, 1.*model_uncertain]  # 面部位置标准差
  # TODO: test both separately when e2e is used  # TODO: 在使用端到端时分别测试这两者
  ds.leftDriverData.readyProb = [0., 0., 0., 0.]  # 准备概率
  ds.leftDriverData.notReadyProb = [0., 0.]  # 不准备概率
  return ds


# driver state from neural net, 10Hz  # 从神经网络获取的驾驶员状态，10Hz
msg_NO_FACE_DETECTED = make_msg(False)  # 未检测到面部
msg_ATTENTIVE = make_msg(True)  # 驾驶员专注
msg_DISTRACTED = make_msg(True, distracted=True)  # 驾驶员分心
msg_ATTENTIVE_UNCERTAIN = make_msg(True, model_uncertain=True)  # 驾驶员专注但不确定
msg_DISTRACTED_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=True)  # 驾驶员分心且不确定
msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN = make_msg(True, distracted=True, model_uncertain=dm_settings._POSESTD_THRESHOLD*1.5)  # 驾驶员分心但某种程度上不确定

# driver interaction with car  # 驾驶员与汽车的交互
car_interaction_DETECTED = True  # 检测到驾驶员交互
car_interaction_NOT_DETECTED = False  # 未检测到驾驶员交互

# some common state vectors  # 一些常见的状态向量
always_no_face = [msg_NO_FACE_DETECTED] * int(TEST_TIMESPAN / DT_DMON)  # 始终未检测到面部
always_attentive = [msg_ATTENTIVE] * int(TEST_TIMESPAN / DT_DMON)  # 始终专注
always_distracted = [msg_DISTRACTED] * int(TEST_TIMESPAN / DT_DMON)  # 始终分心
always_true = [True] * int(TEST_TIMESPAN / DT_DMON)  # 始终为真
always_false = [False] * int(TEST_TIMESPAN / DT_DMON)  # 始终为假

class TestMonitoring:
  def _run_seq(self, msgs, interaction, engaged, standstill):
    DM = DriverMonitoring()  # 创建驾驶员监控实例
    events = []  # 事件列表
    for idx in range(len(msgs)):
      DM._update_states(msgs[idx], [0, 0, 0], 0, engaged[idx])  # 更新状态
      # cal_rpy and car_speed don't matter here  # 这里不需要考虑姿态和车速

      # evaluate events at 10Hz for tests  # 以10Hz评估事件以进行测试
      DM._update_events(interaction[idx], engaged[idx], standstill[idx], 0, 0)  # 更新事件
      events.append(DM.current_events)  # 添加当前事件到事件列表
    assert len(events) == len(msgs), f"got {len(events)} for {len(msgs)} driverState input msgs"  # 确保事件数量与消息数量一致
    return events, DM

  def _assert_no_events(self, events):
    assert all(not len(e) for e in events)  # 确保没有事件

  # engaged, driver is attentive all the time  # 驾驶员始终专注
  def test_fully_aware_driver(self):
    events, _ = self._run_seq(always_attentive, always_false, always_true, always_false)  # 运行测试序列
    self._assert_no_events(events)  # 断言没有事件

  # engaged, driver is distracted and does nothing  # 驾驶员分心且无所作为
  def test_fully_distracted_driver(self):
    events, d_status = self._run_seq(always_distracted, always_false, always_true, always_false)  # 运行测试序列
    assert len(events[int((d_status.settings._DISTRACTED_TIME-d_status.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL)/2/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((d_status.settings._DISTRACTED_TIME-d_status.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL + \
                    ((d_status.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL-d_status.settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL)/2))/DT_DMON)].names[0] == \
                    EventName.preDriverDistracted  # 断言事件名称为预警分心
    assert events[int((d_status.settings._DISTRACTED_TIME-d_status.settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL + \
                    ((d_status.settings._DISTRACTED_PROMPT_TIME_TILL_TERMINAL)/2))/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心
    assert events[int((d_status.settings._DISTRACTED_TIME + \
                    ((TEST_TIMESPAN-10-d_status.settings._DISTRACTED_TIME)/2))/DT_DMON)].names[0] == EventName.driverDistracted  # 断言事件名称为分心
    assert isinstance(d_status.awareness, float)  # 断言意识状态为浮点数

  # engaged, no face detected the whole time, no action  # 整个时间未检测到面部，无任何动作
  def test_fully_invisible_driver(self):
    events, d_status = self._run_seq(always_no_face, always_false, always_true, always_false)  # 运行测试序列
    assert len(events[int((d_status.settings._AWARENESS_TIME-d_status.settings._AWARENESS_PRE_TIME_TILL_TERMINAL)/2/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((d_status.settings._AWARENESS_TIME-d_status.settings._AWARENESS_PRE_TIME_TILL_TERMINAL + \
                      ((d_status.settings._AWARENESS_PRE_TIME_TILL_TERMINAL-d_status.settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL)/2))/DT_DMON)].names[0] == \
                      EventName.preDriverUnresponsive  # 断言事件名称为预警无反应
    assert events[int((d_status.settings._AWARENESS_TIME-d_status.settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL + \
                      ((d_status.settings._AWARENESS_PROMPT_TIME_TILL_TERMINAL)/2))/DT_DMON)].names[0] == EventName.promptDriverUnresponsive  # 断言事件名称为提示无反应
    assert events[int((d_status.settings._AWARENESS_TIME + \
                      ((TEST_TIMESPAN-10-d_status.settings._AWARENESS_TIME)/2))/DT_DMON)].names[0] == EventName.driverUnresponsive  # 断言事件名称为无反应

  # engaged, down to orange, driver pays attention, back to normal; then down to orange, driver touches wheel  # 驾驶员从橙色警告恢复到正常，然后再次分心，触摸方向盘
  #  - should have short orange recovery time and no green afterwards; wheel touch only recovers when paying attention  # - 应该有短暂的橙色恢复时间，之后没有绿色；触摸方向盘仅在专注时恢复
  def test_normal_driver(self):
    ds_vector = [msg_DISTRACTED] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_ATTENTIVE] * int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON) + \
                [msg_DISTRACTED] * int((DISTRACTED_SECONDS_TO_ORANGE+2)/DT_DMON) + \
                [msg_ATTENTIVE] * (int(TEST_TIMESPAN/DT_DMON)-int((DISTRACTED_SECONDS_TO_ORANGE*3+2)/DT_DMON))  # 驾驶员状态向量
    interaction_vector = [car_interaction_NOT_DETECTED] * int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON) + \
                         [car_interaction_DETECTED] * (int(TEST_TIMESPAN/DT_DMON)-int(DISTRACTED_SECONDS_TO_ORANGE*3/DT_DMON))  # 驾驶员与汽车的交互向量
    events, _ = self._run_seq(ds_vector, interaction_vector, always_true, always_false)  # 运行测试序列
    assert len(events[int(DISTRACTED_SECONDS_TO_ORANGE*0.5/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((DISTRACTED_SECONDS_TO_ORANGE-0.1)/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心
    assert len(events[int(DISTRACTED_SECONDS_TO_ORANGE*1.5/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((DISTRACTED_SECONDS_TO_ORANGE*3-0.1)/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心
    assert events[int((DISTRACTED_SECONDS_TO_ORANGE*3+0.1)/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心
    assert len(events[int((DISTRACTED_SECONDS_TO_ORANGE*3+2.5)/DT_DMON)]) == 0  # 断言在特定时间没有事件

  # engaged, down to orange, driver dodges camera, then comes back still distracted, down to red,  # 驾驶员躲避摄像头，然后回来仍然分心，降到红色
  #                          driver dodges, and then touches wheel to no avail, disengages and reengages  # 驾驶员躲避，然后触摸方向盘无效，解除并重新接入
  #  - orange/red alert should remain after disappearance, and only disengaging clears red  # - 橙色/红色警告在消失后应保持，只有解除接入才能清除红色
  def test_biggest_comma_fan(self):
    _invisible_time = 2  # seconds  # 隐形时间，单位为秒
    ds_vector = always_distracted[:]  # 驾驶员状态向量
    interaction_vector = always_false[:]  # 驾驶员与汽车的交互向量
    op_vector = always_true[:]  # 驾驶员操作向量
    ds_vector[int(DISTRACTED_SECONDS_TO_ORANGE/DT_DMON):int((DISTRACTED_SECONDS_TO_ORANGE+_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)  # 在隐形时间段内未检测到面部
    ds_vector[int((DISTRACTED_SECONDS_TO_RED+_invisible_time)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time)/DT_DMON)] \
                                                        = [msg_NO_FACE_DETECTED] * int(_invisible_time/DT_DMON)  # 在隐形时间段内未检测到面部
    interaction_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+0.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)] \
                                                        = [True] * int(1/DT_DMON)  # 驾驶员交互向量
    op_vector[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+2.5)/DT_DMON):int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3)/DT_DMON)] \
                                                        = [False] * int(0.5/DT_DMON)  # 驾驶员操作向量
    events, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)  # 运行测试序列
    assert events[int((DISTRACTED_SECONDS_TO_ORANGE+0.5*_invisible_time)/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心
    assert events[int((DISTRACTED_SECONDS_TO_RED+1.5*_invisible_time)/DT_DMON)].names[0] == EventName.driverDistracted  # 断言事件名称为分心
    assert events[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+1.5)/DT_DMON)].names[0] == EventName.driverDistracted  # 断言事件名称为分心
    assert len(events[int((DISTRACTED_SECONDS_TO_RED+2*_invisible_time+3.5)/DT_DMON)]) == 0  # 断言在特定时间没有事件

  # engaged, invisible driver, down to orange, driver touches wheel; then down to orange again, driver appears  # 驾驶员隐形，降到橙色，触摸方向盘；然后再次降到橙色，驾驶员出现
  #  - both actions should clear the alert, but momentary appearance should not  # - 两个动作都应清除警报，但瞬间出现不应清除
  def test_sometimes_transparent_commuter(self):
    _visible_time = np.random.choice([0.5, 10])  # 随机选择可见时间
    ds_vector = always_no_face[:]*2  # 始终未检测到面部
    interaction_vector = always_false[:]*2  # 驾驶员与汽车的交互向量
    ds_vector[int((2*INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON):int((2*INVISIBLE_SECONDS_TO_ORANGE+1+_visible_time)/DT_DMON)] = \
                                                                                             [msg_ATTENTIVE] * int(_visible_time/DT_DMON)  # 在可见时间段内驾驶员专注
    interaction_vector[int((INVISIBLE_SECONDS_TO_ORANGE)/DT_DMON):int((INVISIBLE_SECONDS_TO_ORANGE+1)/DT_DMON)] = [True] * int(1/DT_DMON)  # 驾驶员交互向量
    events, _ = self._run_seq(ds_vector, interaction_vector, 2*always_true, 2*always_false)  # 运行测试序列
    assert len(events[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)].names[0] == EventName.promptDriverUnresponsive  # 断言事件名称为提示无反应
    assert len(events[int((INVISIBLE_SECONDS_TO_ORANGE+0.1)/DT_DMON)]) == 0  # 断言在特定时间没有事件
    if _visible_time == 0.5:  # 如果可见时间为0.5秒
      assert events[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)].names[0] == EventName.promptDriverUnresponsive  # 断言事件名称为提示无反应
      assert events[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)].names[0] == EventName.preDriverUnresponsive  # 断言事件名称为预警无反应
    elif _visible_time == 10:  # 如果可见时间为10秒
      assert events[int((INVISIBLE_SECONDS_TO_ORANGE*2+1-0.1)/DT_DMON)].names[0] == EventName.promptDriverUnresponsive  # 断言事件名称为提示无反应
      assert len(events[int((INVISIBLE_SECONDS_TO_ORANGE*2+1+0.1+_visible_time)/DT_DMON)]) == 0  # 断言在特定时间没有事件

  # engaged, invisible driver, down to red, driver appears and then touches wheel, then disengages/reengages  # 驾驶员隐形，降到红色，驾驶员出现并触摸方向盘，然后解除/重新接入
  #  - only disengage will clear the alert  # - 只有解除接入才能清除警报
  def test_last_second_responder(self):
    _visible_time = 2  # seconds  # 可见时间，单位为秒
    ds_vector = always_no_face[:]  # 始终未检测到面部
    interaction_vector = always_false[:]  # 驾驶员与汽车的交互向量
    op_vector = always_true[:]  # 驾驶员操作向量
    ds_vector[int(INVISIBLE_SECONDS_TO_RED/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON)] = [msg_ATTENTIVE] * int(_visible_time/DT_DMON)  # 在可见时间段内驾驶员专注
    interaction_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON)] = [True] * int(1/DT_DMON)  # 驾驶员交互向量
    op_vector[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1)/DT_DMON):int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)] = [False] * int(0.5/DT_DMON)  # 驾驶员操作向量
    events, _ = self._run_seq(ds_vector, interaction_vector, op_vector, always_false)  # 运行测试序列
    assert len(events[int(INVISIBLE_SECONDS_TO_ORANGE*0.5/DT_DMON)]) == 0  # 断言在特定时间没有事件
    assert events[int((INVISIBLE_SECONDS_TO_ORANGE-0.1)/DT_DMON)].names[0] == EventName.promptDriverUnresponsive  # 断言事件名称为提示无反应
    assert events[int((INVISIBLE_SECONDS_TO_RED-0.1)/DT_DMON)].names[0] == EventName.driverUnresponsive  # 断言事件名称为无反应
    assert events[int((INVISIBLE_SECONDS_TO_RED+0.5*_visible_time)/DT_DMON)].names[0] == EventName.driverUnresponsive  # 断言事件名称为无反应
    assert events[int((INVISIBLE_SECONDS_TO_RED+_visible_time+0.5)/DT_DMON)].names[0] == EventName.driverUnresponsive  # 断言事件名称为无反应
    assert len(events[int((INVISIBLE_SECONDS_TO_RED+_visible_time+1+0.1)/DT_DMON)]) == 0  # 断言在特定时间没有事件

  # disengaged, always distracted driver  # 驾驶员解除接入，始终分心
  #  - dm should stay quiet when not engaged  # - 驾驶员监控在未接入时应保持安静
  def test_pure_dashcam_user(self):
    events, _ = self._run_seq(always_distracted, always_false, always_false, always_false)  # 运行测试序列
    assert sum(len(event) for event in events) == 0  # 断言没有事件

  # engaged, car stops at traffic light, down to orange, no action, then car starts moving  # 驾驶员接入，汽车在红绿灯处停下，降到橙色，无动作，然后汽车开始移动
  #  - should only reach green when stopped, but continues counting down on launch  # - 只有在停止时才能达到绿色，但在启动时继续倒计时
  def test_long_traffic_light_victim(self):
    _redlight_time = 60  # seconds  # 红灯时间，单位为秒
    standstill_vector = always_true[:]  # 停止状态向量
    standstill_vector[int(_redlight_time/DT_DMON):] = [False] * int((TEST_TIMESPAN-_redlight_time)/DT_DMON)  # 在红灯时间后，汽车开始移动
    events, d_status = self._run_seq(always_distracted, always_false, always_true, standstill_vector)  # 运行测试序列
    assert events[int((d_status.settings._DISTRACTED_TIME-d_status.settings._DISTRACTED_PRE_TIME_TILL_TERMINAL+1)/DT_DMON)].names[0] == \
                                                                                                                    EventName.preDriverDistracted  # 断言事件名称为预警分心
    assert events[int((_redlight_time-0.1)/DT_DMON)].names[0] == EventName.preDriverDistracted  # 断言事件名称为预警分心
    assert events[int((_redlight_time+0.5)/DT_DMON)].names[0] == EventName.promptDriverDistracted  # 断言事件名称为提示分心

  # engaged, model is somehow uncertain and driver is distracted  # 驾驶员接入，模型不确定且驾驶员分心
  #  - should fall back to wheel touch after uncertain alert  # - 在不确定警报后应回退到触摸方向盘
  def test_somehow_indecisive_model(self):
    ds_vector = [msg_DISTRACTED_BUT_SOMEHOW_UNCERTAIN] * int(TEST_TIMESPAN/DT_DMON)  # 驾驶员状态向量
    interaction_vector = always_false[:]  # 驾驶员与汽车的交互向量
    events, d_status = self._run_seq(ds_vector, interaction_vector, always_true, always_false)  # 运行测试序列
    assert EventName.preDriverUnresponsive in \
                              events[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*d_status.settings._HI_STD_FALLBACK_TIME-0.1)/DT_DMON)].names  # 断言事件名称为预警无反应
    assert EventName.promptDriverUnresponsive in \
                              events[int((INVISIBLE_SECONDS_TO_ORANGE-1+DT_DMON*d_status.settings._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)].names  # 断言事件名称为提示无反应
    assert EventName.driverUnresponsive in \
                              events[int((INVISIBLE_SECONDS_TO_RED-1+DT_DMON*d_status.settings._HI_STD_FALLBACK_TIME+0.1)/DT_DMON)].names  # 断言事件名称为无反应
