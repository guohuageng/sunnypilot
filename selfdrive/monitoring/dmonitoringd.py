#!/usr/bin/env python3
import gc

import cereal.messaging as messaging
from openpilot.common.params import Params
from openpilot.common.realtime import set_realtime_priority
from openpilot.selfdrive.monitoring.helpers import DriverMonitoring


def dmonitoringd_thread():
  gc.disable()  # 禁用垃圾回收
  set_realtime_priority(2)  # 设置实时优先级为2

  params = Params()  # 初始化参数管理
  pm = messaging.PubMaster(['driverMonitoringState', 'driverMonitoringStateSP'])  # 创建发布管理器
  sm = messaging.SubMaster(['driverStateV2', 'liveCalibration', 'carState', 'controlsState', 'modelV2'], poll='driverStateV2')  # 创建订阅管理器

  # 初始化驾驶员监控
  DM = DriverMonitoring(rhd_saved=params.get_bool("IsRhdDetected"), always_on=params.get_bool("AlwaysOnDM"), hands_on_wheel_monitoring=params.get_bool("HandsOnWheelMonitoring"))

  # 20Hz <- dmonitoringmodeld
  while True:
    sm.update()  # 更新状态管理器
    if not sm.updated['driverStateV2']:
      # 当模型有新输出时进行迭代
      continue

    valid = sm.all_checks()  # 检查所有状态
    if valid:
      DM.run_step(sm)  # 运行驾驶员监控步骤

    # 发布状态
    dat = DM.get_state_packet(valid=valid)  # 获取状态数据包
    pm.send('driverMonitoringState', dat)  # 发送驾驶员监控状态

    sp_dat = DM.get_sp_state_packet(valid=valid)  # 获取状态数据包
    pm.send('driverMonitoringStateSP', sp_dat)  # 发送驾驶员监控状态SP

    # 加载实时的始终开启切换
    if sm['driverStateV2'].frameId % 40 == 1:
      DM.always_on = params.get_bool("AlwaysOnDM")  # 更新始终开启状态
      DM.hands_on_wheel_monitoring = params.get_bool("HandsOnWheelMonitoring")  # 更新方向盘监控状态

    # 每5分钟保存一次右侧驾驶虚拟切换
    if (sm['driverStateV2'].frameId % 6000 == 0 and
     DM.wheelpos_learner.filtered_stat.n > DM.settings._WHEELPOS_FILTER_MIN_COUNT and
     DM.wheel_on_right == (DM.wheelpos_learner.filtered_stat.M > DM.settings._WHEELPOS_THRESHOLD)):
      params.put_bool_nonblocking("IsRhdDetected", DM.wheel_on_right)  # 非阻塞地保存右侧驾驶状态

def main():
  dmonitoringd_thread()  # 启动驾驶员监控线程


if __name__ == '__main__':
  main()  # 运行主函数
