from cereal import custom, car
from common.conversions import Conversions as CV

EventName = car.CarEvent.EventName
HandsOnWheelState = custom.DriverMonitoringStateSP.HandsOnWheelState

_PRE_ALERT_THRESHOLD = 150  # 15秒
_PROMPT_ALERT_THRESHOLD = 300  # 30秒
_TERMINAL_ALERT_THRESHOLD = 600  # 60秒

_MIN_MONITORING_SPEED = 10 * CV.KPH_TO_MS  # 速度低于10公里每小时时不进行监控


class HandsOnWheelStatus():
  def __init__(self):
    self.hands_on_wheel_state = HandsOnWheelState.none  # 初始化方向盘状态为无
    self.hands_off_wheel_cnt = 0  # 初始化离开方向盘计数器

  def update(self, events, steering_wheel_engaged, ctrl_active, v_ego, always_on, hands_on_wheel_monitoring):
    # 更新方向盘状态
    if v_ego < _MIN_MONITORING_SPEED or \
       (not always_on and not ctrl_active) or \
       (always_on and not ctrl_active and self.hands_on_wheel_state == HandsOnWheelState.terminal) or \
       not hands_on_wheel_monitoring:
      self.hands_on_wheel_state = HandsOnWheelState.none  # 设置状态为无
      self.hands_off_wheel_cnt = 0  # 重置计数器
      return

    if steering_wheel_engaged:
      # 驾驶员手放在方向盘上
      self.hands_on_wheel_state = HandsOnWheelState.ok  # 设置状态为正常
      self.hands_off_wheel_cnt = 0  # 重置计数器
      return

    self.hands_off_wheel_cnt += 1  # 增加离开方向盘计数
    alert = None  # 初始化警报为无

    if self.hands_off_wheel_cnt >= _TERMINAL_ALERT_THRESHOLD:
      # 终极红色警报：需要重新接管
      self.hands_on_wheel_state = HandsOnWheelState.terminal  # 设置状态为终极
      alert = EventName.keepHandsOnWheel  # 触发保持手在方向盘上的事件
    elif self.hands_off_wheel_cnt >= _PROMPT_ALERT_THRESHOLD:
      # 提示橙色警报
      self.hands_on_wheel_state = HandsOnWheelState.critical  # 设置状态为关键
      alert = EventName.promptKeepHandsOnWheel  # 触发提示保持手在方向盘上的事件
    elif self.hands_off_wheel_cnt >= _PRE_ALERT_THRESHOLD:
      # 预警绿色警报
      self.hands_on_wheel_state = HandsOnWheelState.warning  # 设置状态为警告
      alert = EventName.preKeepHandsOnWheel  # 触发预警保持手在方向盘上的事件
    else:
      # 离开方向盘的时间在可接受范围内
      self.hands_on_wheel_state = HandsOnWheelState.minor  # 设置状态为轻微

    if alert is not None:
      events.add(alert)  # 添加警报事件
