#!/usr/bin/env python3
import json
import math
import os
import threading

import requests

import cereal.messaging as messaging
from cereal import log
from openpilot.common.api import Api
from openpilot.common.params import Params
from openpilot.common.realtime import Ratekeeper
from openpilot.selfdrive.navd.helpers import (Coordinate, coordinate_from_param,
                                    distance_along_geometry, maxspeed_to_ms,
                                    minimum_distance,
                                    parse_banner_instructions)
from openpilot.common.swaglog import cloudlog

REROUTE_DISTANCE = 25  # 重新规划路线的距离阈值
MANEUVER_TRANSITION_THRESHOLD = 10  # 操作过渡阈值
REROUTE_COUNTER_MIN = 3  # 重新规划计数的最小值


class RouteEngine:
  def __init__(self, sm, pm):
    self.sm = sm  # 状态管理器
    self.pm = pm  # 发布管理器

    self.params = Params()  # 参数管理

    # 从参数中获取最后的GPS位置
    self.last_position = coordinate_from_param("LastGPSPosition", self.params)
    self.last_bearing = None  # 最后方向

    self.gps_ok = False  # GPS状态
    self.localizer_valid = False  # 本地化状态

    self.nav_destination = None  # 导航目的地
    self.step_idx = None  # 当前步骤索引
    self.route = None  # 路线
    self.route_geometry = None  # 路线几何形状

    self.recompute_backoff = 0  # 重新计算的退避时间
    self.recompute_countdown = 0  # 重新计算倒计时

    self.ui_pid = None  # UI进程ID

    self.reroute_counter = 0  # 重新规划计数器


    self.api = None  # API对象
    self.mapbox_token = None  # Mapbox令牌
    if "MAPBOX_TOKEN" in os.environ:
      self.mapbox_token = os.environ["MAPBOX_TOKEN"]  # 从环境变量获取Mapbox令牌
      self.mapbox_host = "https://api.mapbox.com"  # Mapbox主机地址
    else:
      self.api = Api(self.params.get("DongleId", encoding='utf8'))  # 获取API
      self.mapbox_host = "https://maps.comma.ai"  # 默认地图主机地址

    if self.mapbox_token != "" and self.params.get("CustomMapboxTokenSk") is not None:
      self.mapbox_token = self.params.get("CustomMapboxTokenSk")  # 自定义Mapbox令牌
      self.mapbox_host = "https://api.mapbox.com"  # Mapbox主机地址

  def update(self):
    self.sm.update(0)  # 更新状态管理器

    if self.sm.updated["managerState"]:
      ui_pid = [p.pid for p in self.sm["managerState"].processes if p.name == "ui" and p.running]
      if ui_pid:
        if self.ui_pid and self.ui_pid != ui_pid[0]:
          cloudlog.warning("UI restarting, sending route")  # UI重启，发送路线
          threading.Timer(5.0, self.send_route).start()  # 5秒后发送路线
        self.ui_pid = ui_pid[0]  # 更新UI进程ID

    self.update_location()  # 更新位置
    try:
      self.recompute_route()  # 重新计算路线
      self.send_instruction()  # 发送指令
    except Exception:
      cloudlog.exception("navd.failed_to_compute")  # 计算失败的异常处理

  def update_location(self):
    location = self.sm['liveLocationKalman']  # 获取实时位置
    self.gps_ok = location.gpsOK  # 更新GPS状态

    self.localizer_valid = (location.status == log.LiveLocationKalman.Status.valid) and location.positionGeodetic.valid  # 更新本地化状态

    if self.localizer_valid:
      self.last_bearing = math.degrees(location.calibratedOrientationNED.value[2])  # 更新最后方向
      self.last_position = Coordinate(location.positionGeodetic.value[0], location.positionGeodetic.value[1])  # 更新最后位置

  def recompute_route(self):
    if self.last_position is None:
      return  # 如果最后位置为空，返回

    new_destination = coordinate_from_param("NavDestination", self.params)  # 获取新的目的地
    if new_destination is None:
      self.clear_route()  # 清除路线
      self.reset_recompute_limits()  # 重置重新计算限制
      return

    should_recompute = self.should_recompute()  # 判断是否需要重新计算
    if new_destination != self.nav_destination:
      cloudlog.warning(f"Got new destination from NavDestination param {new_destination}")  # 从参数获取新的目的地
      should_recompute = True

    # 在隧道中GPS漂移时不重新计算
    if not self.gps_ok and self.step_idx is not None:
      return

    if self.recompute_countdown == 0 and should_recompute:
      self.recompute_countdown = 2**self.recompute_backoff  # 设置重新计算倒计时
      self.recompute_backoff = min(6, self.recompute_backoff + 1)  # 增加退避时间
      self.calculate_route(new_destination)  # 计算新路线
      self.reroute_counter = 0  # 重置重新规划计数器
    else:
      self.recompute_countdown = max(0, self.recompute_countdown - 1)  # 更新倒计时

  def calculate_route(self, destination):
    cloudlog.warning(f"Calculating route {self.last_position} -> {destination}")  # 计算路线
    self.nav_destination = destination  # 设置导航目的地

    lang = self.params.get('LanguageSetting', encoding='utf8')  # 获取语言设置
    if lang is not None:
      lang = lang.replace('main_', '')  # 移除前缀

    token = self.mapbox_token  # 获取Mapbox令牌
    if token is None:
      token = self.api.get_token()  # 从API获取令牌

    params = {
      'access_token': token,  # 访问令牌
      'annotations': 'maxspeed',  # 注释类型
      'geometries': 'geojson',  # 几何类型
      'overview': 'full',  # 概览类型
      'steps': 'true',  # 是否包含步骤
      'banner_instructions': 'true',  # 是否包含横幅指令
      'alternatives': 'false',  # 是否包含替代路线
      'language': lang,  # 语言设置
    }

    # TODO: 将途经点移动到NavDestination参数中？
    waypoints = self.params.get('NavDestinationWaypoints', encoding='utf8')  # 获取途经点
    waypoint_coords = []
    if waypoints is not None and len(waypoints) > 0:
      waypoint_coords = json.loads(waypoints)  # 解析途经点

    coords = [
      (self.last_position.longitude, self.last_position.latitude),  # 添加最后位置
      *waypoint_coords,  # 添加途经点
      (destination.longitude, destination.latitude)  # 添加目的地
    ]
    params['waypoints'] = f'0;{len(coords)-1}'  # 设置途经点参数
    if self.last_bearing is not None:
      params['bearings'] = f"{(self.last_bearing + 360) % 360:.0f},90" + (';'*(len(coords)-1))  # 设置方向参数

    coords_str = ';'.join([f'{lon},{lat}' for lon, lat in coords])  # 将坐标转换为字符串
    url = self.mapbox_host + '/directions/v5/mapbox/driving-traffic/' + coords_str  # 构建请求URL
    try:
      resp = requests.get(url, params=params, timeout=10)  # 发送请求
      if resp.status_code != 200:
        cloudlog.event("API request failed", status_code=resp.status_code, text=resp.text, error=True)  # 请求失败日志
      resp.raise_for_status()  # 检查请求状态

      r = resp.json()  # 解析响应
      if len(r['routes']):
        self.route = r['routes'][0]['legs'][0]['steps']  # 获取路线步骤
        self.route_geometry = []  # 初始化路线几何形状

        maxspeed_idx = 0  # 最大速度索引
        maxspeeds = r['routes'][0]['legs'][0]['annotation']['maxspeed']  # 获取最大速度注释

        # 转换坐标
        for step in self.route:
          coords = []

          for c in step['geometry']['coordinates']:
            coord = Coordinate.from_mapbox_tuple(c)  # 从Mapbox元组转换坐标

            # 最后一步没有最大速度
            if (maxspeed_idx < len(maxspeeds)):
              maxspeed = maxspeeds[maxspeed_idx]
              if ('unknown' not in maxspeed) and ('none' not in maxspeed):
                coord.annotations['maxspeed'] = maxspeed_to_ms(maxspeed)  # 设置最大速度注释

            coords.append(coord)  # 添加坐标
            maxspeed_idx += 1  # 更新最大速度索引

          self.route_geometry.append(coords)  # 添加路线几何形状
          maxspeed_idx -= 1  # 每个段落以相同的坐标结束

        self.step_idx = 0  # 初始化步骤索引
      else:
        cloudlog.warning("Got empty route response")  # 收到空路线响应
        self.clear_route()  # 清除路线

      # 清除途经点以避免重新规划包含过去的途经点
      # TODO: 只有在经过途经点后才清除
      self.params.remove('NavDestinationWaypoints')  # 移除途经点参数

    except requests.exceptions.RequestException:
      cloudlog.exception("failed to get route")  # 获取路线失败的异常处理
      self.clear_route()  # 清除路线

    self.send_route()  # 发送路线

  def send_instruction(self):
    msg = messaging.new_message('navInstruction', valid=True)  # 创建新的导航指令消息

    if self.step_idx is None:
      msg.valid = False  # 如果步骤索引为空，设置消息无效
      self.pm.send('navInstruction', msg)  # 发送消息
      return

    step = self.route[self.step_idx]  # 获取当前步骤
    geometry = self.route_geometry[self.step_idx]  # 获取当前步骤几何形状
    along_geometry = distance_along_geometry(geometry, self.last_position)  # 计算沿几何形状的距离
    distance_to_maneuver_along_geometry = step['distance'] - along_geometry  # 计算到操作的距离

    # 横幅指令是针对下一个操作步骤的，不使用空的最后一步
    banner_step = step
    if not len(banner_step['bannerInstructions']) and self.step_idx == len(self.route) - 1:
      banner_step = self.route[max(self.step_idx - 1, 0)]  # 获取上一个步骤作为横幅指令

    # 当前指令
    msg.navInstruction.maneuverDistance = distance_to_maneuver_along_geometry  # 设置操作距离
    instruction = parse_banner_instructions(banner_step['bannerInstructions'], distance_to_maneuver_along_geometry)  # 解析横幅指令
    if instruction is not None:
      for k,v in instruction.items():
        setattr(msg.navInstruction, k, v)  # 设置指令属性

    # 所有指令
    maneuvers = []
    for i, step_i in enumerate(self.route):
      if i < self.step_idx:
        distance_to_maneuver = -sum(self.route[j]['distance'] for j in range(i+1, self.step_idx)) - along_geometry  # 计算到操作的距离
      elif i == self.step_idx:
        distance_to_maneuver = distance_to_maneuver_along_geometry  # 当前操作的距离
      else:
        distance_to_maneuver = distance_to_maneuver_along_geometry + sum(self.route[j]['distance'] for j in range(self.step_idx+1, i+1))  # 计算未来操作的距离

      instruction = parse_banner_instructions(step_i['bannerInstructions'], distance_to_maneuver)  # 解析指令
      if instruction is None:
        continue
      maneuver = {'distance': distance_to_maneuver}  # 创建操作字典
      if 'maneuverType' in instruction:
        maneuver['type'] = instruction['maneuverType']  # 设置操作类型
      if 'maneuverModifier' in instruction:
        maneuver['modifier'] = instruction['maneuverModifier']  # 设置操作修饰符
      maneuvers.append(maneuver)  # 添加操作

    msg.navInstruction.allManeuvers = maneuvers  # 设置所有操作

    # 计算剩余时间和距离
    remaining = 1.0 - along_geometry / max(step['distance'], 1)  # 计算剩余比例
    total_distance = step['distance'] * remaining  # 计算总距离
    total_time = step['duration'] * remaining  # 计算总时间

    if step['duration_typical'] is None:
      total_time_typical = total_time  # 如果没有典型时间，使用总时间
    else:
      total_time_typical = step['duration_typical'] * remaining  # 计算典型时间

    # 累加未来步骤的总和
    for i in range(self.step_idx + 1, len(self.route)):
      total_distance += self.route[i]['distance']  # 累加距离
      total_time += self.route[i]['duration']  # 累加时间
      if self.route[i]['duration_typical'] is None:
        total_time_typical += self.route[i]['duration']  # 累加典型时间
      else:
        total_time_typical += self.route[i]['duration_typical']  # 累加典型时间

    msg.navInstruction.distanceRemaining = total_distance  # 设置剩余距离
    msg.navInstruction.timeRemaining = total_time  # 设置剩余时间
    msg.navInstruction.timeRemainingTypical = total_time_typical  # 设置剩余典型时间

    # 限速
    closest_idx, closest = min(enumerate(geometry), key=lambda p: p[1].distance_to(self.last_position))  # 找到最近的点
    if closest_idx > 0:
      # 如果我们还没有超过最近的点，显示上一个点
      if along_geometry < distance_along_geometry(geometry, geometry[closest_idx]):
        closest = geometry[closest_idx - 1]  # 更新最近的点

    if ('maxspeed' in closest.annotations) and self.localizer_valid:
      msg.navInstruction.speedLimit = closest.annotations['maxspeed']  # 设置限速

    # 限速标志类型
    if 'speedLimitSign' in step:
      if step['speedLimitSign'] == 'mutcd':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.mutcd  # 设置标志类型为mutcd
      elif step['speedLimitSign'] == 'vienna':
        msg.navInstruction.speedLimitSign = log.NavInstruction.SpeedLimitSign.vienna  # 设置标志类型为vienna

    self.pm.send('navInstruction', msg)  # 发送导航指令消息

    # 转换到下一个路线段
    if distance_to_maneuver_along_geometry < -MANEUVER_TRANSITION_THRESHOLD:
      if self.step_idx + 1 < len(self.route):
        self.step_idx += 1  # 更新步骤索引
        self.reset_recompute_limits()  # 重置重新计算限制
      else:
        cloudlog.warning("Destination reached")  # 到达目的地

        # 如果远离目的地则清除路线
        dist = self.nav_destination.distance_to(self.last_position)  # 计算到目的地的距离
        if dist > REROUTE_DISTANCE:
          self.params.remove("NavDestination")  # 移除目的地参数
          self.clear_route()  # 清除路线

  def send_route(self):
    coords = []  # 初始化坐标列表

    if self.route is not None:
      for path in self.route_geometry:
        coords += [c.as_dict() for c in path]  # 将几何形状转换为字典并添加到坐标列表

    msg = messaging.new_message('navRoute', valid=True)  # 创建新的导航路线消息
    msg.navRoute.coordinates = coords  # 设置坐标
    self.pm.send('navRoute', msg)  # 发送导航路线消息

  def clear_route(self):
    self.route = None  # 清除路线
    self.route_geometry = None  # 清除路线几何形状
    self.step_idx = None  # 清除步骤索引
    self.nav_destination = None  # 清除导航目的地

  def reset_recompute_limits(self):
    self.recompute_backoff = 0  # 重置重新计算退避时间
    self.recompute_countdown = 0  # 重置重新计算倒计时

  def should_recompute(self):
    if self.step_idx is None or self.route is None:
      return True  # 如果步骤索引或路线为空，返回True

    # 在最后一个段落中不重新计算，假设目的地已到达
    if self.step_idx == len(self.route) - 1:
      return False

    # 计算当前路径中所有线段的最近距离
    min_d = REROUTE_DISTANCE + 1  # 初始化最小距离
    path = self.route_geometry[self.step_idx]  # 获取当前路径
    for i in range(len(path) - 1):
      a = path[i]  # 当前点
      b = path[i + 1]  # 下一个点

      if a.distance_to(b) < 1.0:
        continue  # 如果距离小于1.0，跳过

      min_d = min(min_d, minimum_distance(a, b, self.last_position))  # 更新最小距离

    if min_d > REROUTE_DISTANCE:
      self.reroute_counter += 1  # 更新重新规划计数器
    else:
      self.reroute_counter = 0  # 重置重新规划计数器
    return self.reroute_counter > REROUTE_COUNTER_MIN  # 返回是否超过最小重新规划计数

    # TODO: 检查在段落中是否走错方向


def main():
  pm = messaging.PubMaster(['navInstruction', 'navRoute'])  # 创建发布管理器
  sm = messaging.SubMaster(['liveLocationKalman', 'managerState'])  # 创建订阅管理器

  rk = Ratekeeper(1.0)  # 创建速率控制器
  route_engine = RouteEngine(sm, pm)  # 创建路线引擎
  while True:
    route_engine.update()  # 更新路线引擎
    rk.keep_time()  # 保持时间


if __name__ == "__main__":
  main()  # 运行主函数
