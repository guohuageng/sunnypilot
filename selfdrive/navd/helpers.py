from __future__ import annotations

import json
import math
from typing import Any, cast

from openpilot.common.conversions import Conversions
from openpilot.common.numpy_fast import clip
from openpilot.common.params import Params

# 方向选项
DIRECTIONS = ('left', 'right', 'straight')
# 可修改的方向选项
MODIFIABLE_DIRECTIONS = ('left', 'right')

# 地球平均半径
EARTH_MEAN_RADIUS = 6371007.2
# 速度单位转换
SPEED_CONVERSIONS = {
    'km/h': Conversions.KPH_TO_MS,
    'mph': Conversions.MPH_TO_MS,
}

class Coordinate:
  def __init__(self, latitude: float, longitude: float) -> None:
    self.latitude = latitude  # 纬度
    self.longitude = longitude  # 经度
    self.annotations: dict[str, float] = {}  # 注释字典

  @classmethod
  def from_mapbox_tuple(cls, t: tuple[float, float]) -> Coordinate:
    # 从Mapbox元组创建坐标
    return cls(t[1], t[0])

  def as_dict(self) -> dict[str, float]:
    # 将坐标转换为字典
    return {'latitude': self.latitude, 'longitude': self.longitude}

  def __str__(self) -> str:
    # 字符串表示
    return f'Coordinate({self.latitude}, {self.longitude})'

  def __repr__(self) -> str:
    # 代表字符串
    return self.__str__()

  def __eq__(self, other) -> bool:
    # 判断两个坐标是否相等
    if not isinstance(other, Coordinate):
      return False
    return (self.latitude == other.latitude) and (self.longitude == other.longitude)

  def __sub__(self, other: Coordinate) -> Coordinate:
    # 坐标相减
    return Coordinate(self.latitude - other.latitude, self.longitude - other.longitude)

  def __add__(self, other: Coordinate) -> Coordinate:
    # 坐标相加
    return Coordinate(self.latitude + other.latitude, self.longitude + other.longitude)

  def __mul__(self, c: float) -> Coordinate:
    # 坐标与标量相乘
    return Coordinate(self.latitude * c, self.longitude * c)

  def dot(self, other: Coordinate) -> float:
    # 计算点积
    return self.latitude * other.latitude + self.longitude * other.longitude

  def distance_to(self, other: Coordinate) -> float:
    # 计算到另一个坐标的距离（使用Haversine公式）
    dlat = math.radians(other.latitude - self.latitude)
    dlon = math.radians(other.longitude - self.longitude)

    haversine_dlat = math.sin(dlat / 2.0)
    haversine_dlat *= haversine_dlat
    haversine_dlon = math.sin(dlon / 2.0)
    haversine_dlon *= haversine_dlon

    y = haversine_dlat \
             + math.cos(math.radians(self.latitude)) \
             * math.cos(math.radians(other.latitude)) \
             * haversine_dlon
    x = 2 * math.asin(math.sqrt(y))
    return x * EARTH_MEAN_RADIUS

def minimum_distance(a: Coordinate, b: Coordinate, p: Coordinate):
  # 计算点p到线段ab的最小距离
  if a.distance_to(b) < 0.01:
    return a.distance_to(p)

  ap = p - a
  ab = b - a
  t = clip(ap.dot(ab) / ab.dot(ab), 0.0, 1.0)
  projection = a + ab * t
  return projection.distance_to(p)

def distance_along_geometry(geometry: list[Coordinate], pos: Coordinate) -> float:
  # 计算当前位置到几何形状的距离
  if len(geometry) <= 2:
    return geometry[0].distance_to(pos)

  # 1. 找到离当前位置最近的线段
  # 2. 总距离是到最近线段起点的距离加上所有之前线段的距离
  total_distance = 0.0
  total_distance_closest = 0.0
  closest_distance = 1e9

  for i in range(len(geometry) - 1):
    d = minimum_distance(geometry[i], geometry[i + 1], pos)

    if d < closest_distance:
      closest_distance = d
      total_distance_closest = total_distance + geometry[i].distance_to(pos)

    total_distance += geometry[i].distance_to(geometry[i + 1])

  return total_distance_closest

def coordinate_from_param(param: str, params: Params = None) -> Coordinate | None:
  # 从参数中获取坐标
  if params is None:
    params = Params()

  json_str = params.get(param)
  if json_str is None:
    return None

  pos = json.loads(json_str)
  if 'latitude' not in pos or 'longitude' not in pos:
    return None

  return Coordinate(pos['latitude'], pos['longitude'])

def string_to_direction(direction: str) -> str:
  # 将字符串方向转换为方向
  for d in DIRECTIONS:
    if d in direction:
      if 'slight' in direction and d in MODIFIABLE_DIRECTIONS:
        return 'slight' + d.capitalize()
      return d
  return 'none'

def maxspeed_to_ms(maxspeed: dict[str, str | float]) -> float:
  # 将最大速度转换为米每秒
  unit = cast(str, maxspeed['unit'])
  speed = cast(float, maxspeed['speed'])
  return SPEED_CONVERSIONS[unit] * speed

def field_valid(dat: dict, field: str) -> bool:
  # 检查字段是否有效
  return field in dat and dat[field] is not None

def parse_banner_instructions(banners: Any, distance_to_maneuver: float = 0.0) -> dict[str, Any] | None:
  # 解析横幅指令
  if not len(banners):
    return None

  instruction = {}

  # 一个段落可以包含多个横幅，找到我们现在需要显示的一个
  current_banner = banners[0]
  for banner in banners:
    if distance_to_maneuver < banner['distanceAlongGeometry']:
      current_banner = banner

  # 只有在足够接近操作时才显示横幅
  instruction['showFull'] = distance_to_maneuver < current_banner['distanceAlongGeometry']

  # 主要信息
  p = current_banner['primary']
  if field_valid(p, 'text'):
    instruction['maneuverPrimaryText'] = p['text']
  if field_valid(p, 'type'):
    instruction['maneuverType'] = p['type']
  if field_valid(p, 'modifier'):
    instruction['maneuverModifier'] = p['modifier']

  # 次要信息
  if field_valid(current_banner, 'secondary'):
    instruction['maneuverSecondaryText'] = current_banner['secondary']['text']

  # 车道线信息
  if field_valid(current_banner, 'sub'):
    lanes = []
    for component in current_banner['sub']['components']:
      if component['type'] != 'lane':
        continue

      lane = {
        'active': component['active'],
        'directions': [string_to_direction(d) for d in component['directions']],
      }

      if field_valid(component, 'active_direction'):
        lane['activeDirection'] = string_to_direction(component['active_direction'])

      lanes.append(lane)
    instruction['lanes'] = lanes

  return instruction
