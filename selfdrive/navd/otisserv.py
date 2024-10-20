#!/usr/bin/env python3.8
# The MIT License
#
# Copyright (c) 2019-, Rick Lan, dragonpilot community, and a number of other of contributors.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from http.server import BaseHTTPRequestHandler, HTTPServer
from cgi import parse_header, parse_multipart
from urllib.parse import parse_qs, unquote
import json
import requests
import math
from openpilot.common.basedir import BASEDIR
from openpilot.common.params import Params
from openpilot.common.realtime import set_core_affinity
from openpilot.common.swaglog import cloudlog
params = Params()

hostName = ""
serverPort = 8082

pi = 3.1415926535897932384626
x_pi = 3.14159265358979324 * 3000.0 / 180.0
a = 6378245.0
ee = 0.00669342162296594323


class OtisServ(BaseHTTPRequestHandler):
  def do_GET(self):
    use_amap = params.get_bool("EnableAmap")  # 检查是否启用高德地图
    use_gmap = not use_amap and params.get_bool("EnableGmap")  # 检查是否启用谷歌地图

    if self.path == '/logo.png':
      self.get_logo()  # 获取logo
      return
    if self.path == '/?reset=1':
      params.put("NavDestination", "")  # 重置导航目的地
    if self.path == '/locations':
      self.get_locations()  # 获取位置
      return
    elif use_amap:
      if self.path == '/style.css':
        self.send_response(200)
        self.send_header("Content-type", "text/css")
        self.end_headers()
        self.get_amap_css()  # 获取高德地图的CSS
        return
      elif self.path == '/index.js':
        self.send_response(200)
        self.send_header("Content-type", "text/javascript")
        self.end_headers()
        self.get_amap_js()  # 获取高德地图的JS
        return
      else:
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        if self.get_amap_key() is None or self.get_amap_key_2() is None:
          self.display_page_amap_key()  # 显示高德地图密钥输入页面
          return
        if self.get_app_token() is None:
          self.display_page_app_token()  # 显示应用程序令牌输入页面
          return
        self.display_page_amap()  # 显示高德地图页面
    elif use_gmap:
      if self.path == '/style.css':
        self.send_response(200)
        self.send_header("Content-type", "text/css")
        self.end_headers()
        self.get_gmap_css()  # 获取谷歌地图的CSS
        return
      elif self.path == '/index.js':
        self.send_response(200)
        self.send_header("Content-type", "text/javascript")
        self.end_headers()
        self.get_gmap_js()  # 获取谷歌地图的JS
        return
      else:
        self.send_response(200)
        self.send_header("Content-type", "text/html")
        self.end_headers()
        if self.get_gmap_key() is None:
          self.display_page_gmap_key()  # 显示谷歌地图密钥输入页面
          return
        if self.get_app_token() is None:
          self.display_page_app_token()  # 显示应用程序令牌输入页面
          return
        self.display_page_gmap()  # 显示谷歌地图页面
    else:
      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()
      if self.get_public_token() is None:
        self.display_page_public_token()  # 显示公共令牌输入页面
        return
      if self.get_app_token() is None:
        self.display_page_app_token()  # 显示应用程序令牌输入页面
        return
      if self.path != '/locations':
        self.display_page_addr_input()  # 显示地址输入页面

  def do_POST(self):
    use_amap = params.get_bool("EnableAmap")  # 检查是否启用高德地图
    use_gmap = not use_amap and params.get_bool("EnableGmap")  # 检查是否启用谷歌地图

    postvars = self.parse_POST()  # 解析POST请求的数据
    # set_destination endpoint
    if self.path == '/set_destination':
      self.send_response(200)
      self.send_header("Content-type", "application/json")
      self.end_headers()
      response_data = {'success': True}  # 返回成功的响应
      self.wfile.write(json.dumps(response_data).encode('utf-8'))
    else:
      self.send_response(200)
      self.send_header("Content-type", "text/html")
      self.end_headers()

    if use_amap:
      # amap token
      if self.get_amap_key() is None or self.get_amap_key_2() is None:
        if postvars is None or \
                ("amap_key_val" not in postvars or postvars.get("amap_key_val")[0] == "") or \
                ("amap_key_val_2" not in postvars or postvars.get("amap_key_val_2")[0] == ""):
          self.display_page_amap_key()  # 显示高德地图密钥输入页面
          return
        params.put("AmapKey1", postvars.get("amap_key_val")[0])  # 存储高德地图密钥1
        params.put("AmapKey2", postvars.get("amap_key_val_2")[0])  # 存储高德地图密钥2

    elif use_gmap:
      # gmap token
      if self.get_gmap_key() is None:
        if postvars is None or "gmap_key_val" not in postvars or postvars.get("gmap_key_val")[0] == "":
          self.display_page_gmap_key()  # 显示谷歌地图密钥输入页面
          return
        params.put("GmapKey", postvars.get("gmap_key_val")[0])  # 存储谷歌地图密钥

    else:
      # mapbox public key
      if self.get_public_token() is None:
        if postvars is None or "pk_token_val" not in postvars or postvars.get("pk_token_val")[0] == "":
          self.display_page_public_token()  # 显示公共令牌输入页面
          return
        token = postvars.get("pk_token_val")[0]
        if "pk." not in token:
          self.display_page_public_token("Your token was incorrect!")  # 显示令牌错误消息
          return
        params.put('CustomMapboxTokenPk', token)  # 存储Mapbox公共令牌

    # app key
    if self.get_app_token() is None:
      if postvars is None or "sk_token_val" not in postvars or postvars.get("sk_token_val")[0] == "":
        self.display_page_app_token()  # 显示应用程序令牌输入页面
        return
      token = postvars.get("sk_token_val")[0]
      if "sk." not in token:
        self.display_page_app_token("Your token was incorrect!")  # 显示令牌错误消息
        return
      params.put('CustomMapboxTokenSk', token)  # 存储Mapbox应用程序令牌

    # nav confirmed
    if postvars is not None:
      if "lat" in postvars and postvars.get("lat")[0] != "" and "lon" in postvars and postvars.get("lon")[0] != "":
        lat = float(postvars.get("lat")[0])  # 获取纬度
        lng = float(postvars.get("lon")[0])  # 获取经度
        save_type = postvars.get("save_type")[0]  # 获取保存类型
        name = postvars.get("name")[0] if postvars.get("name") is not None else ""  # 获取名称
        if use_amap:
          lng, lat = self.gcj02towgs84(lng, lat)  # 将高德坐标转换为WGS84坐标
        params.put('NavDestination', "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (lat, lng, name))  # 存储导航目的地
        self.to_json(lat, lng, save_type, name)  # 将目的地保存为JSON
    if postvars is not None:
      latitude_value = postvars.get("latitude")  # 获取纬度值
      longitude_value = postvars.get("longitude")  # 获取经度值
      if latitude_value is not None and latitude_value != "" and longitude_value is not None and longitude_value != "":
        lat = float(latitude_value)  # 转换纬度为浮点数
        lng = float(longitude_value)  # 转换经度为浮点数
        save_type = "recent"  # 设置保存类型为最近
        name = postvars.get("place_name", [""])  # 获取地点名称
        params.put('NavDestination', "{\"latitude\": %f, \"longitude\": %f, \"place_name\": \"%s\"}" % (lat, lng, name))  # 存储导航目的地
        self.to_json(lat, lng, save_type, name)  # 将目的地保存为JSON
      # favorites
      if not use_gmap and "fav_val" in postvars:
        addr = postvars.get("fav_val")[0]  # 获取收藏的地址
        real_addr = None
        lon = None
        lat = None
        if addr != "favorites":
          val = params.get("ApiCache_NavDestinations", encoding='utf8')  # 获取导航目的地缓存
          if val is not None:
            val = val.rstrip('\x00')
            dests = json.loads(val)  # 解析缓存的目的地
            for item in dests:
              if "label" in item and item["label"] == addr:
                lat = item["latitude"]  # 获取纬度
                lon = item["longitude"]  # 获取经度
                real_addr = item["place_name"]  # 获取真实地址
                break
            else:
              real_addr = None
          if real_addr is not None:
            self.display_page_nav_confirmation(real_addr, lon, lat)  # 显示导航确认页面
            return
          else:
            self.display_page_addr_input("Place Not Found")  # 显示地址输入页面，提示未找到地点
            return
      # search
      if not use_gmap and "addr_val" in postvars:
        addr = postvars.get("addr_val")[0]  # 获取搜索的地址
        if addr != "":
          real_addr, lat, lon = self.query_addr(addr)  # 查询地址
          if real_addr is not None:
            self.display_page_nav_confirmation(real_addr, lon, lat)  # 显示导航确认页面
            return
          else:
            self.display_page_addr_input("Place Not Found")  # 显示地址输入页面，提示未找到地点
            return
    if self.path != '/set_destination':
      if use_amap:
        self.display_page_amap()  # 显示高德地图页面
      elif use_gmap:
        self.display_page_gmap()  # 显示谷歌地图页面
      else:
        self.display_page_addr_input()  # 显示地址输入页面

  def get_logo(self):
    self.send_response(200)
    self.send_header('Content-type','image/png')
    self.end_headers()
    f = open("%s/selfdrive/assets/img_spinner_comma.png" % BASEDIR, "rb")  # 打开logo图片
    self.wfile.write(f.read())  # 发送logo图片
    f.close()

  def get_locations(self):
    self.send_response(200)
    self.send_header('Content-type','application/json')
    self.end_headers()
    val = params.get("ApiCache_NavDestinations", encoding='utf-8')  # 获取导航目的地缓存
    if val is not None:
      self.wfile.write(val.encode('utf-8'))  # 发送导航目的地数据

  def get_gmap_css(self):
    self.wfile.write(bytes(self.get_parsed_template("gmap/style.css"), "utf-8"))  # 获取谷歌地图的CSS

  def get_gmap_js(self):
    lon, lat = self.get_last_lon_lat()  # 获取最后的经纬度
    self.wfile.write(bytes(self.get_parsed_template("gmap/index.js", {"{{lat}}": lat, "{{lon}}": lon}), "utf-8"))  # 获取谷歌地图的JS

  def get_gmap_key(self):
    token = params.get("GmapKey", encoding='utf8')  # 获取谷歌地图密钥
    if token is not None and token != "":
      return token.rstrip('\x00')
    return None

  def get_amap_css(self):
    self.wfile.write(bytes(self.get_parsed_template("amap/style.css"), "utf-8"))  # 获取高德地图的CSS

  def get_amap_js(self):
    lon, lat = self.get_last_lon_lat()  # 获取最后的经纬度
    self.wfile.write(bytes(self.get_parsed_template("amap/index.js", {"{{lat}}": lat, "{{lon}}": lon}), "utf-8"))  # 获取高德地图的JS

  def get_amap_key(self):
    token = params.get("AmapKey1", encoding='utf8')  # 获取高德地图密钥1
    if token is not None and token != "":
      return token.rstrip('\x00')
    return None

  def get_amap_key_2(self):
    token = params.get("AmapKey2", encoding='utf8')  # 获取高德地图密钥2
    if token is not None and token != "":
      return token.rstrip('\x00')
    return None

  def get_public_token(self):
    token = params.get("CustomMapboxTokenPk", encoding='utf8')  # 获取Mapbox公共令牌
    if token is not None and token != "":
      return token.rstrip('\x00')
    return None

  def get_app_token(self):
    token = params.get("CustomMapboxTokenSk", encoding='utf8')  # 获取Mapbox应用程序令牌
    if token is not None and token != "":
      return token.rstrip('\x00')
    return None

  def get_last_lon_lat(self):
    last_pos = Params().get("LastGPSPosition")  # 获取最后的GPS位置
    if last_pos is not None and last_pos != "":
      l = json.loads(last_pos)
      return l["longitude"], l["latitude"]  # 返回经纬度
    return "", ""

  def display_page_gmap_key(self):
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": self.get_parsed_template("gmap/key_input")}), "utf-8"))  # 显示谷歌地图密钥输入页面

  def display_page_amap_key(self):
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": self.get_parsed_template("amap/key_input")}), "utf-8"))  # 显示高德地图密钥输入页面

  def display_page_public_token(self, msg = ""):
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": self.get_parsed_template("public_token_input", {"{{msg}}": msg})}), "utf-8"))  # 显示公共令牌输入页面

  def display_page_app_token(self, msg = ""):
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": self.get_parsed_template("app_token_input", {"{{msg}}": msg})}), "utf-8"))  # 显示应用程序令牌输入页面

  def display_page_addr_input(self, msg = ""):
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": self.get_parsed_template("addr_input", {"{{msg}}": msg})}), "utf-8"))  # 显示地址输入页面

  def display_page_nav_confirmation(self, addr, lon, lat):
    content = self.get_parsed_template("addr_input", {"{{msg}}": ""}) + self.get_parsed_template("nav_confirmation", {"{{token}}": self.get_public_token(), "{{lon}}": lon, "{{lat}}": lat, "{{addr}}": addr})  # 显示导航确认页面
    self.wfile.write(bytes(self.get_parsed_template("body", {"{{content}}": content }), "utf-8"))

  def display_page_gmap(self):
    self.wfile.write(bytes(self.get_parsed_template("gmap/index.html", {"{{gmap_key}}": self.get_gmap_key()}), "utf-8"))  # 显示谷歌地图页面

  def display_page_amap(self):
    self.wfile.write(bytes(self.get_parsed_template("amap/index.html", {"{{amap_key}}": self.get_amap_key(), "{{amap_key_2}}": self.get_amap_key_2()}), "utf-8"))  # 显示高德地图页面

  def get_parsed_template(self, name, replace = {}):
    f = open('%s/selfdrive/navd/tpl/%s.tpl' % (BASEDIR, name), mode='r', encoding='utf-8')  # 打开模板文件
    content = f.read()
    for key in replace:
      content = content.replace(key, str(replace[key]))  # 替换模板中的占位符
    f.close()
    return content

  def query_addr(self, addr):
    if addr == "":
      return None, None, None
    query = "https://api.mapbox.com/geocoding/v5/mapbox.places/" + unquote(addr) + ".json?access_token=" + self.get_public_token() + "&limit=1"  # 查询地址的API请求
    # focus on place around last gps position
    last_pos = Params().get("LastGPSPosition")  # 获取最后的GPS位置
    if last_pos is not None and last_pos != "":
      l = json.loads(last_pos)
      query += "&proximity=%s,%s" % (l["longitude"], l["latitude"])  # 添加位置接近参数

    r = requests.get(query)  # 发送请求
    if r.status_code != 200:
      return None, None, None

    j = json.loads(r.text)  # 解析返回的JSON
    if not j["features"]:
      return None, None, None

    feature = j["features"][0]  # 获取第一个特征
    return feature["place_name"], feature["center"][1], feature["center"][0]  # 返回地点名称和经纬度

  def parse_POST(self):
    ctype, pdict = parse_header(self.headers['content-type'])  # 解析请求头中的内容类型
    if ctype == 'application/x-www-form-urlencoded':
      length = int(self.headers['content-length'])
      postvars = parse_qs(
        self.rfile.read(length).decode('utf-8'),
        keep_blank_values=1)  # 解析表单数据
    elif ctype == 'application/json':
      length = int(self.headers['content-length'])
      post_data = self.rfile.read(length).decode('utf-8')
      try:
        postvars = json.loads(post_data)  # 解析JSON数据
      except json.JSONDecodeError:
        self.send_error(400, 'Invalid JSON data')  # 返回400错误
        return None
    else:
      postvars = {}
    return postvars

  def gcj02towgs84(self, lng, lat):
    dlat = self.transform_lat(lng - 105.0, lat - 35.0)  # 转换纬度
    dlng = self.transform_lng(lng - 105.0, lat - 35.0)  # 转换经度
    radlat = lat / 180.0 * pi
    magic = math.sin(radlat)
    magic = 1 - ee * magic * magic
    sqrtmagic = math.sqrt(magic)
    dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * pi)
    dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * pi)
    mglat = lat + dlat
    mglng = lng + dlng
    return [lng * 2 - mglng, lat * 2 - mglat]  # 返回转换后的经纬度

  def transform_lat(self, lng, lat):
    ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + 0.1 * lng * lat + 0.2 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lat * pi) + 40.0 * math.sin(lat / 3.0 * pi)) * 2.0 / 3.0
    ret += (160.0 * math.sin(lat / 12.0 * pi) + 320 * math.sin(lat * pi / 30.0)) * 2.0 / 3.0
    return ret  # 返回转换后的纬度

  def transform_lng(self, lng, lat):
    ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + 0.1 * lng * lat + 0.1 * math.sqrt(abs(lng))
    ret += (20.0 * math.sin(6.0 * lng * pi) + 20.0 * math.sin(2.0 * lng * pi)) * 2.0 / 3.0
    ret += (20.0 * math.sin(lng * pi) + 40.0 * math.sin(lng / 3.0 * pi)) * 2.0 / 3.0
    ret += (150.0 * math.sin(lng / 12.0 * pi) + 300.0 * math.sin(lng / 30.0 * pi)) * 2.0 / 3.0
    return ret  # 返回转换后的经度

  def to_json(self, lat, lng, type = "recent", name = ""):
    if name == "":
      name =  str(lat) + "," + str(lng)  # 如果没有名称，则使用经纬度作为名称
    new_dest = {"latitude": float(lat), "longitude": float(lng), "place_name": name}  # 创建新的目的地字典

    if type == "recent":
      new_dest["save_type"] = "recent"  # 设置保存类型为最近
    else:
      new_dest["save_type"] = "favorite"  # 设置保存类型为收藏
      new_dest["label"] = type  # 设置标签

    val = params.get("ApiCache_NavDestinations", encoding='utf8')  # 获取导航目的地缓存
    if val is not None:
      val = val.rstrip('\x00')
    dests = [] if val is None else json.loads(val)  # 解析缓存的目的地

    # type idx
    type_label_ids = {"home": None, "work": None, "fav1": None, "fav2": None, "fav3": None, "recent": []}
    idx = 0
    for d in dests:
      if d["save_type"] == "favorite":
        type_label_ids[d["label"]] = idx  # 保存收藏类型的索引
      else:
        type_label_ids["recent"].append(idx)  # 保存最近类型的索引
      idx += 1

    if type == "recent":
      id = None
      if len(type_label_ids["recent"]) > 10:
        dests.pop(type_label_ids["recent"][-1])  # 如果最近的目的地超过10个，则删除最旧的
    else:
      id = type_label_ids[type]

    if id is None:
      dests.insert(0, new_dest)  # 如果没有ID，则插入新的目的地
    else:
      dests[id] = new_dest  # 更新已有目的地

    params.put("ApiCache_NavDestinations", json.dumps(dests).rstrip("\n\r"))  # 存储更新后的目的地缓存

def main():
  try:
    set_core_affinity([0, 1, 2, 3])  # 设置核心亲和性
  except Exception:
    cloudlog.exception("otisserv: failed to set core affinity")  # 记录异常
  webServer = HTTPServer((hostName, serverPort), OtisServ)  # 创建HTTP服务器

  try:
    webServer.serve_forever()  # 启动服务器
  except KeyboardInterrupt:
    pass

  webServer.server_close()  # 关闭服务器

if __name__ == "__main__":
  main()
