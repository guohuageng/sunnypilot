#!/usr/bin/env python3
# 可能需要卸载PyQt5 pip包以避免冲突

import os
import time
import numpy as np
import polyline
from cffi import FFI

from openpilot.common.ffi_wrapper import suffix
from openpilot.common.basedir import BASEDIR

HEIGHT = WIDTH = SIZE = 256  # 图像高度和宽度
METERS_PER_PIXEL = 2  # 每个像素的米数


def get_ffi():
  # 获取FFI实例并加载地图渲染库
  lib = os.path.join(BASEDIR, "selfdrive", "navd", "libmaprender" + suffix())

  ffi = FFI()
  ffi.cdef("""
void* map_renderer_init(char *maps_host, char *token);
void map_renderer_update_position(void *inst, float lat, float lon, float bearing);
void map_renderer_update_route(void *inst, char *polyline);
void map_renderer_update(void *inst);
void map_renderer_process(void *inst);
bool map_renderer_loaded(void *inst);
uint8_t* map_renderer_get_image(void *inst);
void map_renderer_free_image(void *inst, uint8_t *buf);
""")
  return ffi, ffi.dlopen(lib)


def wait_ready(lib, renderer, timeout=None):
  # 等待地图渲染器准备就绪
  st = time.time()
  while not lib.map_renderer_loaded(renderer):
    lib.map_renderer_update(renderer)

    # 主QT应用程序未执行，因此我们需要定期处理事件，例如网络请求
    lib.map_renderer_process(renderer)

    time.sleep(0.01)

    if timeout is not None and time.time() - st > timeout:
      raise TimeoutError("等待地图渲染器准备超时")


def get_image(lib, renderer):
  # 获取渲染的图像
  buf = lib.map_renderer_get_image(renderer)
  r = list(buf[0:WIDTH * HEIGHT])
  lib.map_renderer_free_image(renderer, buf)

  # 转换为numpy数组
  r = np.asarray(r)
  return r.reshape((WIDTH, HEIGHT))


def navRoute_to_polyline(nr):
  # 将导航路线转换为多段线
  coords = [(m.latitude, m.longitude) for m in nr.navRoute.coordinates]
  return coords_to_polyline(coords)


def coords_to_polyline(coords):
  # TODO: 这个10的因子来自哪里？
  return polyline.encode([(lat * 10., lon * 10.) for lat, lon in coords])


def polyline_to_coords(p):
  # 将多段线转换为坐标
  coords = polyline.decode(p)
  return [(lat / 10., lon / 10.) for lat, lon in coords]


if __name__ == "__main__":
  import matplotlib.pyplot as plt

  ffi, lib = get_ffi()
  renderer = lib.map_renderer_init(ffi.NULL, ffi.NULL)
  wait_ready(lib, renderer)

  # 示例几何形状
  geometry = r"{yxk}@|obn~Eg@@eCFqc@J{RFw@?kA@gA?q|@Riu@NuJBgi@ZqVNcRBaPBkG@iSD{I@_H@cH?gG@mG@gG?aD@{LDgDDkVVyQLiGDgX@q_@@qI@qKhS{R~[}NtYaDbGoIvLwNfP_b@|f@oFnF_JxHel@bf@{JlIuxAlpAkNnLmZrWqFhFoh@jd@kX|TkJxH_RnPy^|[uKtHoZ~Um`DlkCorC``CuShQogCtwB_ThQcr@fk@sVrWgRhVmSb\\oj@jxA{Qvg@u]tbAyHzSos@xjBeKbWszAbgEc~@~jCuTrl@cYfo@mRn\\_m@v}@ij@jp@om@lk@y|A`pAiXbVmWzUod@xj@wNlTw}@|uAwSn\\kRfYqOdS_IdJuK`KmKvJoOhLuLbHaMzGwO~GoOzFiSrEsOhD}PhCqw@vJmnAxSczA`Vyb@bHk[fFgl@pJeoDdl@}}@zIyr@hG}X`BmUdBcM^aRR}Oe@iZc@mR_@{FScHxAn_@vz@zCzH~GjPxAhDlB~DhEdJlIbMhFfG|F~GlHrGjNjItLnGvQ~EhLnBfOn@p`@AzAAvn@CfC?fc@`@lUrArStCfSxEtSzGxM|ElFlBrOzJlEbDnC~BfDtCnHjHlLvMdTnZzHpObOf^pKla@~G|a@dErg@rCbj@zArYlj@ttJ~AfZh@r]LzYg@`TkDbj@gIdv@oE|i@kKzhA{CdNsEfOiGlPsEvMiDpLgBpHyB`MkB|MmArPg@|N?|P^rUvFz~AWpOCdAkB|PuB`KeFfHkCfGy@tAqC~AsBPkDs@uAiAcJwMe@s@eKkPMoXQux@EuuCoH?eI?Kas@}Dy@wAUkMOgDL" # noqa: E501
  lib.map_renderer_update_route(renderer, geometry.encode())

  POSITIONS = [
    (32.71569271952601, -117.16384270868463, 0), (32.71569271952601, -117.16384270868463, 45),  # 圣地亚哥
    (52.378641991483136, 4.902623379456488, 0), (52.378641991483136, 4.902623379456488, 45),  # 阿姆斯特丹
  ]
  plt.figure()

  for i, pos in enumerate(POSITIONS):
    t = time.time()
    lib.map_renderer_update_position(renderer, *pos)
    wait_ready(lib, renderer)

    print(f"{pos} 耗时 {time.time() - t:.2f} s")

    plt.subplot(2, 2, i + 1)
    plt.imshow(get_image(lib, renderer), cmap='gray')

  plt.show()
