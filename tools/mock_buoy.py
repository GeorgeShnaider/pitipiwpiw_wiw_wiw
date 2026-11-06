#!/usr/bin/env python3
"""Эмулятор буя ТНПА для отладки WebApp на ПК.

Один порт обслуживает и HTTP, и WebSocket:
  GET /  (и /index.html)  -> data/index.html (перечитывается при каждом запросе)
  WS  /ws                 -> принимает cmd (контракт №1), шлёт телеметрию
                             (контракт №3 v2 с armed ТНПА, 10 Гц) и JPEG-кадры
                             (контракт №4, binary)

Зависимость: websockets >= 13. Pillow не обязателен: без него кадры
строятся встроенным JPEG-кодером (мозаика 8x8, только DC-коэффициенты).
"""

import argparse
import asyncio
import http
import json
import logging
import math
import os
import socket
import sys
import time

try:
    import websockets
    from websockets.asyncio.server import serve
    from websockets.datastructures import Headers
    from websockets.http11 import Response
except ImportError:
    sys.stderr.write("Нужен пакет websockets >= 13:  pip install 'websockets>=13'\n")
    sys.exit(1)

try:
    from PIL import Image, ImageDraw  # type: ignore
    HAVE_PIL = True
except ImportError:
    HAVE_PIL = False

from jpeg_mini import builtin_frame  # общий JPEG-кодер, tools/jpeg_mini.py

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
INDEX_HTML = os.path.join(ROOT, "data", "index.html")

WS_PATH = "/ws"                 # protocol.h WS_PATH
WS_CMD_TYPE = "cmd"             # protocol.h WS_CMD_TYPE
WS_CMD_RATE_HZ = 20             # protocol.h WS_CMD_RATE_HZ
TELEMETRY_RATE_HZ = 10          # protocol.h TELEMETRY_RATE_HZ
TELEMETRY_MAX_LEN = 160         # protocol.h TELEMETRY_MAX_LEN (с '\n')
VIDEO_MAX_FRAME = 32768         # protocol.h VIDEO_MAX_FRAME
BUOY_WS_TIMEOUT_MS = 300        # config.h BUOY_WS_TIMEOUT_MS
# Формат protocol.h TELEMETRY_JSON_FMT (протокол v2, D-010) без завершающего '\n' (буй его отрезает)
TELEMETRY_FMT = ('{"depth":%.2f,"temp":%.1f,"vbat":%.2f,'
                 '"roll":%.1f,"pitch":%.1f,"yaw":%.1f,"armed":%d}')
# Формат протокола v1 (без armed) — для проверки совместимости WebApp, флаг --no-armed
TELEMETRY_FMT_V1 = ('{"depth":%.2f,"temp":%.1f,"vbat":%.2f,'
                    '"roll":%.1f,"pitch":%.1f,"yaw":%.1f}')
ROV_ARMED_DELAY_S = 0.1         # mock: ТНПА отражает armed из cmd с задержкой (пакет + телеметрия)
CMD_KEYS = ["type", "armed", "light", "axes"]
AXIS_KEYS = ["surge", "yaw", "heave", "sway"]


def pil_frame(n, t):
    import io
    im = Image.new("RGB", (320, 240))
    d = ImageDraw.Draw(im)
    for y in range(0, 240, 4):
        d.rectangle([0, y, 319, y + 3], fill=(20, int(60 + y * 0.4), int(110 + y * 0.4)))
    x = int((t * 64) % 320)
    d.rectangle([x, 0, x + 12, 239], fill=(240, 200, 40))
    d.line([160, 0, 160, 239], fill=(230, 230, 230))
    d.line([0, 120, 319, 120], fill=(230, 230, 230))
    d.text((8, 8), "MOCK BUOY  frame %d" % n, fill=(255, 255, 255))
    d.text((8, 22), "t=%.1f s" % t, fill=(255, 255, 255))
    buf = io.BytesIO()
    im.save(buf, "JPEG", quality=60)
    return buf.getvalue()


def load_frames_dir(path):
    names = sorted(f for f in os.listdir(path) if f.lower().endswith((".jpg", ".jpeg")))
    frames = []
    for name in names:
        with open(os.path.join(path, name), "rb") as fh:
            data = fh.read()
        if 0 < len(data) <= VIDEO_MAX_FRAME:
            frames.append(data)
        else:
            print("[video] пропущен %s: %d байт (лимит %d)" % (name, len(data), VIDEO_MAX_FRAME))
    return frames


# =============================================================================
#  Проверка команды по контракту №1
# =============================================================================
def _is_num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def validate_cmd(text):
    """Возвращает (cmd или None, список замечаний)."""
    errs = []
    try:
        obj = json.loads(text, object_pairs_hook=lambda pairs: pairs)
    except ValueError as e:
        return None, ["некорректный JSON: %s" % e]
    if not isinstance(obj, list):
        return None, ["корень не объект"]

    def to_dict(pairs, where):
        keys = [k for k, _ in pairs]
        if len(set(keys)) != len(keys):
            errs.append("%s: повторяющиеся ключи %s" % (where, keys))
        return keys, dict(pairs)

    keys, cmd = to_dict(obj, "cmd")
    extra = [k for k in keys if k not in CMD_KEYS]
    missing = [k for k in CMD_KEYS if k not in keys]
    if extra:
        errs.append("лишние ключи: %s" % extra)
    if missing:
        errs.append("нет ключей: %s" % missing)
    if not extra and not missing and keys != CMD_KEYS:
        errs.append("порядок ключей %s, в контракте %s" % (keys, CMD_KEYS))
    if "type" in cmd and cmd["type"] != WS_CMD_TYPE:
        errs.append("type=%r, ожидается %r" % (cmd["type"], WS_CMD_TYPE))
    for k in ("armed", "light"):
        if k in cmd and not isinstance(cmd[k], bool):
            errs.append("%s должен быть bool, а не %s" % (k, type(cmd[k]).__name__))
    axes = {}
    if "axes" in cmd:
        if not isinstance(cmd["axes"], list):
            errs.append("axes должен быть объектом")
        else:
            akeys, axes = to_dict(cmd["axes"], "axes")
            aextra = [k for k in akeys if k not in AXIS_KEYS]
            amissing = [k for k in AXIS_KEYS if k not in akeys]
            if aextra:
                errs.append("лишние оси: %s" % aextra)
            if amissing:
                errs.append("нет осей: %s" % amissing)
            if not aextra and not amissing and akeys != AXIS_KEYS:
                errs.append("порядок осей %s, в контракте %s" % (akeys, AXIS_KEYS))
            for k in AXIS_KEYS:
                if k not in axes:
                    continue
                v = axes[k]
                if not _is_num(v):
                    errs.append("ось %s не число: %r" % (k, v))
                elif not math.isfinite(v) or v < -1.0 or v > 1.0:
                    errs.append("ось %s вне -1..+1: %r" % (k, v))
    cmd = dict(cmd)
    cmd["axes"] = axes
    return cmd, errs


# =============================================================================
#  Упрощённая модель фактического armed ТНПА (ключ armed телеметрии, D-010)
# =============================================================================
class RovModel:
    """armed ТНПА = armed последней cmd с задержкой ROV_ARMED_DELAY_S.

    Нет cmd дольше BUOY_WS_TIMEOUT_MS — буй шлёт вниз armed=0, ТНПА не взведён.
    --rov-disarm-after N: через N с после взвода ТНПА перестаёт видеть команды
    (обрыв линии вниз) и сообщает armed:0, пока cmd не сбросит armed.
    """

    def __init__(self, disarm_after):
        self.disarm_after = disarm_after
        self.hist = []          # (t, armed, момент фронта) по принятым cmd
        self.rise_t = None      # момент фронта armed 0->1 в cmd
        self.cmd_armed = False

    def on_cmd(self, armed, now):
        if armed and not self.cmd_armed:
            self.rise_t = now
        if not armed:
            self.rise_t = None
        self.cmd_armed = armed
        self.hist.append((now, armed, self.rise_t))

    def armed(self, now):
        t_ref = now - ROV_ARMED_DELAY_S
        while len(self.hist) > 1 and self.hist[1][0] <= t_ref:
            self.hist.pop(0)
        if not self.hist or self.hist[0][0] > t_ref:
            return 0
        t_cmd, a, rise_t = self.hist[0]
        if (t_ref - t_cmd) * 1000.0 > BUOY_WS_TIMEOUT_MS:
            return 0
        if a and self.disarm_after > 0 and rise_t is not None and t_ref - rise_t >= self.disarm_after:
            return 0
        return 1 if a else 0


# =============================================================================
#  Сервер
# =============================================================================
class Mock:
    def __init__(self, args):
        self.args = args
        self.t0 = time.monotonic()
        self.frames_dir = load_frames_dir(args.frames) if args.frames else []
        if args.frames and not self.frames_dir:
            print("[video] в %s нет подходящих JPEG, используется генератор" % args.frames)
        self.frame_no = 0
        self.clients = 0

    def silent(self):
        """Режим --drop: последние drop_len секунд каждого периода буй молчит."""
        if not self.args.drop:
            return False
        ph = (time.monotonic() - self.t0) % self.args.drop_period
        return ph >= self.args.drop_period - self.args.drop_len

    def telemetry(self, rov_armed):
        t = time.monotonic() - self.t0
        depth = max(0.0, 2.5 + 2.4 * math.sin(t * 0.15))
        temp = 18.0 + 1.5 * math.sin(t * 0.05)
        # Батарея медленно разряжается 16.6 -> 12.8 В и снова "заряжается": видно все цвета
        vbat = 16.6 - 3.8 * (0.5 - 0.5 * math.cos(t * 2 * math.pi / 90.0))
        if self.args.no_vbat:
            vbat = 0.0   # ICD §6: 0 — напряжение не измеряется (делителя нет, D-009)
        roll = 25.0 * math.sin(t * 0.7)
        pitch = 15.0 * math.sin(t * 0.43 + 1.0)
        yaw = (t * 12.0) % 360.0
        if self.args.no_armed:
            s = TELEMETRY_FMT_V1 % (depth, temp, vbat, roll, pitch, yaw)
        else:
            s = TELEMETRY_FMT % (depth, temp, vbat, roll, pitch, yaw, rov_armed)
        assert len(s) + 1 <= TELEMETRY_MAX_LEN
        return s

    def next_frame(self):
        self.frame_no += 1
        if self.frames_dir:
            return self.frames_dir[self.frame_no % len(self.frames_dir)]
        t = time.monotonic() - self.t0
        if HAVE_PIL and not self.args.no_pil:
            return pil_frame(self.frame_no, t)
        return builtin_frame(self.frame_no, t)

    # --- HTTP на том же порту --------------------------------------------------
    def process_request(self, connection, request):
        path = request.path.split("?", 1)[0]
        if path == WS_PATH:
            return None   # продолжаем WebSocket-рукопожатие
        if path in ("/", "/index.html"):
            try:
                with open(INDEX_HTML, "rb") as fh:
                    body = fh.read()
            except OSError as e:
                return connection.respond(http.HTTPStatus.INTERNAL_SERVER_ERROR, "нет index.html: %s\n" % e)
            headers = Headers([
                ("Content-Type", "text/html; charset=utf-8"),
                ("Content-Length", str(len(body))),
                ("Cache-Control", "no-store"),
                ("Connection", "close"),
            ])
            return Response(200, "OK", headers, body)
        return connection.respond(http.HTTPStatus.NOT_FOUND, "404\n")

    # --- WebSocket ---------------------------------------------------------------
    async def handler(self, ws):
        path = ws.request.path.split("?", 1)[0]
        if path != WS_PATH:
            await ws.close(1008, "bad path")
            return
        self.clients += 1
        peer = ws.remote_address
        print("[ws] подключён %s (клиентов: %d)" % (peer, self.clients))
        ws.stat = {"n": 0, "bad": 0, "dtmin": None, "dtmax": 0.0, "last": None,
                   "last_t": None, "armed": False, "light": False, "gaps": 0}
        ws.rov = RovModel(self.args.rov_disarm_after)
        tasks = [asyncio.ensure_future(self.telemetry_loop(ws)),
                 asyncio.ensure_future(self.video_loop(ws)),
                 asyncio.ensure_future(self.stats_loop(ws))]
        try:
            async for msg in ws:
                if isinstance(msg, bytes):
                    print("[cmd] WARN: бинарное сообщение %d байт (ожидается text)" % len(msg))
                    ws.stat["bad"] += 1
                    continue
                self.on_cmd(ws, msg)
        except websockets.ConnectionClosed:
            pass
        finally:
            for t in tasks:
                t.cancel()
            self.clients -= 1
            print("[ws] отключён %s (клиентов: %d)" % (peer, self.clients))

    def on_cmd(self, ws, msg):
        st = ws.stat
        now = time.monotonic()
        if st["last_t"] is not None:
            dt = now - st["last_t"]
            st["dtmin"] = dt if st["dtmin"] is None else min(st["dtmin"], dt)
            st["dtmax"] = max(st["dtmax"], dt)
            if dt * 1000.0 > BUOY_WS_TIMEOUT_MS:
                st["gaps"] += 1
                print("[cmd] WARN: пауза %.0f мс > %d мс (буй ушёл бы в fail-safe)" % (dt * 1000, BUOY_WS_TIMEOUT_MS))
        st["last_t"] = now
        cmd, errs = validate_cmd(msg)
        st["n"] += 1
        if errs:
            st["bad"] += 1
            print("[cmd] WARN: %s | %s" % ("; ".join(errs), msg[:200]))
            return
        ws.rov.on_cmd(cmd["armed"], now)
        if cmd["armed"] != st["armed"]:
            print("[cmd] armed: %s -> %s" % (st["armed"], cmd["armed"]))
            st["armed"] = cmd["armed"]
        if cmd["light"] != st["light"]:
            print("[cmd] light: %s -> %s" % (st["light"], cmd["light"]))
            st["light"] = cmd["light"]
        st["last"] = cmd
        if self.args.verbose:
            print("[cmd] %s" % msg)

    async def stats_loop(self, ws):
        while True:
            await asyncio.sleep(1.0)
            st = ws.stat
            a = st["last"]["axes"] if st["last"] else {k: 0 for k in AXIS_KEYS}
            rate = st["n"]
            mark = "OK" if abs(rate - WS_CMD_RATE_HZ) <= 1 else "WARN"
            dtmin = (st["dtmin"] or 0.0) * 1000
            print("[cmd] %2d Гц %-4s ошибок %d | dt %3.0f..%3.0f мс | armed=%d light=%d | "
                  "surge=%+.2f yaw=%+.2f heave=%+.2f sway=%+.2f%s"
                  % (rate, mark, st["bad"], dtmin, st["dtmax"] * 1000,
                     st["armed"], st["light"], a["surge"], a["yaw"], a["heave"], a["sway"],
                     "  [DROP: буй молчит]" if self.silent() else ""))
            st["n"] = 0
            st["bad"] = 0
            st["dtmin"] = None
            st["dtmax"] = 0.0

    async def telemetry_loop(self, ws):
        period = 1.0 / TELEMETRY_RATE_HZ
        nxt = time.monotonic()
        shown = 0
        while True:
            nxt += period
            rov_armed = ws.rov.armed(time.monotonic())
            if rov_armed != shown and not self.args.no_armed:
                print("[rov] armed ТНПА: %d -> %d" % (shown, rov_armed))
                shown = rov_armed
            if not self.silent():
                await ws.send(self.telemetry(rov_armed))
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))

    async def video_loop(self, ws):
        if self.args.fps == 0:
            return   # --fps 0: камеры нет, WebApp показывает заглушку «нет видео»
        period = 1.0 / self.args.fps
        nxt = time.monotonic()
        while True:
            nxt += period
            if not self.silent():
                frame = self.next_frame()
                # Как буй: если клиент не успевает — кадр пропускается
                if ws.transport is not None and ws.transport.get_write_buffer_size() < 2 * VIDEO_MAX_FRAME:
                    await ws.send(frame)
            await asyncio.sleep(max(0.0, nxt - time.monotonic()))


def local_ips():
    ips = set()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        ips.add(s.getsockname()[0])
        s.close()
    except OSError:
        pass
    try:
        for info in socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET):
            ips.add(info[4][0])
    except OSError:
        pass
    ips.discard("127.0.0.1")
    return sorted(ips)


async def main_async(args):
    mock = Mock(args)
    src = ("папка %s" % args.frames if mock.frames_dir else
           ("Pillow" if HAVE_PIL and not args.no_pil else "встроенный JPEG-кодер"))
    async with serve(mock.handler, args.host, args.port,
                     process_request=mock.process_request,
                     max_size=4096, compression=None):
        print("Mock буя слушает %s:%d  (видео: %s, %.1f к/с%s%s%s)"
              % (args.host, args.port, src, args.fps,
                 ", DROP %.0f из %.0f с" % (args.drop_len, args.drop_period) if args.drop else "",
                 ", vbat=0 (не измеряется)" if args.no_vbat else "",
                 ", телеметрия v1 без armed" if args.no_armed else
                 (", ТНПА теряет команды через %.1f с после взвода" % args.rov_disarm_after
                  if args.rov_disarm_after > 0 else "")))
        print("  на этом ПК:   http://127.0.0.1:%d/" % args.port)
        for ip in local_ips():
            print("  с телефона:   http://%s:%d/" % (ip, args.port))
        if args.duration > 0:
            await asyncio.sleep(args.duration)
        else:
            await asyncio.Future()


def main():
    ap = argparse.ArgumentParser(description="Эмулятор буя ТНПА (HTTP + WebSocket /ws на одном порту)")
    ap.add_argument("--host", default="0.0.0.0", help="адрес прослушивания (по умолчанию 0.0.0.0)")
    ap.add_argument("--port", type=int, default=8080, help="порт HTTP и WS (по умолчанию 8080)")
    ap.add_argument("--fps", type=float, default=10.0, help="частота видеокадров (по умолчанию 10; 0 — без видео)")
    ap.add_argument("--frames", help="папка с JPEG-кадрами (по кругу) вместо генератора")
    ap.add_argument("--no-vbat", action="store_true", help="слать vbat 0.00 («не измеряется», ICD §6)")
    ap.add_argument("--no-pil", action="store_true", help="не использовать Pillow даже если он есть")
    ap.add_argument("--drop", action="store_true", help="периодически замолкать (проверка «НЕТ СВЯЗИ»)")
    ap.add_argument("--drop-period", type=float, default=10.0, help="период цикла --drop, с (по умолчанию 10)")
    ap.add_argument("--drop-len", type=float, default=3.0, help="длительность молчания в цикле, с (по умолчанию 3)")
    ap.add_argument("--no-armed", action="store_true",
                    help="телеметрия протокола v1 без ключа armed (проверка совместимости WebApp)")
    ap.add_argument("--rov-disarm-after", type=float, default=0.0, metavar="N",
                    help="через N с после взвода ТНПА сообщает armed:0 (обрыв линии вниз); 0 — выкл.")
    ap.add_argument("--duration", type=float, default=0.0, help="завершиться через N секунд (0 — работать бесконечно)")
    ap.add_argument("-v", "--verbose", action="store_true", help="печатать каждую принятую команду")
    args = ap.parse_args()
    try:
        sys.stdout.reconfigure(line_buffering=True)   # лог виден сразу и при перенаправлении в файл
    except AttributeError:
        pass
    if args.fps < 0 or args.drop_len <= 0 or args.drop_len >= args.drop_period or args.rov_disarm_after < 0:
        ap.error("нужны fps ≥ 0, 0 < drop-len < drop-period, rov-disarm-after ≥ 0")
    # Обрывы до HTTP-запроса (предварительные соединения браузера) — не ошибка, не шумим
    logging.getLogger("websockets.server").setLevel(logging.CRITICAL)
    try:
        asyncio.run(main_async(args))
    except KeyboardInterrupt:
        print("\nОстановлено")


if __name__ == "__main__":
    main()
