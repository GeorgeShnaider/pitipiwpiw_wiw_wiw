#!/usr/bin/env python3
"""bench_link.py — стенд «ПК вместо соседнего узла» для проверки плат по одной.

Режимы:
  emulate-buoy  ПК вместо буя: шлёт ControlPacket (контракт №2) 20 Гц по
                сценарию, принимает и проверяет телеметрию rov_core (контракт №3).
  emulate-rov   ПК вместо ТНПА: принимает ControlPacket от буя, проверяет CRC,
                диапазоны и интервалы, шлёт синтетическую телеметрию 10 Гц.
  send-video    ПК вместо камеры: шлёт кадры контракта №4 в видеовход буя.

Зависимость: pyserial (pip install pyserial). Протокол — docs/ICD.md §5–§8.
Подробности и критерии «прошло» — tools/README.md, раздел «Стенд (этап 7)».
"""

import argparse
import binascii
import datetime
import json
import math
import os
import re
import struct
import sys
import threading
import time

# =============================================================================
#  Константы протокола и узлов. Источник: include/protocol.h и include/config.h
#  (сверено с protocol.h v1 и config.h FW 0.1.0). При изменении там — поправить здесь.
# =============================================================================
PROTOCOL_VERSION = 2            # protocol.h PROTOCOL_VERSION (v2: armed в телеметрии, D-010)
LINK_BAUD = 115200              # protocol.h LINK_BAUD (пары 2+3)
VIDEO_BAUD = 921600             # protocol.h VIDEO_BAUD (пара 4)

CTRL_HEADER1 = 0xAA             # protocol.h CTRL_HEADER1
CTRL_HEADER2 = 0x55             # protocol.h CTRL_HEADER2
CTRL_FLAG_ARMED = 0x01          # protocol.h CTRL_FLAG_ARMED
CTRL_FLAG_LIGHT = 0x02          # protocol.h CTRL_FLAG_LIGHT
CTRL_AXIS_MAX = 100             # protocol.h CTRL_AXIS_MAX
CTRL_RATE_HZ = 20               # protocol.h CTRL_RATE_HZ
CTRL_FMT = "<BBbbbbB"           # struct ControlPacket без crc: h1 h2 surge yaw heave sway flags
CTRL_LEN = 8                    # sizeof(ControlPacket)

TELEMETRY_RATE_HZ = 10          # protocol.h TELEMETRY_RATE_HZ
TELEMETRY_MAX_LEN = 160         # protocol.h TELEMETRY_MAX_LEN (с '\n')
# protocol.h TELEMETRY_JSON_FMT (v2): armed — фактическое состояние ROV_CORE, int 0/1
TELEMETRY_FMT = ('{"depth":%.2f,"temp":%.1f,"vbat":%.2f,'
                 '"roll":%.1f,"pitch":%.1f,"yaw":%.1f,"armed":%d}\n')
TELEMETRY_KEYS = ["depth", "temp", "vbat", "roll", "pitch", "yaw", "armed"]
# Строгая форма строки по TELEMETRY_JSON_FMT (число знаков после точки)
_NUM2 = r"(-?\d+\.\d{2})"
_NUM1 = r"(-?\d+\.\d)"
TELEMETRY_RE = re.compile(r'^\{"depth":%s,"temp":%s,"vbat":%s,"roll":%s,"pitch":%s,"yaw":%s,"armed":([01])\}$'
                          % (_NUM2, _NUM1, _NUM2, _NUM1, _NUM1, _NUM1))

VIDEO_MAGIC1 = 0xA5             # protocol.h VIDEO_MAGIC1
VIDEO_MAGIC2 = 0x5A             # protocol.h VIDEO_MAGIC2
VIDEO_MAX_FRAME = 32768         # protocol.h VIDEO_MAX_FRAME
VIDEO_HDR_FMT = "<BBHIB"        # VideoFrameHeader без hcrc: magic1 magic2 seq length flags
CRC16_INIT = 0xFFFF             # protocol.h CRC16_INIT (CRC16-CCITT, полином 0x1021)

FAILSAFE_TIMEOUT_MS = 500       # config.h FAILSAFE_TIMEOUT_MS (rov_core)
BUOY_WS_TIMEOUT_MS = 300        # config.h BUOY_WS_TIMEOUT_MS (буй)
ESC_ARM_TIME_MS = 3000          # config.h ESC_ARM_TIME_MS (окно старта rov_core)
ESC_US_MIN = 1100               # config.h ESC_US_MIN (полный реверс = PPM Min Throttle)
ESC_US_NEUTRAL = 1500           # config.h ESC_US_NEUTRAL (стоп = PPM Center Throttle)
ESC_US_MAX = 1900               # config.h ESC_US_MAX (полный вперёд = PPM Max Throttle)
ESC_DEADBAND = 0.05             # config.h ESC_DEADBAND
THRUST_LIMIT = 0.50             # config.h THRUST_LIMIT
MOTOR_DIR = [+1, +1, +1, +1]    # config.h MOTOR_DIR (активные M0..M3)
# config.h MIX, активные моторы M0..M3, оси {surge, yaw, heave, sway}
MIX = [
    [1.0, 1.0, 0.0, 0.0],       # M0 M_STERN_LEFT
    [1.0, -1.0, 0.0, 0.0],      # M1 M_STERN_RIGHT
    [0.0, 0.0, 1.0, 0.0],       # M2 M_BOW_VERT
    [0.0, 0.0, 0.0, 1.0],       # M3 M_BOW_HORIZ
]
CAM_TARGET_FPS = 10             # config.h CAM_TARGET_FPS

ROV_LINK_RX_PIN = 16            # config.h ROV_LINK_RX_PIN (управление сверху)
ROV_LINK_TX_PIN = 17            # config.h ROV_LINK_TX_PIN (телеметрия наверх)
BUOY_LINK_RX_PIN = 16           # config.h BUOY_LINK_RX_PIN (телеметрия снизу)
BUOY_LINK_TX_PIN = 17           # config.h BUOY_LINK_TX_PIN (управление вниз)
BUOY_VIDEO_RX_PIN = 18          # config.h BUOY_VIDEO_RX_PIN (видео от камеры)
LIGHT_PIN = 4                   # config.h LIGHT_PIN (LED-лента rov_core)

AXIS_NAMES = ["surge", "yaw", "heave", "sway"]

# Параметры самой утилиты (не протокол)
CTRL_PERIOD_S = 1.0 / CTRL_RATE_HZ
TELEM_PERIOD_S = 1.0 / TELEMETRY_RATE_HZ
ARM_WINDOW_PAD_S = ESC_ARM_TIME_MS / 1000.0 + 0.5   # 3.5 с armed=0 до первого взвода
BOOT_GAP_S = 0.4                # пауза телеметрии, после которой новая строка = перезагрузка rov_core
CTRL_DT_LIMIT_MS = 60           # приёмка буя: максимальный интервал между пакетами
VIDEO_CHUNK = 1024              # порция записи файла в режиме send-video --file


# =============================================================================
#  Общие функции
# =============================================================================
def xor8(data):
    """XOR-контрольная сумма, как calculateCRC() в protocol.h."""
    x = 0
    for b in data:
        x ^= b
    return x


def crc16_ccitt(data, crc=CRC16_INIT):
    """CRC16-CCITT (0x1021, init 0xFFFF), как crc16Update() в protocol.h."""
    return binascii.crc_hqx(data, crc)


def axis_to_i8(v):
    """ICD §5: round(axis * 100) (округление от нуля, как roundf), ограничение ±100."""
    x = v * CTRL_AXIS_MAX
    r = int(math.floor(abs(x) + 0.5))
    r = r if x >= 0 else -r
    return max(-CTRL_AXIS_MAX, min(CTRL_AXIS_MAX, r))


def build_packet(surge, yaw, heave, sway, flags, bad_crc=False):
    """ControlPacket побайтно как controlPacketSeal(): оси — целые int8 (-128..127)."""
    body = struct.pack(CTRL_FMT, CTRL_HEADER1, CTRL_HEADER2, surge, yaw, heave, sway, flags)
    crc = xor8(body)
    if bad_crc:
        crc ^= 0xFF
        if crc == CTRL_HEADER1:   # испорченный CRC не должен выглядеть как начало заголовка
            crc ^= 0xF0
    return body + bytes([crc])


def command_to_us(u):
    """Как Mixer::commandToUs() (src/rov_core/mixer.cpp): мёртвая зона, 1500 + полуход·u, пределы ESC."""
    u = max(-1.0, min(1.0, u))
    if abs(u) < ESC_DEADBAND:
        return ESC_US_NEUTRAL
    half_span = (ESC_US_MAX - ESC_US_MIN) * 0.5
    r = int(ESC_US_NEUTRAL + half_span * u + 0.5)   # значение всегда положительное
    return max(ESC_US_MIN, min(ESC_US_MAX, r))


def expected_us(axes):
    """Ожидаемые мкс M0..M3 в установившемся режиме (MIX -> норм. -> DIR -> LIMIT -> deadband)."""
    axes = [max(-1.0, min(1.0, a)) for a in axes]
    m = [sum(MIX[i][j] * axes[j] for j in range(4)) for i in range(4)]
    peak = max(abs(x) for x in m)
    norm = 1.0 / peak if peak > 1.0 else 1.0
    return [command_to_us(max(-1.0, min(1.0, m[i] * norm)) * MOTOR_DIR[i] * THRUST_LIMIT) for i in range(4)]


def stamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]


def open_port(port, baud, timeout=0.05):
    try:
        import serial  # type: ignore
    except ImportError:
        sys.stderr.write("Нужен пакет pyserial:  pip install pyserial\n")
        sys.exit(2)
    try:
        return serial.Serial(port, baud, timeout=timeout, write_timeout=2.0)
    except (serial.SerialException, OSError, ValueError) as e:
        sys.stderr.write("Не удалось открыть порт %s на %d бод: %s\n" % (port, baud, e))
        sys.exit(2)


class Out:
    """Печать из нескольких потоков без перемешивания строк."""
    lock = threading.Lock()
    t0 = time.monotonic()

    @classmethod
    def line(cls, text):
        with cls.lock:
            sys.stdout.write("[%7.2f с] %s\n" % (time.monotonic() - cls.t0, text))
            sys.stdout.flush()


class RateWindow:
    """События за окно статистики: число и средняя частота по интервалам внутри окна.

    Средняя частота не зависит от того, как граница окна легла относительно
    пакетов (простое «штук за секунду» прыгает на ±1).
    """

    def __init__(self):
        self.n = 0
        self.first = None
        self.last = None

    def add(self, t, k=1):
        if self.first is None:
            self.first = t
        self.last = t
        self.n += k

    def take(self, dt, nominal):
        """Возвращает (текст, ok) и обнуляет окно."""
        n, first, last = self.n, self.first, self.last
        self.n, self.first, self.last = 0, None, None
        if n >= 2 and last > first:
            mean = (n - 1) / (last - first)
            ok = abs(mean - nominal) <= 1.0 and n >= nominal * dt - 1.5
            return "%4.1f Гц (%d за %.2f с) %s" % (mean, n, dt, "OK" if ok else "WARN"), ok
        return "%4.1f Гц (%d за %.2f с) WARN" % (n / dt, n, dt), False


def write_or_stop(port, data, stop):
    """Запись в порт; при ошибке (адаптер выдернули, запись зависла) — сообщение и останов."""
    try:
        port.write(data)
        return True
    except Exception as e:  # noqa: BLE001 — любая ошибка порта завершает режим
        if not stop.is_set():
            Out.line("ОШИБКА записи в порт: %s — выход" % e)
        stop.set()
        return False


# =============================================================================
#  Разбор ControlPacket скользящим окном (повторяет src/rov_core/packet_parser.cpp)
# =============================================================================
class ControlParser:
    def __init__(self):
        self.buf = bytearray()
        self.valid = 0
        self.crc_err = 0
        self.range_err = 0
        self.junk = 0

    def _resync(self):
        n = len(self.buf)
        start = 1
        while start < n:
            if self.buf[start] == CTRL_HEADER1 and (start + 1 >= n or self.buf[start + 1] == CTRL_HEADER2):
                break
            start += 1
        self.junk += start
        del self.buf[:start]

    def feed_byte(self, b):
        """Возвращает dict пакета или None."""
        self.buf.append(b)
        n = len(self.buf)
        if n == 1 and b != CTRL_HEADER1:
            self.buf.clear()
            self.junk += 1
            return None
        if n == 2 and self.buf[1] != CTRL_HEADER2:
            self._resync()
            return None
        if n < CTRL_LEN:
            return None
        raw = bytes(self.buf)
        _, _, s, y, h, w, flags = struct.unpack(CTRL_FMT, raw[:7])
        if xor8(raw[:7]) != raw[7]:
            self.crc_err += 1
            self._resync()
            return None
        if any(a > CTRL_AXIS_MAX or a < -CTRL_AXIS_MAX for a in (s, y, h, w)):
            self.range_err += 1
            self._resync()
            return None
        self.buf.clear()
        self.valid += 1
        return {"raw": raw, "axes": (s, y, h, w), "flags": flags,
                "armed": 1 if flags & CTRL_FLAG_ARMED else 0,
                "light": 1 if flags & CTRL_FLAG_LIGHT else 0,
                "extra_flags": flags & ~(CTRL_FLAG_ARMED | CTRL_FLAG_LIGHT) & 0xFF}


# =============================================================================
#  Разбор и проверка телеметрии (контракт №3)
# =============================================================================
class TelemetryReader:
    def __init__(self):
        self.buf = bytearray()
        self.skip = False          # после переполнения пропускаем до '\n'
        self.ok = 0
        self.bad = 0
        self.out_of_range = 0
        self.last = None
        self.last_error = ""

    def feed(self, data):
        """Возвращает количество целых валидных строк в порции."""
        good = 0
        for b in data:
            if b == 0x0A:
                if self.skip:
                    self.skip = False
                    self.buf.clear()
                    continue
                if self._check(bytes(self.buf)):
                    good += 1
                self.buf.clear()
                continue
            if self.skip:
                continue
            self.buf.append(b)
            if len(self.buf) + 1 > TELEMETRY_MAX_LEN:
                self._bad("строка длиннее %d Б без '\\n'" % TELEMETRY_MAX_LEN, bytes(self.buf[:40]))
                self.buf.clear()
                self.skip = True
        return good

    def _bad(self, why, raw):
        self.bad += 1
        self.last_error = "%s: %r" % (why, raw[:80])

    def _check(self, raw):
        try:
            text = raw.decode("ascii")
        except UnicodeDecodeError:
            self._bad("не ASCII", raw)
            return False
        try:
            obj = json.loads(text)
        except ValueError:
            self._bad("не JSON", raw)
            return False
        if isinstance(obj, dict) and list(obj.keys()) == TELEMETRY_KEYS[:-1]:
            self._bad("нет ключа armed — прошивка rov_core протокола v1, нужна v%d" % PROTOCOL_VERSION, raw)
            return False
        if not isinstance(obj, dict) or list(obj.keys()) != TELEMETRY_KEYS:
            self._bad("ключи не %s" % ",".join(TELEMETRY_KEYS), raw)
            return False
        for k in TELEMETRY_KEYS:
            v = obj[k]
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v):
                self._bad("%s не число" % k, raw)
                return False
        if obj["armed"] not in (0, 1) or isinstance(obj["armed"], float):
            self._bad("armed не 0/1", raw)
            return False
        if not TELEMETRY_RE.match(text):
            self._bad("формат не TELEMETRY_JSON_FMT", raw)
            return False
        self.ok += 1
        self.last = obj
        if not (-180.0 <= obj["roll"] <= 180.0 and -90.0 <= obj["pitch"] <= 90.0
                and 0.0 <= obj["yaw"] <= 360.0 and obj["vbat"] >= 0.0):
            self.out_of_range += 1
        return True


def synth_telemetry(t, no_vbat, armed):
    """Синтетическая телеметрия как в mock_buoy.py (строка с '\\n'), armed — 0/1."""
    depth = max(0.0, 2.5 + 2.4 * math.sin(t * 0.15))
    temp = 18.0 + 1.5 * math.sin(t * 0.05)
    vbat = 0.0 if no_vbat else 16.6 - 3.8 * (0.5 - 0.5 * math.cos(t * 2 * math.pi / 90.0))
    roll = 25.0 * math.sin(t * 0.7)
    pitch = 15.0 * math.sin(t * 0.43 + 1.0)
    yaw = (t * 12.0) % 360.0
    s = TELEMETRY_FMT % (depth, temp, vbat, roll, pitch, yaw, armed)
    assert len(s) <= TELEMETRY_MAX_LEN
    return s.encode("ascii")


# =============================================================================
#  Сценарии emulate-buoy
# =============================================================================
class Phase:
    """Отрезок сценария. dur=None — до Ctrl+C/--duration. mode: send | silent | badcrc."""

    def __init__(self, dur, armed, axes=(0.0, 0.0, 0.0, 0.0), light=0, mode="send",
                 expect="", light_toggle_s=0.0):
        self.dur = dur
        self.armed = armed
        self.axes = axes
        self.light = light
        self.mode = mode
        self.expect = expect
        self.light_toggle_s = light_toggle_s

    def describe(self):
        if self.mode == "silent":
            return "ТИШИНА (пакеты не отправляются)"
        ax = " ".join("%s=%+.2f" % (n, v) for n, v in zip(AXIS_NAMES, self.axes))
        s = "armed=%d light=%s %s" % (self.armed, "перекл." if self.light_toggle_s else self.light, ax)
        if self.mode == "badcrc":
            s += " | 50% битый CRC, 25% ось=101, 25% валидные"
        return s


def _us_text(axes):
    # ROV_CORE получает оси квантованными в int8 (ICD §5) и делит на CTRL_AXIS_MAX
    us = expected_us([axis_to_i8(v) / CTRL_AXIS_MAX for v in axes])
    return "M0=%d M1=%d M2=%d M3=%d" % tuple(us)


def _pre_window():
    return Phase(ARM_WINDOW_PAD_S, 0,
                 expect="armed=0, все M = %d мкс; если плату только что сбросили — причина "
                        "«старт (инициализация ESC)», затем «команда armed=0»" % ESC_US_NEUTRAL)


def build_scenario(name):
    if name == "idle":
        return [Phase(None, 0, expect="пакетов/с=20, ошибок 0, armed=0, причина «команда armed=0», "
                                      "все M = %d мкс, свет=0" % ESC_US_NEUTRAL)]
    if name == "light":
        return [Phase(None, 0, light_toggle_s=2.0,
                      expect="armed=0, все M = %d мкс; «свет=» в логе и GPIO%d (LED-лента) "
                             "меняются раз в 2 с" % (ESC_US_NEUTRAL, LIGHT_PIN))]
    if name == "arm-sweep":
        ph = [_pre_window()]
        first = True
        for i, axis in enumerate(AXIS_NAMES):
            for val, dur in ((0.5, 2.0), (-0.5, 2.0), (0.0, 1.0)):
                axes = tuple(val if j == i else 0.0 for j in range(4))
                exp = "%s %+.1f: %s (с плавным нарастанием slew)" % (axis, val, _us_text(axes))
                if first:
                    exp = "«[rov_core] ARMED»; " + exp
                    first = False
                ph.append(Phase(dur, 1, axes, expect=exp))
        ph.append(Phase(1.0, 0, expect="«[rov_core] DISARM: команда armed=0», все M = %d мкс"
                                       % ESC_US_NEUTRAL))
        return ph
    if name == "failsafe":
        a = (0.3, 0.0, 0.0, 0.0)
        return [
            _pre_window(),
            Phase(3.0, 1, a, expect="«[rov_core] ARMED», %s" % _us_text(a)),
            Phase(2.0, 1, a, mode="silent",
                  expect="через %d мс «[rov_core] DISARM: нет связи», все M = %d мкс, "
                         "пакетов/с=0, LED мигает медленно; здесь — «[телем] armed ТНПА 1 -> 0»"
                         % (FAILSAFE_TIMEOUT_MS, ESC_US_NEUTRAL)),
            Phase(3.0, 1, a, expect="связь вернулась сразу с armed=1: «[rov_core] ARMED» снова "
                                    "(таймаут сбросил детектор фронта), тяга нарастает с нуля: %s"
                                    % _us_text(a)),
            Phase(1.0, 0, expect="«[rov_core] DISARM: команда armed=0»"),
        ]
    if name == "edge":
        return [
            Phase(5.0, 1, expect="armed=1 с самого старта: «ARMED» НЕ появляется, "
                                 "после окна %d мс причина «ждём фронт armed 0->1», все M = %d мкс, "
                                 "в телеметрии ТНПА armed=0"
                                 % (ESC_ARM_TIME_MS, ESC_US_NEUTRAL)),
            Phase(1.0, 0, expect="armed=0: причина «команда armed=0», ARMED нет"),
            Phase(3.0, 1, expect="фронт 0->1: «[rov_core] ARMED» (оси 0, все M = %d мкс)"
                                 % ESC_US_NEUTRAL),
            Phase(1.0, 0, expect="«[rov_core] DISARM: команда armed=0»"),
        ]
    if name == "badcrc":
        return [
            _pre_window(),
            Phase(1.0, 1, expect="«[rov_core] ARMED»"),
            Phase(6.0, 1, mode="badcrc",
                  expect="пакетов/с≈5, «ошибок CRC» растёт на ≈10/с, «вне диапазона» на ≈5/с; "
                         "armed=1 держится, DISARM НЕТ (валидные идут каждые 200 мс < %d мс)"
                         % FAILSAFE_TIMEOUT_MS),
            Phase(1.0, 1, expect="пакетов/с=20, счётчики ошибок больше не растут, armed=1"),
            Phase(1.0, 0, expect="«[rov_core] DISARM: команда armed=0»"),
        ]
    raise ValueError(name)


SCENARIOS = ["idle", "arm-sweep", "failsafe", "edge", "badcrc", "light"]


# =============================================================================
#  Режим emulate-buoy
# =============================================================================
class BuoyEmulator:
    def __init__(self, args):
        self.args = args
        self.port = open_port(args.port, args.baud)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.telem = TelemetryReader()
        self.rx_bytes = 0
        self.lines_win = RateWindow()
        self.last_rx = time.monotonic()
        self.boot_seen = threading.Event()
        self.rov_armed = None         # armed ТНПА из последней валидной телеметрии
        self.sent = 0
        self.sent_win = 0
        self.sent_bad = 0

    # --- приём телеметрии в отдельном потоке ---------------------------------
    def rx_loop(self):
        while not self.stop.is_set():
            try:
                data = self.port.read(self.port.in_waiting or 1)
            except Exception as e:  # noqa: BLE001 — порт выдернули: сообщаем и выходим
                Out.line("ОШИБКА чтения порта: %s" % e)
                self.stop.set()
                return
            if not data:
                continue
            now = time.monotonic()
            with self.lock:
                if now - self.last_rx >= BOOT_GAP_S:
                    self.boot_seen.set()
                self.last_rx = now
                self.rx_bytes += len(data)
                bad_before = self.telem.bad
                good = self.telem.feed(data)
                if good:
                    self.lines_win.add(now, good)
                    ra = self.telem.last["armed"]
                    if ra != self.rov_armed:
                        if self.rov_armed is not None:
                            Out.line("[телем] armed ТНПА %d -> %d  (%s)" % (self.rov_armed, ra, stamp()))
                        self.rov_armed = ra
                if self.telem.bad != bad_before:
                    Out.line("[телем] БИТАЯ строка: %s" % self.telem.last_error)

    def send(self, pkt, bad=False):
        if not write_or_stop(self.port, pkt, self.stop):
            return
        self.sent += 1
        self.sent_win += 1
        if bad:
            self.sent_bad += 1
        if self.args.verbose:
            Out.line("[вниз] %s" % pkt.hex(" "))

    def stats(self, dt, phase_text):
        with self.lock:
            tel_text, _ = self.lines_win.take(dt, TELEMETRY_RATE_HZ)
            t = self.telem
            last = t.last
            ok, bad, oor = t.ok, t.bad, t.out_of_range
        sent = self.sent_win
        self.sent_win = 0
        rate_tx = sent / dt
        if last:
            vals = "ТНПА armed=%d | depth=%.2f temp=%.1f vbat=%s roll=%.1f pitch=%.1f yaw=%.1f" % (
                last["armed"], last["depth"], last["temp"],
                "— (не измеряется)" if last["vbat"] == 0 else "%.2f" % last["vbat"],
                last["roll"], last["pitch"], last["yaw"])
        else:
            vals = "телеметрии нет — проверьте провод GPIO%d rov_core → RX адаптера и GND" % ROV_LINK_TX_PIN
        Out.line("вниз %4.1f пак/с (всего %d, битых намеренно %d) | телем %s, "
                 "целых %d, битых %d, вне диап. %d | %s | %s"
                 % (rate_tx, self.sent, self.sent_bad, tel_text, ok, bad, oor, phase_text, vals))

    def wait_boot(self):
        Out.line("Ожидание перезагрузки rov_core: нажмите EN (RST) на плате. Сценарий стартует по первой "
                 "строке телеметрии после паузы ≥ %.1f с. Пакеты пока не отправляются." % BOOT_GAP_S)
        with self.lock:
            self.boot_seen.clear()
            self.last_rx = time.monotonic()
        last_stats = time.monotonic()
        while not self.stop.is_set():
            if self.boot_seen.wait(0.1):
                Out.line("Перезагрузка rov_core обнаружена (%s), сценарий стартует" % stamp())
                return True
            now = time.monotonic()
            if now - last_stats >= 1.0:
                self.stats(now - last_stats, "ждём EN")
                last_stats = now
        return False

    def run(self):
        args = self.args
        phases = build_scenario(args.scenario)
        Out.line("emulate-buoy: порт %s %d бод, сценарий «%s», управление %d Гц, телеметрия ожидается %d Гц"
                 % (args.port, args.baud, args.scenario, CTRL_RATE_HZ, TELEMETRY_RATE_HZ))
        rx = threading.Thread(target=self.rx_loop, daemon=True)
        rx.start()
        if args.wait_boot and not self.wait_boot():
            return self.finish()

        # Границы фаз в тиках (20 Гц): тайминг детерминирован числом пакетов
        bounds = []
        acc = 0
        for p in phases:
            if p.dur is None:
                bounds.append(None)
            else:
                acc += int(round(p.dur * CTRL_RATE_HZ))
                bounds.append(acc)
        limit_ticks = int(round(args.duration * CTRL_RATE_HZ)) if args.duration > 0 else None

        t_start = time.monotonic()
        last_stats = t_start
        tick = 0
        idx = -1
        light_state = None
        while not self.stop.is_set():
            if limit_ticks is not None and tick >= limit_ticks:
                break
            # Сначала ждём слот тика, потом смотрим фазу: печать фазы совпадает с первым пакетом
            delay = t_start + tick * CTRL_PERIOD_S - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            while idx + 1 < len(phases) and (idx < 0 or (bounds[idx] is not None and tick >= bounds[idx])):
                idx += 1
                p = phases[idx]
                Out.line("ФАЗА %d/%d (%s): %s" % (idx + 1, len(phases),
                                                 "до Ctrl+C" if p.dur is None else "%.2f с" % p.dur,
                                                 p.describe()))
                if p.expect:
                    Out.line("    ожидание в логе rov_core: %s" % p.expect)
            if bounds[idx] is not None and tick >= bounds[idx]:
                break   # последняя фаза закончилась
            p = phases[idx]

            light = p.light
            if p.light_toggle_s:
                light = int((tick * CTRL_PERIOD_S) // p.light_toggle_s) % 2
            if light != light_state:
                if light_state is not None:
                    Out.line("свет -> %d (ожидание: в логе rov_core «свет=%d»)" % (light, light))
                light_state = light
            flags = (CTRL_FLAG_ARMED if p.armed else 0) | (CTRL_FLAG_LIGHT if light else 0)
            ax = [axis_to_i8(v) for v in p.axes]
            if p.mode == "send":
                self.send(build_packet(ax[0], ax[1], ax[2], ax[3], flags))
            elif p.mode == "badcrc":
                k = tick % 4
                if k == 0:
                    self.send(build_packet(ax[0], ax[1], ax[2], ax[3], flags))
                elif k == 2:
                    self.send(build_packet(CTRL_AXIS_MAX + 1, ax[1], ax[2], ax[3], flags), bad=True)
                else:
                    self.send(build_packet(ax[0], ax[1], ax[2], ax[3], flags, bad_crc=True), bad=True)
            tick += 1

            now = time.monotonic()
            if now - last_stats >= 1.0:
                self.stats(now - last_stats, p.describe() if p.mode != "silent" else "ТИШИНА")
                last_stats = now
        if not self.stop.is_set() and bounds[idx] is not None:
            Out.line("Сценарий «%s» завершён." % args.scenario)
        return self.finish()

    def finish(self):
        # Безопасный выход: несколько пакетов armed=0, оси 0
        try:
            for i in range(3):
                if i:
                    time.sleep(CTRL_PERIOD_S)
                self.port.write(build_packet(0, 0, 0, 0, 0))
        except Exception:  # noqa: BLE001 — порт уже недоступен, выходим как есть
            pass
        self.stop.set()
        time.sleep(0.1)
        t = self.telem
        Out.line("ИТОГ: отправлено пакетов %d (битых намеренно %d) | телеметрия: целых %d, битых %d, "
                 "вне диапазона %d, принято %d Б" % (self.sent, self.sent_bad, t.ok, t.bad,
                                                     t.out_of_range, self.rx_bytes))
        if t.last_error:
            Out.line("последняя ошибка телеметрии: %s" % t.last_error)
        self.port.close()
        return 0 if t.bad == 0 else 1


# =============================================================================
#  Режим emulate-rov
# =============================================================================
class RovEmulator:
    def __init__(self, args):
        self.args = args
        self.port = open_port(args.port, args.baud)
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.parser = ControlParser()
        self.t0 = time.monotonic()
        self.last_pkt = None          # время последнего валидного пакета
        self.last = None              # последний валидный пакет
        self.win = RateWindow()
        self.win_dt_max = 0.0
        self.win_dt_min = None
        self.total_dt_max = 0.0
        self.gaps = 0
        self.silence_reported = False
        self.extra_flags = 0
        self.telem_sent = 0
        self.telem_win = 0
        self.csv = open(args.csv, "w", encoding="utf-8") if args.csv else None
        if self.csv:
            self.csv.write("t_s,hex,armed,light,surge,yaw,heave,sway\n")

    def on_packet(self, pkt, now):
        if self.last_pkt is not None:
            dt = now - self.last_pkt
            self.win_dt_max = max(self.win_dt_max, dt)
            self.win_dt_min = dt if self.win_dt_min is None else min(self.win_dt_min, dt)
            self.total_dt_max = max(self.total_dt_max, dt)
            if dt * 1000 > FAILSAFE_TIMEOUT_MS:
                self.gaps += 1
                Out.line("СВЯЗЬ ВЕРНУЛАСЬ после паузы %.0f мс (> %d мс: rov_core ушёл бы в fail-safe)"
                         % (dt * 1000, FAILSAFE_TIMEOUT_MS))
        prev = self.last
        if prev is None:
            Out.line("первый валидный пакет: %s armed=%d light=%d" % (pkt["raw"].hex(" "), pkt["armed"],
                                                                      pkt["light"]))
        else:
            if pkt["armed"] != prev["armed"]:
                Out.line("ПЕРЕХОД armed %d -> %d  (время %s, предыдущий пакет %.0f мс назад)"
                         % (prev["armed"], pkt["armed"], stamp(), (now - self.last_pkt) * 1000))
            if pkt["light"] != prev["light"]:
                Out.line("свет %d -> %d  (%s)" % (prev["light"], pkt["light"], stamp()))
        if pkt["extra_flags"]:
            self.extra_flags += 1
        self.last = pkt
        self.last_pkt = now
        self.silence_reported = False
        self.win.add(now)
        if self.args.verbose:
            Out.line("[пакет] %s" % pkt["raw"].hex(" "))
        if self.csv:
            s, y, h, w = pkt["axes"]
            self.csv.write("%.4f,%s,%d,%d,%d,%d,%d,%d\n" % (now - self.t0, pkt["raw"].hex(), pkt["armed"],
                                                           pkt["light"], s, y, h, w))

    def rx_loop(self):
        while not self.stop.is_set():
            try:
                data = self.port.read(self.port.in_waiting or 1)
            except Exception as e:  # noqa: BLE001 — порт выдернули: сообщаем и выходим
                Out.line("ОШИБКА чтения порта: %s" % e)
                self.stop.set()
                return
            if not data:
                continue
            now = time.monotonic()
            with self.lock:
                c0, r0 = self.parser.crc_err, self.parser.range_err
                for b in data:
                    pkt = self.parser.feed_byte(b)
                    if pkt is not None:
                        self.on_packet(pkt, now)
                if self.args.verbose and (self.parser.crc_err != c0 or self.parser.range_err != r0):
                    Out.line("[битый] CRC %d, вне диапазона %d" % (self.parser.crc_err, self.parser.range_err))

    def stats(self, dt):
        with self.lock:
            rate_text, _ = self.win.take(dt, CTRL_RATE_HZ)
            dmin, dmax = self.win_dt_min, self.win_dt_max
            self.win_dt_min = None
            self.win_dt_max = 0.0
            p = self.parser
            last = self.last
            crc, rng, junk = p.crc_err, p.range_err, p.junk
        tel = self.telem_win / dt
        self.telem_win = 0
        if dmin is None:
            dt_text = "dt —"
        else:
            dt_ok = dmax * 1000 <= CTRL_DT_LIMIT_MS
            dt_text = "dt %3.0f..%3.0f мс %s" % (dmin * 1000, dmax * 1000, "OK" if dt_ok else "WARN")
        if last:
            s, y, h, w = last["axes"]
            pk = "armed=%d light=%d surge=%+4d yaw=%+4d heave=%+4d sway=%+4d" % (
                last["armed"], last["light"], s, y, h, w)
        else:
            pk = "пакетов нет — проверьте провод GPIO%d буя → RX адаптера и GND" % BUOY_LINK_TX_PIN
        Out.line("пакеты %s | %s | битых CRC %d, вне диап. %d, мусор %d Б | %s | телем вверх %.1f/с"
                 % (rate_text, dt_text, crc, rng, junk, pk, tel))

    def run(self):
        args = self.args
        Out.line("emulate-rov: порт %s %d бод; ожидание: пакеты %d ± 1 Гц, dt ≤ %d мс; телеметрия вверх %d Гц%s"
                 % (args.port, args.baud, CTRL_RATE_HZ, CTRL_DT_LIMIT_MS, TELEMETRY_RATE_HZ,
                    ", vbat=0 (--no-vbat)" if args.no_vbat else ""))
        rx = threading.Thread(target=self.rx_loop, daemon=True)
        rx.start()
        t_start = time.monotonic()
        last_stats = t_start
        k = 0
        try:
            while not self.stop.is_set():
                now = time.monotonic()
                if args.duration > 0 and now - t_start >= args.duration:
                    break
                target = t_start + k * TELEM_PERIOD_S
                if now >= target:
                    with self.lock:
                        # Как ROV_CORE без окна старта: armed последнего валидного пакета,
                        # 0 — если валидных пакетов нет дольше FAILSAFE_TIMEOUT_MS
                        rov_armed = (self.last["armed"] if self.last is not None and
                                     (now - self.last_pkt) * 1000 <= FAILSAFE_TIMEOUT_MS else 0)
                    line = synth_telemetry(now - t_start, args.no_vbat, rov_armed)
                    if not write_or_stop(self.port, line, self.stop):
                        break
                    self.telem_sent += 1
                    self.telem_win += 1
                    k += 1
                with self.lock:
                    if (self.last_pkt is not None and not self.silence_reported
                            and (now - self.last_pkt) * 1000 > FAILSAFE_TIMEOUT_MS):
                        self.silence_reported = True
                        Out.line("НЕТ ПАКЕТОВ уже %d мс (%s) — rov_core в этот момент ушёл бы в fail-safe"
                                 % (FAILSAFE_TIMEOUT_MS, stamp()))
                if now - last_stats >= 1.0:
                    self.stats(now - last_stats)
                    last_stats = now
                time.sleep(max(0.0, min(0.01, t_start + k * TELEM_PERIOD_S - time.monotonic())))
        finally:
            self.stop.set()
            time.sleep(0.1)
            p = self.parser
            Out.line("ИТОГ: валидных пакетов %d, битых CRC %d, вне диапазона %d, мусор %d Б, "
                     "пауз > %d мс: %d, макс. интервал %.0f мс, лишние биты flags: %d | телеметрии отправлено %d"
                     % (p.valid, p.crc_err, p.range_err, p.junk, FAILSAFE_TIMEOUT_MS, self.gaps,
                        self.total_dt_max * 1000, self.extra_flags, self.telem_sent))
            if self.csv:
                self.csv.close()
            self.port.close()
        return 0 if (p.crc_err == 0 and p.range_err == 0) else 1


# =============================================================================
#  Режим send-video
# =============================================================================
def video_frame(seq, jpeg, corrupt=False):
    hdr = struct.pack(VIDEO_HDR_FMT, VIDEO_MAGIC1, VIDEO_MAGIC2, seq & 0xFFFF, len(jpeg), 0)
    hdr += bytes([xor8(hdr)])
    crc = crc16_ccitt(jpeg)
    if corrupt:
        crc ^= 0x5555
    return hdr + jpeg + struct.pack("<H", crc)


def run_send_video(args):
    port = open_port(args.port, args.baud)
    stop = threading.Event()
    line_bps = args.baud / 10.0          # 8N1: 10 бит на байт
    t_start = time.monotonic()
    last_stats = t_start
    win_frames = win_bytes = 0
    tot_frames = tot_bytes = tot_bad = 0
    try:
        if args.file:
            with open(args.file, "rb") as fh:
                data = fh.read()
            bps = args.bps if args.bps > 0 else line_bps
            Out.line("send-video: файл %s (%d Б) -> %s %d бод, скорость %.0f Б/с%s"
                     % (args.file, len(data), args.port, args.baud, bps, ", по кругу" if args.loop else ""))
            pos = 0
            while True:
                if pos >= len(data):
                    if not args.loop:
                        break
                    pos = 0
                    tot_frames += 1   # здесь — число проходов файла
                chunk = data[pos:pos + VIDEO_CHUNK]
                pos += len(chunk)
                if not write_or_stop(port, chunk, stop):
                    return 1
                tot_bytes += len(chunk)
                win_bytes += len(chunk)
                target = t_start + tot_bytes / bps
                delay = target - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                now = time.monotonic()
                if args.duration > 0 and now - t_start >= args.duration:
                    break
                if now - last_stats >= 1.0:
                    Out.line("отправлено %.1f КиБ/с, всего %d Б (%.1f%% файла)"
                             % (win_bytes / (now - last_stats) / 1024, tot_bytes, 100.0 * pos / len(data)))
                    win_bytes = 0
                    last_stats = now
            if not args.loop:
                tot_frames = 1
            port.flush()
            Out.line("ИТОГ: отправлено %d Б, проходов файла %d, %.1f с" % (tot_bytes, tot_frames,
                                                                          time.monotonic() - t_start))
            return 0

        from jpeg_mini import builtin_frame
        Out.line("send-video: генератор кадров 320x240, %.1f к/с -> %s %d бод%s"
                 % (args.fps, args.port, args.baud,
                    (", каждый %d-й кадр с битым CRC" % args.corrupt) if args.corrupt else ""))
        period = 1.0 / args.fps
        seq = 0
        warned = False
        while True:
            now = time.monotonic()
            if args.duration > 0 and now - t_start >= args.duration:
                break
            if args.count and tot_frames >= args.count:
                break
            target = t_start + tot_frames * period
            if target > now:
                time.sleep(target - now)
            jpeg = builtin_frame(seq, time.monotonic() - t_start)
            bad = bool(args.corrupt) and (seq % args.corrupt == args.corrupt - 1)
            frame = video_frame(seq, jpeg, corrupt=bad)
            if not warned and len(frame) * args.fps > line_bps:
                Out.line("WARN: %d Б x %.1f к/с больше пропускной способности линии %.0f Б/с"
                         % (len(frame), args.fps, line_bps))
                warned = True
            if not write_or_stop(port, frame, stop):
                return 1
            seq = (seq + 1) & 0xFFFF
            tot_frames += 1
            tot_bytes += len(frame)
            tot_bad += 1 if bad else 0
            win_frames += 1
            win_bytes += len(frame)
            now = time.monotonic()
            if now - last_stats >= 1.0:
                dt = now - last_stats
                Out.line("отправлено %.1f к/с, %.1f КиБ/с | всего кадров %d (битых намеренно %d), %d Б"
                         % (win_frames / dt, win_bytes / dt / 1024, tot_frames, tot_bad, tot_bytes))
                win_frames = win_bytes = 0
                last_stats = now
        port.flush()
        el = time.monotonic() - t_start
        Out.line("ИТОГ: кадров %d (битых намеренно %d), %d Б за %.1f с, средняя %.2f к/с"
                 % (tot_frames, tot_bad, tot_bytes, el, tot_frames / el if el > 0 else 0.0))
        return 0
    finally:
        port.close()


# =============================================================================
#  Командная строка
# =============================================================================
WIRING_BUOY = """\
Подключение (ПК вместо буя, проверяется ROV_CORE):
  TX адаптера  -> GPIO{rx} rov_core (ROV_LINK_RX_PIN, управление вниз)
  RX адаптера  <- GPIO{tx} rov_core (ROV_LINK_TX_PIN, телеметрия вверх)
  GND адаптера -- GND rov_core (обязательно)
  Адаптер строго 3.3 В по сигналам. Лог rov_core — по его собственному USB:
  pio device monitor -e rov_core. ВИНТЫ СНЯТЫ перед arm-sweep/failsafe/edge/badcrc.

Сценарии: idle, arm-sweep, failsafe, edge, badcrc, light (что ждать в логе —
печатается в начале каждой фазы). Для edge нужен свежий сброс платы:
по умолчанию включён --wait-boot (нажмите EN после запуска утилиты).

Пример:
  python3 tools/bench_link.py emulate-buoy --port /dev/tty.usbserial-0001 --scenario arm-sweep
  python3 tools/bench_link.py emulate-buoy --port COM5 --scenario edge
""".format(rx=ROV_LINK_RX_PIN, tx=ROV_LINK_TX_PIN)

WIRING_ROV = """\
Подключение (ПК вместо ТНПА, проверяется буй):
  RX адаптера  <- GPIO{tx} буя (BUOY_LINK_TX_PIN, управление вниз)
  TX адаптера  -> GPIO{rx} буя (BUOY_LINK_RX_PIN, телеметрия снизу)
  GND адаптера -- GND буя (обязательно)
  Адаптер строго 3.3 В по сигналам.

Телеметрия вверх — протокол v2: armed = флаг последнего валидного пакета,
0 при отсутствии пакетов > {fs} мс (как ROV_CORE, без окна старта).

Ожидание: пакеты {hz} ± 1 Гц, dt ≤ {dt} мс, битых 0. Переходы armed 1->0
печатаются с временем: закрыли вкладку -> armed=0 сразу; телефон ушёл из
Wi-Fi -> armed=0 не позже {ws} мс после последней cmd.

Пример:
  python3 tools/bench_link.py emulate-rov --port /dev/tty.usbserial-0001 --no-vbat
""".format(fs=FAILSAFE_TIMEOUT_MS, tx=BUOY_LINK_TX_PIN, rx=BUOY_LINK_RX_PIN, hz=CTRL_RATE_HZ, dt=CTRL_DT_LIMIT_MS,
           ws=BUOY_WS_TIMEOUT_MS)

WIRING_VIDEO = """\
Подключение (ПК вместо камеры, проверяется видеовход буя):
  TX адаптера  -> GPIO{vrx} буя (BUOY_VIDEO_RX_PIN, UART1 RX, {baud} бод)
  GND адаптера -- GND буя (обязательно). RX адаптера не нужен.
  Адаптер строго 3.3 В и должен держать {baud} бод (CP2102/CH340).

Без --file — встроенный генератор кадров (tools/jpeg_mini.py), с --file —
сырой поток как есть (например test/test_buoy/fixtures/cam_stream.bin).

Пример:
  python3 tools/bench_link.py send-video --port /dev/tty.usbserial-0001 --fps 10
  python3 tools/bench_link.py send-video --port COM5 --file test/test_buoy/fixtures/cam_stream.bin --loop
""".format(vrx=BUOY_VIDEO_RX_PIN, baud=VIDEO_BAUD)


def positive_float(s):
    v = float(s)
    if not v > 0:
        raise argparse.ArgumentTypeError("нужно число > 0")
    return v


def main():
    ap = argparse.ArgumentParser(
        description="Стенд ТНПА: ПК вместо буя, ТНПА или камеры (контракты №2–4, протокол v%d)."
                    % PROTOCOL_VERSION,
        epilog="Подробности: python3 tools/bench_link.py <режим> --help",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="mode", required=True)

    pb = sub.add_parser("emulate-buoy", help="ПК вместо буя: проверка ROV_CORE",
                        description="ПК вместо буя: ControlPacket %d Гц по сценарию + проверка телеметрии."
                                    % CTRL_RATE_HZ,
                        epilog=WIRING_BUOY, formatter_class=argparse.RawDescriptionHelpFormatter)
    pb.add_argument("--port", required=True, help="порт USB-UART адаптера")
    pb.add_argument("--baud", type=int, default=LINK_BAUD, help="скорость (по умолчанию %d)" % LINK_BAUD)
    pb.add_argument("--scenario", choices=SCENARIOS, default="idle", help="сценарий (по умолчанию idle)")
    pb.add_argument("--wait-boot", dest="wait_boot", action="store_true", default=None,
                    help="перед сценарием ждать сброса rov_core (по умолчанию включено только для edge)")
    pb.add_argument("--no-wait-boot", dest="wait_boot", action="store_false",
                    help="не ждать сброса rov_core")
    pb.add_argument("--duration", type=float, default=0.0,
                    help="остановить через N с (0 — до конца сценария или Ctrl+C)")
    pb.add_argument("-v", "--verbose", action="store_true", help="печатать каждый отправленный пакет")

    pr = sub.add_parser("emulate-rov", help="ПК вместо ТНПА: проверка буя (управление/телеметрия)",
                        description="ПК вместо ТНПА: приём ControlPacket, телеметрия %d Гц."
                                    % TELEMETRY_RATE_HZ,
                        epilog=WIRING_ROV, formatter_class=argparse.RawDescriptionHelpFormatter)
    pr.add_argument("--port", required=True, help="порт USB-UART адаптера")
    pr.add_argument("--baud", type=int, default=LINK_BAUD, help="скорость (по умолчанию %d)" % LINK_BAUD)
    pr.add_argument("--no-vbat", action="store_true", help="vbat=0 в телеметрии («не измеряется», D-009)")
    pr.add_argument("--duration", type=float, default=0.0, help="секунд работы (0 — до Ctrl+C)")
    pr.add_argument("--csv", help="записывать каждый валидный пакет в CSV-файл")
    pr.add_argument("-v", "--verbose", action="store_true", help="печатать каждый принятый пакет")

    pv = sub.add_parser("send-video", help="ПК вместо камеры: проверка видеовхода буя",
                        description="ПК вместо камеры: кадры контракта №4 в UART1 буя.",
                        epilog=WIRING_VIDEO, formatter_class=argparse.RawDescriptionHelpFormatter)
    pv.add_argument("--port", required=True, help="порт USB-UART адаптера")
    pv.add_argument("--baud", type=int, default=VIDEO_BAUD, help="скорость (по умолчанию %d)" % VIDEO_BAUD)
    pv.add_argument("--file", help="слать сырой поток из файла как есть")
    pv.add_argument("--loop", action="store_true", help="с --file: повторять файл по кругу")
    pv.add_argument("--bps", type=float, default=0.0,
                    help="с --file: байт/с (по умолчанию пропускная способность линии baud/10)")
    pv.add_argument("--fps", type=positive_float, default=float(CAM_TARGET_FPS),
                    help="кадров/с генератора (по умолчанию CAM_TARGET_FPS=%d)" % CAM_TARGET_FPS)
    pv.add_argument("--count", type=int, default=0, help="отправить N кадров и выйти (0 — без ограничения)")
    pv.add_argument("--corrupt", type=int, default=0,
                    help="портить CRC каждого N-го кадра (проверка счётчика битых CRC буя)")
    pv.add_argument("--duration", type=float, default=0.0, help="секунд работы (0 — до Ctrl+C)")

    args = ap.parse_args()
    assert crc16_ccitt(b"123456789") == 0x29B1, "самопроверка CRC16 не прошла"
    if args.mode == "send-video" and args.file and not os.path.isfile(args.file):
        ap.error("файл не найден: %s" % args.file)
    if args.mode == "send-video" and args.corrupt < 0:
        ap.error("--corrupt должен быть ≥ 0")

    Out.t0 = time.monotonic()
    try:
        if args.mode == "emulate-buoy":
            if args.wait_boot is None:
                args.wait_boot = args.scenario == "edge"
            emu = BuoyEmulator(args)
            try:
                return emu.run()
            except KeyboardInterrupt:
                Out.line("Ctrl+C: шлю armed=0 и выхожу")
                return emu.finish()
        if args.mode == "emulate-rov":
            return RovEmulator(args).run()
        return run_send_video(args)
    except KeyboardInterrupt:
        Out.line("Ctrl+C: выход")
        return 130


if __name__ == "__main__":
    sys.exit(main())
