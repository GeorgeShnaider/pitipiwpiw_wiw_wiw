#!/usr/bin/env python3
"""
cam_rx.py — приёмник видеопотока ESP32-CAM по контракту №4 (docs/ICD.md §7).

Кадр по линии:
    [VideoFrameHeader 10 Б][JPEG length Б][CRC16-CCITT(JPEG) 2 Б, little-endian]
    VideoFrameHeader (<BBHIBB): magic1=0xA5, magic2=0x5A, seq u16, length u32
    (1..32768), flags u8 = 0, hcrc u8 = XOR байт 0..8.
    CRC16-CCITT/FALSE: полином 0x1021, начальное значение 0xFFFF.

Режимы:
    python3 cam_rx.py --port /dev/tty.usbserial-0001          # с USB-UART (921600)
    python3 cam_rx.py --file stream.bin --out frames           # разбор записи из файла
    python3 cam_rx.py --port COM5 --dump raw.bin --duration 30 # приём + запись сырого потока

Подключение без буя: RX USB-UART (CP2102/CH340, умеющий 921600) к GPIO13
ESP32-CAM, общая земля. Для pyserial: pip install pyserial.
"""

import argparse
import binascii
import os
import struct
import sys
import time

MAGIC = b"\xA5\x5A"
HDR_FMT = "<BBHIBB"
HDR_LEN = struct.calcsize(HDR_FMT)  # 10
TRAILER_LEN = 2
VIDEO_MAX_FRAME = 32768


def crc16_ccitt(data, crc=0xFFFF):
    """CRC16-CCITT/FALSE (poly 0x1021, init 0xFFFF) — как crc16Update в protocol.h."""
    return binascii.crc_hqx(data, crc)


def xor8(data):
    x = 0
    for b in data:
        x ^= b
    return x


class FrameParser:
    """Потоковый разборщик: feed() принимает байты кусками любого размера."""

    def __init__(self, on_frame):
        self.buf = bytearray()
        self.on_frame = on_frame
        self.ok = 0
        self.bad_hdr = 0
        self.bad_crc = 0
        self.junk_bytes = 0
        self.seq_gaps = 0
        self.not_jpeg = 0
        self.last_seq = None

    def feed(self, data):
        self.buf += data
        while True:
            i = self.buf.find(MAGIC)
            if i < 0:
                # Сохраняем последний байт: он может быть началом магии
                keep = 1 if self.buf[-1:] == MAGIC[:1] else 0
                self.junk_bytes += len(self.buf) - keep
                del self.buf[: len(self.buf) - keep]
                return
            if i > 0:
                self.junk_bytes += i
                del self.buf[:i]
            if len(self.buf) < HDR_LEN:
                return
            m1, m2, seq, length, flags, hcrc = struct.unpack_from(HDR_FMT, self.buf, 0)
            if xor8(self.buf[: HDR_LEN - 1]) != hcrc or not (0 < length <= VIDEO_MAX_FRAME):
                # Плохой заголовок: поиск магии заново со следующего байта
                self.bad_hdr += 1
                self.junk_bytes += 1
                del self.buf[:1]
                continue
            total = HDR_LEN + length + TRAILER_LEN
            if len(self.buf) < total:
                return
            jpeg = bytes(self.buf[HDR_LEN: HDR_LEN + length])
            (rx_crc,) = struct.unpack_from("<H", self.buf, HDR_LEN + length)
            if crc16_ccitt(jpeg) != rx_crc:
                # Битый кадр: отбросить и искать магию со следующего байта
                # (если кадр был обрезан, следующий кадр может начаться внутри него)
                self.bad_crc += 1
                self.junk_bytes += 1
                del self.buf[:1]
                continue
            del self.buf[:total]
            if self.last_seq is not None and seq != ((self.last_seq + 1) & 0xFFFF):
                self.seq_gaps += 1
            self.last_seq = seq
            if not (jpeg[:2] == b"\xFF\xD8" and jpeg[-2:] == b"\xFF\xD9"):
                self.not_jpeg += 1
            self.ok += 1
            self.on_frame(seq, flags, jpeg)


def main():
    ap = argparse.ArgumentParser(description="Приёмник видео ESP32-CAM (контракт №4)")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--port", help="последовательный порт USB-UART")
    src.add_argument("--file", help="разобрать записанный поток из файла")
    ap.add_argument("--baud", type=int, default=921600, help="скорость (по умолчанию 921600)")
    ap.add_argument("--out", default="frames", help="каталог для .jpg (по умолчанию frames)")
    ap.add_argument("--no-save", action="store_true", help="не сохранять кадры, только статистика")
    ap.add_argument("--dump", help="дописывать сырой поток в файл (режим --port)")
    ap.add_argument("--duration", type=float, default=0.0, help="секунд приёма (0 = до Ctrl+C)")
    ap.add_argument("--chunk", type=int, default=4096, help="размер порции чтения из файла")
    args = ap.parse_args()

    assert crc16_ccitt(b"123456789") == 0x29B1, "самопроверка CRC16 не прошла"

    if not args.no_save:
        os.makedirs(args.out, exist_ok=True)

    sizes = []
    win = {"frames": 0, "bytes": 0}

    def on_frame(seq, flags, jpeg):
        sizes.append(len(jpeg))
        win["frames"] += 1
        win["bytes"] += len(jpeg)
        if not args.no_save:
            name = os.path.join(args.out, "frame_%06d_seq%05d.jpg" % (parser.ok, seq))
            with open(name, "wb") as f:
                f.write(jpeg)

    parser = FrameParser(on_frame)

    def stats_line(prefix, dt):
        fps = win["frames"] / dt if dt > 0 else 0.0
        kbps = win["bytes"] / 1024.0 / dt if dt > 0 else 0.0
        total = parser.ok + parser.bad_crc
        bad_pct = 100.0 * parser.bad_crc / total if total else 0.0
        return ("%s %.1f к/с, %.1f КиБ/с | целых %d, битых CRC %d (%.2f%%), плохих заголовков %d, "
                "пропусков seq %d, не-JPEG %d, мусора %d Б" % (
                    prefix, fps, kbps, parser.ok, parser.bad_crc, bad_pct, parser.bad_hdr,
                    parser.seq_gaps, parser.not_jpeg, parser.junk_bytes))

    t_start = time.monotonic()
    if args.file:
        with open(args.file, "rb") as f:
            while True:
                data = f.read(args.chunk)
                if not data:
                    break
                parser.feed(data)
        print(stats_line("[файл]", 0.0))
    else:
        try:
            import serial  # pyserial
        except ImportError:
            print("нужен pyserial: pip install pyserial", file=sys.stderr)
            return 2
        dump = open(args.dump, "ab") if args.dump else None
        ser = serial.Serial(args.port, args.baud, timeout=0.05)
        t_win = time.monotonic()
        try:
            while True:
                data = ser.read(ser.in_waiting or 1)
                if data:
                    if dump:
                        dump.write(data)
                    parser.feed(data)
                now = time.monotonic()
                if now - t_win >= 1.0:
                    print(stats_line("[порт]", now - t_win), flush=True)
                    win["frames"] = 0
                    win["bytes"] = 0
                    t_win = now
                if args.duration and now - t_start >= args.duration:
                    break
        except KeyboardInterrupt:
            pass
        finally:
            ser.close()
            if dump:
                dump.close()

    elapsed = time.monotonic() - t_start
    total = parser.ok + parser.bad_crc
    print("ИТОГ: целых кадров %d, битых CRC %d (%.2f%%), плохих заголовков %d, пропусков seq %d, "
          "не-JPEG %d" % (parser.ok, parser.bad_crc, 100.0 * parser.bad_crc / total if total else 0.0,
                          parser.bad_hdr, parser.seq_gaps, parser.not_jpeg))
    if sizes:
        print("размер кадра: мин %d, сред %d, макс %d Б" % (min(sizes), sum(sizes) // len(sizes), max(sizes)))
    if args.port and elapsed > 0:
        print("средняя частота: %.2f к/с за %.1f с" % (parser.ok / elapsed, elapsed))
    return 0


if __name__ == "__main__":
    sys.exit(main())
