"""Минимальный baseline-JPEG кодер без внешних зависимостей (общий модуль tools/).

Используется mock_buoy.py (видео в WebApp без Pillow) и bench_link.py send-video
(кадры для видеоканала буя). Каждый блок 8x8 залит одним цветом, кодируется
только DC, все AC = 0 (EOB), YCbCr 4:4:4. Результат — валидный JFIF (FFD8..FFD9).
"""

import math

# =============================================================================
#  Минимальный baseline-JPEG кодер (без Pillow): каждый блок 8x8 залит одним
#  цветом, кодируется только DC, все AC = 0 (EOB). YCbCr 4:4:4.
# =============================================================================
_DC_BITS = [0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0]
_DC_VALS = list(range(12))
_AC_BITS = [1] + [0] * 15       # единственный символ 0x00 (EOB), код "0"
_AC_VALS = [0x00]
_DC_Q = 8                       # DC = 8*(v-128) -> квантованное = v-128


def _huff_codes(bits, vals):
    codes = {}
    code = 0
    k = 0
    for length in range(1, 17):
        for _ in range(bits[length - 1]):
            codes[vals[k]] = (code, length)
            code += 1
            k += 1
        code <<= 1
    return codes


_DC_CODES = _huff_codes(_DC_BITS, _DC_VALS)
_AC_EOB = _huff_codes(_AC_BITS, _AC_VALS)[0]


class _BitWriter:
    def __init__(self):
        self.out = bytearray()
        self.acc = 0
        self.n = 0

    def put(self, value, length):
        self.acc = (self.acc << length) | (value & ((1 << length) - 1))
        self.n += length
        while self.n >= 8:
            self.n -= 8
            b = (self.acc >> self.n) & 0xFF
            self.out.append(b)
            if b == 0xFF:
                self.out.append(0x00)
        self.acc &= (1 << self.n) - 1

    def flush(self):
        if self.n:
            self.put((1 << (8 - self.n)) - 1, 8 - self.n)


def _seg(marker, payload):
    return bytes([0xFF, marker]) + (len(payload) + 2).to_bytes(2, "big") + payload


def encode_blocks_jpeg(blocks, bw, bh):
    """blocks: список (r, g, b) длиной bw*bh, построчно. Размер кадра 8bw x 8bh."""
    w, h = bw * 8, bh * 8
    out = bytearray(b"\xFF\xD8")
    out += _seg(0xE0, b"JFIF\x00\x01\x01\x00\x00\x01\x00\x01\x00\x00")
    out += _seg(0xDB, bytes([0x00]) + bytes([_DC_Q] * 64))
    out += _seg(0xC0, bytes([8]) + h.to_bytes(2, "big") + w.to_bytes(2, "big") +
                bytes([3, 1, 0x11, 0, 2, 0x11, 0, 3, 0x11, 0]))
    out += _seg(0xC4, bytes([0x00] + _DC_BITS + _DC_VALS))
    out += _seg(0xC4, bytes([0x10] + _AC_BITS + _AC_VALS))
    out += _seg(0xDA, bytes([3, 1, 0x00, 2, 0x00, 3, 0x00, 0, 63, 0]))
    bwr = _BitWriter()
    pred = [0, 0, 0]
    eob_code, eob_len = _AC_EOB
    for (r, g, b) in blocks:
        y = 0.299 * r + 0.587 * g + 0.114 * b
        cb = 128 - 0.168736 * r - 0.331264 * g + 0.5 * b
        cr = 128 + 0.5 * r - 0.418688 * g - 0.081312 * b
        for i, v in enumerate((y, cb, cr)):
            dc = int(round(max(0.0, min(255.0, v)))) - 128
            diff = dc - pred[i]
            pred[i] = dc
            mag = abs(diff)
            cat = mag.bit_length()
            code, length = _DC_CODES[cat]
            bwr.put(code, length)
            if cat:
                bwr.put(diff if diff >= 0 else diff + (1 << cat) - 1, cat)
            bwr.put(eob_code, eob_len)
    bwr.flush()
    out += bwr.out
    out += b"\xFF\xD9"
    return bytes(out)


# Шрифт 3x5 для цифр (номер кадра на мозаике)
_FONT = {
    "0": ["111", "101", "101", "101", "111"], "1": ["010", "110", "010", "010", "111"],
    "2": ["111", "001", "111", "100", "111"], "3": ["111", "001", "111", "001", "111"],
    "4": ["101", "101", "111", "001", "001"], "5": ["111", "100", "111", "001", "111"],
    "6": ["111", "100", "111", "101", "111"], "7": ["111", "001", "010", "010", "010"],
    "8": ["111", "101", "111", "101", "111"], "9": ["111", "101", "111", "001", "111"],
}


def builtin_frame(n, t, bw=40, bh=30):
    """Тестовый кадр 320x240: градиент, бегущая полоса, перекрестие, номер кадра."""
    blocks = []
    bar = int((t * 8) % bw)
    for by in range(bh):
        for bx in range(bw):
            r = int(20 + 40 * by / bh)
            g = int(60 + 80 * (0.5 + 0.5 * math.sin(t + bx * 0.2)))
            b = int(110 + 100 * by / bh)
            if bx == bar or bx == (bar + 1) % bw:
                r, g, b = 240, 200, 40
            if bx == bw // 2 or by == bh // 2:
                r, g, b = 230, 230, 230
            blocks.append((r, g, b))
    text = str(n % 100000)
    x0, y0 = 2, 2
    for ch in text:
        glyph = _FONT[ch]
        for gy, row in enumerate(glyph):
            for gx, bit in enumerate(row):
                if bit == "1":
                    blocks[(y0 + gy) * bw + x0 + gx] = (255, 255, 255)
        x0 += 4
    return encode_blocks_jpeg(blocks, bw, bh)
