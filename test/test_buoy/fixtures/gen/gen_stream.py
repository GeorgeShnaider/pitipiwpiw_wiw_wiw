"""
gen_stream.py — синтетический видеопоток контракта №4 (копия генератора
субагента cam): целые кадры, мусор, ложная магия в JPEG, битый CRC,
плохой заголовок (hcrc), обрезанный кадр и в конце кадр, собранный C++-кодом
прошивки камеры (cpp_frame.bin, seq 777).

Перегенерация фикстуры (из корня проекта):
    python3 -I test/test_buoy/fixtures/gen/gen_stream.py \
        test/test_buoy/fixtures/cam_stream.bin test/test_buoy/fixtures/gen/cpp_frame.bin
    python3 -I test/test_buoy/fixtures/gen/py_ref.py test/test_buoy/fixtures/cam_stream.bin \
        > test/test_buoy/fixtures/cam_stream.expected
cpp_frame.bin получен утилитой emit_frame субагента cam (src/cam/video_frame.h);
поток детерминирован (random.seed(42)).
"""
import binascii, random, struct, sys
random.seed(42)

def frame(seq, jpeg):
    h = struct.pack("<BBHIB", 0xA5, 0x5A, seq & 0xFFFF, len(jpeg), 0)
    x = 0
    for b in h: x ^= b
    return h + bytes([x]) + jpeg + struct.pack("<H", binascii.crc_hqx(jpeg, 0xFFFF))

def fake_jpeg(n):
    body = bytearray(random.getrandbits(8) for _ in range(n - 4))
    body[10:12] = b"\xA5\x5A"          # ложная магия внутри данных
    return b"\xFF\xD8" + bytes(body) + b"\xFF\xD9"

out = bytearray()
out += bytes(random.getrandbits(8) for _ in range(137)) + b"\xA5"   # мусор в начале + полмагии
seq = 0
for k in range(12):
    j = fake_jpeg(random.randint(3000, 12000))
    f = bytearray(frame(seq, j))
    if k == 4:                       # битый кадр: испорчен байт JPEG -> CRC не сойдётся
        f[10 + 500] ^= 0x40
    elif k == 7:                     # плохой заголовок (hcrc)
        f[9] ^= 0xFF
    elif k == 9:                     # обрезанный кадр (обрыв посреди JPEG)
        f = f[: len(f) // 2]
    out += f
    seq += 1
    if k % 3 == 0:
        out += bytes(random.getrandbits(8) for _ in range(random.randint(1, 300)))  # мусор между кадрами
out += open(sys.argv[2], "rb").read()                                         # кадр от C++ (seq 777)
open(sys.argv[1], "wb").write(out)
