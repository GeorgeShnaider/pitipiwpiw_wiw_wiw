"""
gen_extra.py — дополнительный поток для парсера видео буя (контракт №4).
Случаи:
  - магия A5 A5 A5 5A с плохим заголовком;
  - кадр максимальной длины 32768 (принимается);
  - заголовок с длиной 32769 (плохой заголовок; тело укорочено — для решения
    достаточно заголовка);
  - кадр длины 1; заголовок с длиной 0 (плохой);
  - целый кадр 103 внутри кадра 104 с битым CRC (восстанавливается);
  - JPEG, целиком состоящий из A5 5A;
  - обрезанный кадр 107, сразу за ним целый 108 (восстанавливается);
  - валидный заголовок 109 без данных, внутри его окна 10 настоящих кадров
    110..119 и мусор, дополняющий окно (все 10 восстанавливаются);
  - кадр 120 после этого и полмагии A5 в самом конце.

Перегенерация (из корня проекта):
    python3 -I test/test_buoy/fixtures/gen/gen_extra.py test/test_buoy/fixtures/extra.bin
    python3 -I test/test_buoy/fixtures/gen/py_ref.py test/test_buoy/fixtures/extra.bin \
        > test/test_buoy/fixtures/extra.expected
"""
import binascii, random, struct, sys
random.seed(7)

def hdr(seq, n):
    h = struct.pack("<BBHIB", 0xA5, 0x5A, seq & 0xFFFF, n, 0)
    x = 0
    for b in h: x ^= b
    return h + bytes([x])

def frame(seq, j):
    return hdr(seq, len(j)) + j + struct.pack("<H", binascii.crc_hqx(j, 0xFFFF))

def rnd(n):
    return bytes(random.getrandbits(8) for _ in range(n))

FAKE_LEN = 8000
out = bytearray()
out += b"\xA5\xA5\xA5\x5A\x00"                         # магия с плохим заголовком
out += frame(100, rnd(32768))                           # максимальный кадр
out += hdr(101, 32769) + rnd(100)                       # слишком длинный -> плохой заголовок
out += frame(102, b"\x42")                              # длина 1
inner = frame(103, rnd(500))
outer = bytearray(frame(104, rnd(200) + inner + rnd(300)))
outer[-1] ^= 0x01                                       # внешний битый CRC, внутри целый кадр 103
out += outer
out += hdr(105, 0)                                      # длина 0 -> плохой заголовок
out += frame(106, b"\xA5\x5A" * 3000)                   # JPEG сплошь из магии
out += frame(107, rnd(8000))[:3000] + frame(108, rnd(2000))  # обрезанный, следом целый
out += hdr(109, FAKE_LEN)                               # валидный заголовок без данных
inside = bytearray()
for s in range(110, 120):
    inside += frame(s, rnd(random.randint(1, 500)))
out += inside
out += rnd(FAKE_LEN + 2 - len(inside) + 500)            # мусор до конца окна ложного кадра и дальше
out += frame(120, rnd(1234))
out += b"\xA5"                                          # хвост: полмагии
open(sys.argv[1], "wb").write(out)
