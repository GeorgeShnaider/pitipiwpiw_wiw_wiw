# Эталон CRC4 PROM MS5837 для test_rov_core (тест от Python не зависит).
# Независимая формулировка: остаток от деления полинома сообщения на x^4+x+1.
# Сообщение — 8 слов MSB-first (C0 & 0x0FFF, C1..C6, C7 = 0). 16-битный регистр
# алгоритма AN520 эквивалентен M(x)/x^4 mod P(x): последние 4 нулевых бита C7
# отбрасываются.
# Перегенерация: python3 -I test/test_rov_core/ref/crcref.py test/test_rov_core/crc_ref.inc
import random
import sys

C = [0, 34982, 36352, 20328, 22354, 26646, 26146]


def polymod(bits):
    reg = 0
    for b in bits:
        reg = (reg << 1) | b
        if reg & 0x10:
            reg ^= 0x13
    return reg


def crc4(words):
    w = [words[0] & 0x0FFF] + list(words[1:7]) + [0]
    bits = [(x >> k) & 1 for x in w for k in range(15, -1, -1)]
    return polymod(bits[:-4])


print("CRC4 C1..C6 даташита:", crc4(C))
random.seed(5837)
rows = []
for _ in range(20):
    w = [random.randrange(65536) for _ in range(7)]
    rows.append("{{%s}, %d}" % (",".join("0x%04X" % x for x in w), crc4(w)))
with open(sys.argv[1], "w") as f:
    f.write("// Сгенерировано test/test_rov_core/ref/crcref.py — не править вручную.\n")
    f.write("// Эталон CRC4 MS5837: остаток M(x)/x^4 mod (x^4+x+1), 20 случайных PROM (seed 5837).\n")
    f.write("static const struct { uint16_t w[7]; uint8_t crc; } CRC_REF[] = {\n    ")
    f.write(",\n    ".join(rows) + "\n};\n")
