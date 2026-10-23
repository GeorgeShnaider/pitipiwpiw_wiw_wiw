"""
py_ref.py — эталонный разбор потока приёмником tools/cam_rx.py (FrameParser,
порции по 4096 Б; результат от нарезки не зависит). Формат вывода — файл
ожидаемых результатов для test/test_buoy:
    <seq> <len> <crc32 hex8>        — по строке на целый кадр
    STATS ok=.. bad_hdr=.. bad_crc=.. junk=..

Запуск (из корня проекта):
    python3 -I test/test_buoy/fixtures/gen/py_ref.py <поток.bin> [путь к cam_rx.py] > <поток>.expected
"""
import binascii, importlib.util, os, sys

here = os.path.dirname(os.path.abspath(__file__))
cam_rx = sys.argv[2] if len(sys.argv) > 2 else os.path.join(here, "..", "..", "..", "..", "tools", "cam_rx.py")
spec = importlib.util.spec_from_file_location("cam_rx", cam_rx)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)

lines = []
p = m.FrameParser(lambda s, f, j: lines.append("%d %d %08x" % (s, len(j), binascii.crc32(j))))
data = open(sys.argv[1], "rb").read()
for i in range(0, len(data), 4096):
    p.feed(data[i:i + 4096])
for l in lines:
    print(l)
print("STATS ok=%d bad_hdr=%d bad_crc=%d junk=%d" % (p.ok, p.bad_hdr, p.bad_crc, p.junk_bytes))
