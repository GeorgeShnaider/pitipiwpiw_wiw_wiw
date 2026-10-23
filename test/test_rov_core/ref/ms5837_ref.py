# Эталон компенсации MS5837-30BA для test_rov_core (тест от Python не зависит).
# Формулы даташита (первый и второй порядок) на целых Python; // — деление с
# округлением вниз, как сдвиг в даташите. Проверка формулы: пример даташита
# D1=4958179, D2=6815414 -> TEMP=1981, P=39998.
# Перегенерация: python3 -I test/test_rov_core/ref/ms5837_ref.py
# Вывод (d1, d2, первый порядок (T, P), второй порядок (T, P)) переносится
# вручную в таблицу ref[] функции test_ms5837_compensation в test_main.cpp.
C = [0, 34982, 36352, 20328, 22354, 26646, 26146]


def calc(D1, D2, second):
    dT = D2 - C[5] * 2**8
    TEMP = 2000 + dT * C[6] // 2**23
    OFF = C[2] * 2**16 + (C[4] * dT) // 2**7
    SENS = C[1] * 2**15 + (C[3] * dT) // 2**8
    if second:
        if TEMP < 2000:
            Ti = 3 * dT * dT // 2**33
            OFFi = 3 * (TEMP - 2000) ** 2 // 2
            SENSi = 5 * (TEMP - 2000) ** 2 // 8
            if TEMP < -1500:
                OFFi += 7 * (TEMP + 1500) ** 2
                SENSi += 4 * (TEMP + 1500) ** 2
        else:
            Ti = 2 * dT * dT // 2**37
            OFFi = (TEMP - 2000) ** 2 // 16
            SENSi = 0
        OFF -= OFFi
        SENS -= SENSi
        TEMP -= Ti
    P = (D1 * SENS // 2**21 - OFF) // 2**13
    return TEMP, P


for d1, d2 in [(4958179, 6815414), (4958179, 6400000), (4958179, 5200000),
               (4958179, 7300000), (5300000, 6700000)]:
    print(d1, d2, calc(d1, d2, False), calc(d1, d2, True))
