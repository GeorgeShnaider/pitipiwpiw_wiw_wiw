# Перепрошивка ESC в режим Bidirectional (BLHeli_S)

Для кого: команда, которая готовит ESC Cyclone 35A BLHeli_S к стенду.

## Зачем
По умолчанию BLHeli_S крутит мотор только в одну сторону: 1000 мкс — стоп, 2000 мкс — полный газ. ТНПА нужен реверс (назад, разворот на месте, вниз). В режиме **Bidirectional** середина диапазона означает стоп, выше — вперёд, ниже — назад. Прошивка ROV_CORE рассчитана именно на это: 1500 мкс — стоп (ICD §9).

Сама прошивка ESC не меняется — меняются **настройки** BLHeli_S, которые хранятся в ESC. Это делается для каждого из 6 ESC по отдельности (или для 4 установленных, резервные — потом).

## Что понадобится (один из вариантов)

| Вариант | Компьютер | Интерфейс | Программа |
|---|---|---|---|
| **А (самый проверенный)** | Windows | Arduino Nano или Uno | BLHeliSuite 16.x (версия для BLHeli_S) |
| Б | Любой, в т.ч. Mac | Полётный контроллер с Betaflight | esc-configurator.com в Chrome/Edge |
| В | Любой, в т.ч. Mac | Arduino Nano со скетчем-passthrough | esc-configurator.com в Chrome/Edge |

Если в команде есть Windows-ноутбук и Arduino — вариант А. Вариант В на Mac возможен, но лид его не проверял.

Также нужны: батарея 4S (ESC программируется **под питанием**), провода-перемычки, **моторы без винтов**.

## Вариант А: Windows + Arduino + BLHeliSuite

1. Скачать **BLHeliSuite 16.x** (именно для BLHeli_S; BLHeliSuite32 — для другой прошивки и не подходит).
2. Превратить Arduino в интерфейс: в BLHeliSuite меню **Make interfaces → Arduino 4way-interface** → выбрать плату (Nano/Uno) и COM-порт → прошить.
3. Подключение (батарея пока **отключена**):
   - сигнальный провод ESC (белый/жёлтый) → пин **D3** Arduino;
   - минус ESC (чёрный провод сигнального разъёма или минус силовых проводов) → **GND** Arduino;
   - **красный провод сигнального разъёма (если есть, это BEC 5 В) не подключать** к Arduino;
   - мотор подключён к ESC, **винт снят**.
4. Arduino — к компьютеру по USB. В BLHeliSuite слева внизу выбрать интерфейс **«SILABS BLHeli Bootloader (4way-if)»** и COM-порт Arduino, нажать **Connect**.
5. Подключить батарею к ESC. Нажать **Read Setup**. Должны появиться настройки и тип ESC.
6. Изменить:
   - **Motor Direction → Bidirectional**;
   - **PPM Center Throttle → 1500** (по умолчанию 1488 — при нашем стопе 1500 мотор может еле заметно крутиться вперёд);
   - **PPM Min Throttle** и **PPM Max Throttle** — выставить симметрично вокруг 1500, как можно шире из того, что разрешает программа (ориентир: Min 1100, Max 1900). **Запишите, какие значения получились** — лид пропишет их в `config.h`, чтобы полный ход прошивки совпал с полным ходом ESC;
   - остальное не трогать.
7. **Write Setup**. Затем **Read Setup** ещё раз — убедиться, что сохранилось.
8. Отключить батарею, перейти к следующему ESC.

## Вариант Б: полётный контроллер + esc-configurator.com
1. Подключить сигнальные провода ESC к выходам моторов полётного контроллера с Betaflight, общая земля, батарея к ESC.
2. Открыть https://esc-configurator.com в Chrome/Edge, **Select Port** → порт контроллера → **Connect** → **Read Setup**.
3. Настройки — как в п.6 варианта А (названия: Motor Direction, PPM Center/Min/Max Throttle). **Write Setup**.

## Как проверить, что получилось
После перепрошивки ESC при подаче питания и 1500 мкс на сигнале пищит и **не крутится**. На стенде (этап 7) с ROV_CORE: при armed и surge вперёд маршевые крутятся в одну сторону, назад — в другую. Если мотор крутится «не в ту сторону» — это правится не в ESC, а в `MOTOR_DIR` в `config.h` (или перекидыванием двух любых фаз мотора).

## Безопасность
- **Винты сняты** всё время, пока идёт программирование и стенд.
- ESC и мотор могут резко дёрнуться при записи настроек — держите мотор закреплённым.
- Не подключайте красный (5 В) провод ESC к Arduino или ESP32.

## Что сообщить лиду после перепрошивки
1. Получилось ли Bidirectional на всех ESC.
2. Итоговые **PPM Min / Center / Max Throttle** (одинаковые на всех ESC).
3. Версию прошивки, которую показал Read Setup (например, «BLHeli_S 16.7»).

Источники: [BLHeli_S manual SiLabs Rev16.x](https://bluerobotics.com/wp-content/uploads/2018/10/BLHeli_S-manual-SiLabs-Rev16.x.pdf), [Bluejay Settings wiki](https://github.com/mathiasvr/bluejay/wiki/Settings), [BLHeli_S Firmware Knowledge Base](https://github.com/blheli-configurator/blheli-configurator/wiki/BLHeli_S-Firmware-Knowledge-Base), [Arduino Nano passthrough для esc-configurator](https://zff.dev/fluxin/blheli_passthrough).
