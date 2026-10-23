# pitipiwpiw_wiw_wiw — мелководный ТНПА с надводным буем

Телеуправляемый подводный аппарат (до 5 м). Управление идёт с телефона через WebApp, связь — через надводный буй и 10-метровый кабель.

- `src/rov_core/` — главный контроллер ТНПА (ESP32): движители, датчики, fail-safe
- `src/buoy/` — буй (ESP32): Wi-Fi AP, WebSocket ↔ кабель, видео
- `src/cam/` — курсовая камера (ESP32-CAM): JPEG по кабелю
- `data/index.html` — WebApp для телефона
- `docs/ICD.md` — все интерфейсы и протоколы; `docs/decisions.md` — решения
- `CLAUDE.md`, `docs/agents/` — правила и роли для агентов разработки

```bash
pio run -e rov_core -e buoy -e cam    # сборка всех узлов
pio run -e buoy -t uploadfs           # загрузка WebApp в буй
```
