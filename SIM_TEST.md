# Prove the logic view on HDMI from RemoteXY (no BS05U needed)

This runs the **entire** ESP32→UART→Pi→HDMI path using a synthetic, animated
8-channel source, triggered from the RemoteXY Message Text box. When your
mini-USB→Type-C cable arrives you swap the source for the real BS05U capture;
nothing else changes.

```
RemoteXY "la"  ──>  ESP32 laSimTask  ──UART "la begin/d/end"──>  Pi  ──>  HDMI
                    (la_sim_fill → bvm_emit_frame)                 (la_render)
```

No new wiring: it uses the same UART link the clock already uses
(ESP32 TX17 → Pi RX GPIO15, 115200).

## ESP32 (stays on `framework = arduino` — no USB components yet)

1. Copy into `src/`:  `bvm.h`, `bvm.cpp`, `bs05_logic.h`, `la_sim.h`, `la_sim.c`
   (and use the updated `main.cpp`).
   - **Do NOT** add `bs05_logic.cpp` or `esp_usb_vcp.cpp` yet — those need the
     `arduino, espidf` framework + USB components and are only for the real BS05U.
   - `bs05_logic.h` is header-only here (just supplies the `BS_*` tunables).
2. `pio run -t upload`.

## Pi 3 / 400

1. Copy `la_render.c`, `la_render.h` into `RaspberryPi3_400-FreeRTOS/drivers/`.
2. In `RaspberryPi3_400-FreeRTOS/Makefile`, after the qrcodegen line:
   ```
   OBJS +=build/qrcodegen.o
   OBJS +=build/la_render.o      # <-- add
   ```
3. `make BOARD=pi3` (or `pi400`), flash.

## Pi 1

1. Copy `la_render.c`, `la_render.h` into the Pi 1 `drivers/` dir
   (`.../Demo/ARM6_BCM2835/drivers/`). It auto-globs `drivers/*.c` — no Makefile
   change. (Pi 1's HDMI is the same `fb_*` API, so the view works there too.)

## Run

1. Open the RemoteXY app, go to the **Message Text** box.
2. Type `la`, press **Send** → HDMI switches to the live 8-lane logic view,
   visibly scrolling at ~4 fps.
3. Type `la off`, press **Send** → back to the clock dashboard.

The ESP32 USB console prints `[LA] on` / `[LA] off` for each trigger, and the Pi
acks only `ok la off` (data frames are silent to keep the link clean).

### Quick Pi-only check (optional, no ESP32)

From a serial terminal straight into the Pi you can paste:
```
la begin 8 8 1000000
la d 0 ff00ff00ff00ff00
la end
```
Eight short lanes should appear; `la off` returns to the clock.

## When the BS05U cable arrives (switch sim → real)

1. Switch the PlatformIO env to `framework = arduino, espidf`, add
   `src/idf_component.yml` (see ESP_USB_SETUP.md), and add `bs05_logic.cpp` +
   `esp_usb_vcp.cpp` to `src/`.
2. In `main.cpp`, give the BS05U its own source: keep `laSimTask` for "la",
   and either replace its `la_sim_fill(...)` with `bs05_capture(...)`, or add a
   `bs05_begin(&vcp)` + the `bs05_task` and route a new `la real` command to it.
   The wire format, the Pi side, and the renderer are identical — only the
   sample *source* differs.

## Files (this step)

`main.cpp` (ESP32, updated) · `main.c` (Pi3/400, updated) ·
`la_sim.h`/`la_sim.c` (synthetic source) · plus the already-delivered
`bvm.*`, `bs05_logic.h`, `la_render.*`.
