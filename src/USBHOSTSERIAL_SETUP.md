# BS05U USB host via USBHostSerial (single Arduino framework)

Replaces the esp-usb / dual-framework route in ESP_USB_SETUP.md. You stay on
`framework = arduino`, so all your WiFi/BLE libraries keep working with none of
the sdkconfig/FREERTOS_HZ/WiFiClientSecure pain.

## platformio.ini

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32                 ; see "platform note" if USB headers are missing
board    = esp32-s3-devkitc-1
framework = arduino                    ; SINGLE framework
upload_port = COM14
monitor_speed = 115200
build_flags =
    -DARDUINO_USB_CDC_ON_BOOT=0        ; keep native USB-OTG free for HOST mode
    -DUSBHOSTSERIAL_BUFFERSIZE=2048    ; RX/TX rings big enough for a logic dump
board_build.partitions = huge_app.csv ; BLE + WiFi + USB host won't fit the default
lib_deps =
    tzapu/WiFiManager
    remotexy/RemoteXY
    bblanchon/ArduinoJson
    links2004/WebSockets
    https://github.com/bertmelis/USBHostSerial.git#3608e9a9e2680a2001f82b91421c8830399ebf43  ; pin to commit (no v0.2.0 tag exists)
```

**Platform note (important):** USB-host needs Arduino core 3.x (IDF 5.x). The
stock `platform = espressif32` is pinned to the old core 2.0.17 / IDF 4.4.7
(compiles at gnu++11), so `USBHostSerial`'s headers fail to compile there
(e.g. `vcp.hpp` uses `auto` parameters). Use the pioarduino fork instead —
replace the platform line with:

    platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip

Keep `framework = arduino` and everything else. Note: core 2.x -> 3.x is a major
bump, so a few APIs changed; your existing libraries work, but watch for a
deprecation or changed signature in your own code after switching.

## Files in src/

Add (now compile fine under single framework — `USBHostSerial` supplies the USB
headers, and `bs05_logic.cpp` is transport-agnostic):

- `esp_usb_vcp.cpp` / `esp_usb_vcp.h`  (USBHostSerial backend — new versions)
- `bs05_logic.cpp` / `bs05_logic.h`    (unchanged)
- `bvm.cpp` / `bvm.h`                  (unchanged)

Delete from the project: the old `idf_component.yml` and `sdkconfig.defaults`
(only needed by the dual framework).

## main.cpp: switch the source from sim to real capture

You already have `laSimTask` driving the view from `la_sim_fill()`. To use the
BS05U, register the backend and run the real capture task instead:

```c
#include "esp_usb_vcp.h"
static EspUsbVcp vcp;

// in setup(), after Serial1.begin(...):
bs05_begin(&vcp);                                            // starts USB host (manages hot-plug)
xTaskCreatePinnedToCore(bs05_task, "bs05", 4096, (void*)piSendRaw, 1, NULL, 1);
// (replace the laSimTask line, or keep both and gate which one runs)

// point the "la" command at the real capture instead of the sim:
//   "la"     -> bs05_start();
//   "la off" -> bs05_stop(); piSendRaw("la off");
```

`bs05_task` runs the BVM capture (`set regs -> > -> U -> D -> poll -> dump`),
packs to columns, and emits the same `la begin/d/end` frames — so the Pi side
and the renderer you tuned are untouched. Keep `laSimTask` around (e.g. behind a
`la sim` command) for testing without the device.

## Wiring (unchanged, and easy to get wrong)

```
BS05U  D+  -> GPIO20 (USB_D+)    ESP32-S3 native USB port
BS05U  D-  -> GPIO19 (USB_D-)
BS05U  VBUS-> +5V   <-- the S3 must SUPPLY 5V; the BS05U is bus-powered
BS05U  GND -> GND
```
The DevKitC-1's native USB connector won't source VBUS to a downstream device on
its own — feed +5V to the BS05U's VBUS from the board's 5V rail. No VBUS = the
device never enumerates (the #1 bring-up dead end).

## Bring-up order

1. `pio run -t upload`, open the monitor.
2. Plug in the BS05U (with VBUS). USBHostSerial logs the connection; note the
   FTDI **PID** it reports — you can then pin `USBHostSerial(0x0403, <pid>)` in
   esp_usb_vcp.cpp to bind only the BitScope.
3. If it won't sync at `BS_BAUD = 1000000`, drop it to `115200` in bs05_logic.h.
4. Send `la` -> the live logic trace should appear on HDMI.

## Hardware-confirm checklist

- VBUS wired (above).
- FTDI driver matches the BS05U's chip (the lib registers FT23x; FT232R should
  work — confirm at connect).
- `BS_BAUD` actually syncs (try 1M, fall back 115200).
- BVM device is in non-echo mode after `!` so command letters (`D`,`U`) don't
  leak into the reply stream (see bs05_logic.cpp `[HW]` notes).
