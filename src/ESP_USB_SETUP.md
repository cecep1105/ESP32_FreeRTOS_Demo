# ESP32-S3 USB-host setup (esp-usb VCP route)

Wires `EspUsbVcp` (esp_usb_vcp.cpp) into the project. This is the only part of
the logic-analyzer feature that needs the hardware; the capture protocol and the
whole ESP32→Pi data path are already host-validated.

## 1. PlatformIO env — Arduino *as an IDF component*

The esp-usb VCP drivers are ESP-IDF managed components, so the env must be the
dual `arduino, espidf` framework (your Arduino code — RemoteXY, WiFiManager,
WebSockets — keeps working unchanged):

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32
board = esp32-s3-devkitc-1
framework = arduino, espidf          ; <-- was: arduino
upload_port = COM14
build_flags =
    -DARDUINO_USB_CDC_ON_BOOT=0       ; keep native USB-OTG free for HOST, not device-CDC
board_build.embed_files =
lib_deps =
    tzapu/WiFiManager@^2.0.17
    remotexy/RemoteXY@^4.1.10
    bblanchon/ArduinoJson@^7.2.2
    links2004/WebSockets@^2.7.3
```

With `arduino, espidf`, Arduino provides `app_main()` and still calls your
`setup()`/`loop()` — put the USB-host init in `setup()` as before.

## 2. Managed components

Create **`src/idf_component.yml`**:

```yaml
dependencies:
  espressif/usb_host_vcp: "^1.0.0"      # pulls cdc_acm + the VCP service
  espressif/usb_host_ftdi: "^2.0.0"     # FT23x driver (the BS05U's chip)
  # espressif/usb_host_cp210x: "^2.0.0" # optional, other bridges
  # espressif/usb_host_ch34x:  "^2.0.0"
```

PlatformIO resolves these from the Espressif Component Registry on first build.
(Pin to the latest versions the registry shows; `usb_host_ftdi` provides `FT23x`.)

## 3. sdkconfig

Make sure USB-OTG host is enabled (defaults are usually fine on S3). If you keep
a `sdkconfig.defaults`, add:

```
CONFIG_USB_OTG_SUPPORTED=y
```

## 4. main.cpp wiring

```c
#include "bvm.h"
#include "bs05_logic.h"
#include "esp_usb_vcp.h"

static EspUsbVcp vcp;

static void piSendLine(const char *l){
    xSemaphoreTake(g_uartMux, portMAX_DELAY);
    Serial1.print(l); Serial1.print("\r\n");
    xSemaphoreGive(g_uartMux);
}

// in setup(), after Serial1.begin(...):
bs05_begin(&vcp);                                  // installs host + starts connect task
xTaskCreatePinnedToCore(bs05_task, "bs05", 4096, (void*)piSendLine, 1, NULL, 1);

// RemoteXY button (edge) toggles the live capture:
if (button_edge) {
    if (!bs05_running()) bs05_start();
    else { bs05_stop(); piSendLine("la off"); }
}

// serial command from terminal/Pi:
if (!strcmp(line,"la on"))  bs05_start();
if (!strcmp(line,"la off")){ bs05_stop(); piSendLine("la off"); }
```

## 5. Hardware wiring (important)

```
BS05U  D+  ── GPIO20 (USB_D+)   ESP32-S3 native USB port
BS05U  D-  ── GPIO19 (USB_D-)
BS05U  VBUS ── +5V   <-- the S3 must SUPPLY 5V; the BS05U is bus-powered
BS05U  GND ── GND
```

The DevKitC-1's native USB connector is wired as a *device* port and does **not**
source VBUS to a downstream device. For host mode you must feed +5 V to the
BS05U's VBUS from the board's 5V rail (and share ground). Without VBUS the device
never enumerates — this is the #1 bring-up gotcha.

## 6. Bring-up order

1. `pio run -t upload`, open the monitor. You should see `waiting for BS05U...`.
2. Plug in the BS05U (with VBUS supplied). Expect `BS05U connected @ 1000000 baud`.
   If it never connects, drop `BS_BAUD` to `115200` in `bs05_logic.h`.
3. Tap the RemoteXY button → the Pi HDMI should switch to the live logic view.
4. If grabs are empty/garbled: the BVM device should be in **non-echo** mode after
   `!` (so command letters like `D`/`U` aren't echoed back into the reply stream);
   confirm reply framing in `vcp_read_hex_token` (CR-terminated hex) on your unit.

## Files

`esp_usb_vcp.h/.cpp` — the IVcp backend (this route). Drop into `src/`.
Everything else (`bvm.*`, `bs05_logic.*`, Pi `la_render.*`) is unchanged.
