# Feature: BitScope BS05U logic analyzer → HDMI (ESP32-S3 USB host bridge)

```
BS05U ──FTDI(0403:6001)──> ESP32-S3 (native USB host, GPIO19/20)
                            │  BitScope VM: ! · set-regs · > · U · D · (poll) · > · A · read N
                            │  decimate → 240 columns (1 byte = 8 ch) → wire frame
                            ▼
                        UART1 TX17 @115200 ──"la …" lines──> Pi RX(GPIO15)
                                                              reassemble → vHdmi paints 8 lanes
```

The BS05U is an FTDI serial device; the hard work is the **BitScope VM** capture
protocol, ported from the BS05-tested `scopething` reference. The whole data path
(BVM command builder → sample pack → wire frame → Pi decode → render) is
**host-validated**: register grammar is byte-exact and the frame round-trips with
zero loss. Only the ESP32 USB-host bring-up needs a hardware pass (seams marked
`[HW]`).

## Wire protocol (ESP32 → Pi, each line < 128 B, no acks on data)

```
la begin <ncols> <nch> <rate_hz>     start a frame
la d <col_off> <hex>                 chunk: 2 hex chars/col, bit i = channel i (≤56 cols/line)
la end                               frame complete → repaint
la off                               leave logic view, back to clock
```

At 240 cols / 4 fps this is ≈ 3.4 KB/s — about 30% of the 115200 link, which is
otherwise idle while the clock GUI is hidden.

## Files (new)

| File | Side | Role |
|------|------|------|
| `drivers/la_render.c`, `la_render.h` | Pi | renderer + frame decoder (fb_* only). **Validated.** |
| `src/bvm.h`, `bvm.cpp` | ESP32 | BVM helpers: register builder, capture plan, packer, framer. **Validated.** |
| `src/bs05_logic.h`, `bs05_logic.cpp` | ESP32 | USB-host capture task (BVM sequence over an `IVcp` seam). |

## Pi integration (mirrors the QR wiring exactly)

1. Copy `la_render.c`/`.h` into both Pi `drivers/` dirs.
2. Pi3/400 `Makefile`: `OBJS += build/la_render.o`  (Pi1 auto-globs `drivers/*.c`).
3. In each `main.c`:

```c
#include "la_render.h"

static la_frame_t g_la;
volatile int g_la_req = 0;   /* 1 = frame ready, 2 = off */
volatile int g_la_on  = 0;

/* in parse_line(), alongside the qr branch — NO acks on begin/d/end */
} else if (seq(cw,"la")) {
    skip_sp(&p);
    if      (seq(p,"off"))   { g_la_req = 2; uart_puts("ok la off\r\n"); }
    else if (!strncmp(p,"begin ",6)) { int nc,ch; unsigned r;
        /* parse: begin <ncols> <nch> <rate> */
        const char *q=p+6; nc=parse_uint(&q); skip_sp(&q); ch=parse_uint(&q); skip_sp(&q); r=parse_uint(&q);
        la_begin(&g_la, nc, ch, r); }
    else if (!strncmp(p,"d ",2))     { const char *q=p+2; int off=parse_uint(&q); skip_sp(&q);
        la_chunk(&g_la, off, q); }
    else if (seq(p,"end"))   { g_la.ready = 1; g_la_req = 1; }
}
```

4. In `vHdmi`, next to the QR handling:

```c
int la_rq = g_la_req;
if (la_rq) {
    g_la_req = 0;
    if (la_rq == 1) {                     /* frame ready */
        if (!g_la_on) { la_logic_chrome(&g_la); g_la_on = 1; }  /* enter on 1st frame */
        la_logic_traces(&g_la);           /* repaint lane bands only -> no flicker */
    } else if (la_rq == 2) {              /* leave logic view */
        g_la_on = 0; fb_clock_reset(); prev[0] = 0; prev_sweep = -2; fb_marquee_set(g_msg);
    }
}
if (g_la_on) { vTaskDelay(pdMS_TO_TICKS(20)); continue; }   /* logic owns the screen */
```

(Place the `if(g_la_on) continue;` guard with the existing full-screen-QR guard so
the two views are mutually exclusive.)

## ESP32 integration

1. `platformio.ini` → add a USB-host VCP library, e.g. `bertmelis/USBHostSerial`
   (Arduino/pioarduino, bundles the FTDI VCP driver).
2. Add `bvm.cpp` + `bs05_logic.cpp` to `src/`.
3. In `main.cpp`:

```c
#include "bvm.h"
#include "bs05_logic.h"

/* reuse the existing UART1 mutex that guards Serial1 to the Pi */
static void piSendLine(const char *l){
    xSemaphoreTake(g_uartMux, portMAX_DELAY);
    Serial1.print(l); Serial1.print("\r\n");
    xSemaphoreGive(g_uartMux);
}

/* --- bind IVcp to your USB-host library [HW: this class is the seam] --- */
class EspUsbVcp : public IVcp { /* wrap USBHostSerial open/read/write here */ };
static EspUsbVcp vcp;

/* in setup(): */
bs05_begin(&vcp);
xTaskCreatePinnedToCore(bs05_task, "bs05", 4096, (void*)piSendLine, 1, NULL, 1);

/* RemoteXY button (toggle live capture): */
if (button_pressed_edge) {
    if (!bs05_running()) bs05_start();
    else { bs05_stop(); piSendLine("la off"); }
}

/* serial command from terminal/Pi: "la on" / "la off" */
if (!strcmp(line,"la on"))  bs05_start();
if (!strcmp(line,"la off")){ bs05_stop(); piSendLine("la off"); }
```

The ESP32 free-runs at `BS_FPS`, emitting one `la begin/d/end` frame per grab while
running; stopping sends a single `la off` so the Pi reverts to the clock.

## Build / flash / verify

1. **Pi**: `make BOARD=pi3` (or `pi400`) and the Pi1 build; flash as usual.
2. **ESP32**: `pio run -t upload`.
3. Smoke test the Pi half *without hardware*: from a serial terminal send
   `la begin 8 8 1000000`, `la d 0 ff00ff00ff00ff00`, `la end` — eight short lanes
   should appear; `la off` returns to the clock.
4. Plug BS05U into the S3's **native USB** port (not the UART/debug port), tap the
   RemoteXY button, and the live trace should stream.

## Hardware-confirm checklist (the `[HW]` seams)

- **`BS_BAUD`** (bs05_logic.h): FT232R link rate. Try `1000000`; if the device
  doesn't respond to `!`, fall back to `115200`.
- **`EspUsbVcp`**: wire `begin/connected/setBaud/read/write` to your VCP library;
  match VID 0x0403 / PID 0x6001.
- **BVM reply framing** (`vcp_read_hex_token`): confirm the status/address replies
  are CR-terminated hex on your unit; adjust the terminator if needed.
- **Free-run trigger**: `TriggerMask=0` is used for immediate capture. If grabs
  stall, set a small non-zero `Timeout` (already 1000) or a real edge trigger.

## Tuning (bs05_logic.h)

`BS_NCOLS` (display width), `BS_NSAMPLES` (capture depth ≤12288), `BS_RATE_HZ`
(40 MHz/ticks → timebase), `BS_FPS` (2–5), `BS_DIGITAL` (channel enable mask).
