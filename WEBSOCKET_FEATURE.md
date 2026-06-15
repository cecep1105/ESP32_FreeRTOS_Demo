# Feature: backend notifications -> Pi marquee (ESP32 WebSocket client)

The ESP32 connects as a **WebSocket client** to your existing Python backend.
While connected it listens for pushed events (your MikroTik RouterOS netwatch
notifications, relayed by the backend) and forwards each one to the Pi as a
`msg`, so it scrolls across the MAX7219 + HDMI marquee.

ESP32-only change. The Pi needs nothing new — it already turns any `msg <text>`
into a scrolling message.

```
backend (Python WS server)  --push-->  ESP32 (WS client)  --UART "msg ..."-->  Pi marquee
        ^ MikroTik netwatch
```

## Configure (top of `ESP32S3_RaspberyPi/src/main.cpp`)

```c
#define WS_HOST    "192.168.88.10"   // your backend server IP / hostname
#define WS_PORT    8000              // your WS server port
#define WS_PATH    "/ws"             // your WS endpoint path
#define WS_USE_SSL 0                 // 1 = wss:// (TLS), 0 = ws:// (plain)
#define WS_HELLO   ""                // optional line sent on connect (subscribe/auth)
```

`platformio.ini` gains one dependency: `links2004/WebSockets@^2.7.3`.

## Message formats the ESP32 understands

Each WS **text frame** becomes one marquee line. Two accepted shapes:

1. **Plain text** — forwarded as-is.
   `Router1 is DOWN` -> marquee `Router1 is DOWN`

2. **JSON object** — if it has `message` / `msg` / `text`, that is used
   directly. Otherwise a netwatch-style line is built from
   `name` (or `host` / `address`) + `status` (or `state`) + `comment`:

   ```json
   {"host":"10.0.0.5","status":"down","comment":"WAN link"}
   ```
   -> marquee `NETWATCH 10.0.0.5 down WAN link`

   ```json
   {"message":"Office AP recovered at 14:03"}
   ```
   -> marquee `Office AP recovered at 14:03`

Text is sanitized (CR/LF/control chars -> space) and clamped to `MARQUEE_MAX`
(100 chars) to fit the Pi's `msg` buffer.

### Minimal backend push example (Python, `websockets`)

```python
import json
# on a netwatch event, send to each connected client:
await ws.send(json.dumps({"host": host, "status": status, "comment": comment}))
# or simply:
await ws.send(f"NETWATCH {host} {status}")
```

If your backend uses topics/rooms and expects the client to subscribe first,
set `WS_HELLO` to the line it wants (e.g. `{"subscribe":"netwatch"}`); it is
sent once on every (re)connect.

## Behavior

- Runs in its own FreeRTOS task (`websocketTask`, core 0), separate from
  RemoteXY and the NTP/prayer task. Uses the same WiFi RemoteXY brings up.
- Auto-reconnects every 5 s if the link drops; heartbeat ping every 15 s.
- Each event overwrites the marquee with the latest (it persists until the
  next event or a manual `msg`). If you'd rather it auto-revert to the clock
  message after N seconds, that's a small add — say the word.
- Debug: the USB serial console prints `[WS] connected/disconnected` and
  `[WS->PI] msg ...` for every forwarded notification.

## Files changed (ESP32 only)

| File | Change |
|------|--------|
| `ESP32S3_RaspberyPi/platformio.ini` | Added `links2004/WebSockets@^2.7.3`. |
| `ESP32S3_RaspberyPi/src/main.cpp` | WS includes + config; `webSocket` client; `forwardMarquee()`, `handleWsText()`, `webSocketEvent()`, `websocketTask`; task started in `setup()`. |
