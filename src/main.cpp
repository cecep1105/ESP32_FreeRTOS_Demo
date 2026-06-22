#include <Arduino.h>

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

//#define REMOTEXY__DEBUGLOG

#define REMOTEXY_MODE__ESP32CORE_BLE        // RemoteXY over Bluetooth LE (frees WiFi for WiFiManager)
#define REMOTEXY_BLUETOOTH_NAME "LogicLA"   // name shown in the RemoteXY app's BLE device list
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiManager.h>                     // tzapu/WiFiManager: pick a hotspot at runtime
#include <BLEDevice.h>                        // required by RemoteXY ESP32 BLE mode
#include <RemoteXY.h>
#include <time.h>
#include <stdarg.h>
#include <ctype.h>
#include <string.h>
#include <math.h>                             // powf() for the timebase-knob rate map
#include <ArduinoJson.h>
#include <WebSocketsClient.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// ---- Logic-analyzer feature (BitScope BS05U -> HDMI) -----------------------
#include "bvm.h"          // protocol helpers (pack + wire framing)
#include "bs05_logic.h"   // BS_* tunables (+ real capture, added later)
#include "la_sim.h"       // synthetic source: prove HDMI path without the BS05U
#include "siggen.h"       // A0..A3 test-signal generator (square / PWM-sine)

#include "esp_usb_vcp.h"
static EspUsbVcp vcp;

/* Live capture rates — defined in bs05_logic.cpp and driven by the RemoteXY
 * timebase knobs. Declared here too so this translation unit compiles even if
 * bs05_logic.h on disk predates the timebase feature. */
extern volatile uint32_t g_la_rate_hz;
extern volatile uint32_t g_sc_rate_hz;







// RemoteXY connection settings
// RemoteXY connection settings — UNUSED in BLE mode (kept for reference only).
// WiFi is now provisioned by WiFiManager, not RemoteXY.
//#define REMOTEXY_WIFI_SSID     "wifissid"
//#define REMOTEXY_WIFI_PASSWORD "wifipassword"
//#define REMOTEXY_SERVER_PORT   6377

// Time zone. Jakarta = UTC+7, no DST.
#define TZ_OFFSET_SEC   (7 * 3600)
#define DST_OFFSET_SEC  0

// Aladhan API location & calculation method (20 = Kemenag Indonesia)
#define PRAYER_CITY     "Jakarta"
#define PRAYER_COUNTRY  "Indonesia"
#define PRAYER_METHOD   "20"

// Serial link to the Pi clock (UART1). ESP32 TX(17)->Pi RX(GPIO15),
// ESP32 RX(18)<-Pi TX(GPIO14), common GND, 115200 8N1.
#define PI_RX_PIN  18
#define PI_TX_PIN  17
#define PI_BAUD    115200

// ---- WebSocket client: backend notifications -> Pi marquee -----------------
// The ESP32 connects as a WS client to your existing Python backend. The
// backend receives MikroTik RouterOS netwatch events and pushes them to its
// connected clients; each text frame here becomes a scrolling message on the
// Pi via the existing "msg" command. Plain ws:// by default; flip WS_USE_SSL
// to 1 for wss:// (TLS).

#define WS_HOST      "172.16.10.36" //your backend server IP / hostname
#define WS_PORT      4001          // <-- your WS server port
#define WS_PATH      "/ws/netmon/?timeid=1234567890"    // your WS endpoint path
#define WS_USE_SSL   0                 // 1 = wss:// (TLS), 0 = ws:// (plain)
#define WS_HELLO     "RPI"              // optional line sent on connect (subscribe/auth); "" = none
#define MARQUEE_MAX  100               // clamp forwarded text to the Pi "msg" limit

// RemoteXY GUI configuration
#pragma pack(push, 1)  
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 701 bytes V21 
  { 254,230,0,0,0,1,0,0,0,178,2,21,0,0,0,0,31,1,106,200,
  3,1,0,0,16,0,130,3,3,100,53,11,107,130,3,62,100,71,11,107,
  2,31,19,53,16,0,2,26,31,31,82,85,78,0,83,84,79,80,0,2,
  12,85,16,16,1,2,26,31,31,49,0,79,70,70,0,2,36,85,16,16,
  1,2,26,31,31,50,0,79,70,70,0,2,59,85,16,16,1,2,26,31,
  31,51,0,79,70,70,0,2,82,85,16,16,1,2,26,31,31,52,0,79,
  70,70,0,2,12,103,16,16,1,2,26,31,31,53,0,79,70,70,0,2,
  36,103,16,16,1,2,26,31,31,54,0,79,70,70,0,2,59,103,16,16,
  1,2,26,31,31,55,0,79,70,70,0,2,82,103,16,16,1,2,26,31,
  31,56,0,79,70,70,0,4,8,41,93,9,128,2,26,129,27,7,58,11,
  64,1,76,69,68,32,83,87,69,69,80,0,129,23,71,66,12,64,1,76,
  69,68,32,79,78,124,79,70,70,0,131,6,182,94,12,1,6,2,31,78,
  101,120,116,0,6,193,12,146,20,20,2,1,14,0,130,4,122,100,49,11,
  17,130,1,8,103,101,11,17,7,6,26,93,12,100,0,2,26,2,201,1,
  27,42,46,18,1,2,31,83,69,78,68,32,84,69,88,84,0,7,57,137,
  42,10,118,64,2,26,2,1,16,154,74,10,1,2,31,83,69,84,32,84,
  73,77,69,0,129,21,125,34,8,64,6,67,108,111,99,107,32,83,101,116,
  0,131,3,185,37,12,1,6,2,31,80,82,69,86,0,41,131,66,185,37,
  12,1,6,2,31,78,69,88,84,0,26,129,34,74,35,9,64,35,81,82,
  67,79,68,69,0,2,6,87,42,11,0,2,26,31,31,83,72,79,87,0,
  72,73,68,69,0,2,58,87,39,11,0,2,26,31,31,66,73,71,0,83,
  77,65,76,76,0,129,14,13,77,9,64,8,67,77,68,32,79,82,32,77,
  69,83,83,65,71,69,0,7,7,137,43,10,118,64,2,26,2,19,0,130,
  4,6,99,78,11,17,129,23,13,11,9,64,35,76,65,0,2,13,25,28,
  8,0,2,26,31,31,79,78,0,79,70,70,0,129,65,14,29,9,64,35,
  83,67,79,80,69,0,2,67,25,27,8,0,2,26,31,31,79,78,0,79,
  70,70,0,131,34,185,37,12,1,6,2,31,80,82,69,86,0,38,12,67,
  39,27,8,193,30,26,65,66,0,65,0,66,0,4,58,58,41,8,128,2,
  26,4,9,58,41,8,128,2,26,130,4,90,97,42,11,105,2,14,106,29,
  13,0,2,26,31,31,79,78,0,79,70,70,0,4,16,122,77,7,128,2,
  26,2,68,106,26,12,0,2,26,31,31,82,0,76,0,129,27,93,55,8,
  64,1,83,69,82,86,79,32,77,79,84,79,82,0,130,9,137,87,43,11,
  107,2,14,149,26,13,0,2,26,31,31,79,78,0,79,70,70,0,4,15,
  165,78,9,128,2,26,2,64,149,26,12,0,2,26,31,31,82,0,76,0,
  129,20,140,64,8,64,1,83,84,69,80,80,69,82,32,77,79,84,79,82,
  0 };
  
// this structure defines all the variables and events of your control interface 
struct {

    // input variables
  uint8_t switch1; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led1; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led2; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led3; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led4; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led5; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led6; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led7; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t led8; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t slider1; // from 0 to 100
  char msgText[201]; // string UTF8 end zero
  uint8_t buttonMsg; // =1 if button pressed, else =0, from 0 to 1
  int16_t minval; // -32768 .. +32767
  uint8_t settime; // =1 if button pressed, else =0, from 0 to 1
  uint8_t qr_switch; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t qrmode; // =1 if switch ON and =0 if OFF, from 0 to 1
  int16_t hourval; // -32768 .. +32767
  uint8_t laSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t scSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t scChan; // from 0 to 3
  int8_t scTimebase; // from 0 to 100
  int8_t laTimebase; // from 0 to 100
  uint8_t servoSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t servoSpeed; // from 0 to 100
  uint8_t servoDir; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t stepSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t stepSpeed; // from 0 to 100
  uint8_t stepDir; // =1 if switch ON and =0 if OFF, from 0 to 1

    // complex variables
  RemoteXYType_RealTime realTime_01; // use .getTimeStamp(), .getTime()

} RemoteXY;   
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

void RemoteXY_msgText_event() {
  // TODO: RemoteXY.msgText was changed
}


// Guards the single UART1 link to the Pi (shared by both tasks).
static SemaphoreHandle_t serialMutex;

/* ---- one text command per call, mutex-guarded ------------------------------
 * The Pi clock speaks a plain line protocol (CR/LF terminated):
 *   set HH:MM:SS | run | stop | speed <ms> | leds <mask> |
 *   msg <text> | bright <0-15> | prayer <idx> <NAME> <HH:MM> | machine on
 * ---------------------------------------------------------------------------*/
static void piCmd(const char *fmt, ...) {
    char buf[112];
    va_list ap; va_start(ap, fmt);
    vsnprintf(buf, sizeof buf, fmt, ap);
    va_end(ap);
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
        Serial1.print(buf);
        Serial1.print("\r\n");
        xSemaphoreGive(serialMutex);
    }
}


// Send an arbitrary line to the Pi, mutex-guarded, no printf formatting.
// void piSendRaw(const char *s) {
//     if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
//         Serial1.print(s);
//         Serial1.print("\r\n");
//         xSemaphoreGive(serialMutex);
//     }
// }

void piSendRaw(const char *s) {
    if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {   // was pdMS_TO_TICKS(100)
        Serial1.print(s);
        Serial1.print("\r\n");
        xSemaphoreGive(serialMutex);
    }
}




// ---- Logic-analyzer streaming source ---------------------------------------
// Streams 8-channel logic frames to the Pi using the same "la begin/d/end"
// line protocol the HDMI logic view consumes. Right now the source is the
// synthetic generator (la_sim_fill) so the whole path can be proven from
// RemoteXY with no BS05U attached. When the cable arrives, replace the
// la_sim_fill() call with bs05_capture() (esp-usb backend) and keep the rest.
// Trigger: Message Text box -> "la" starts, "la off" stops.
static volatile bool g_laRun = false;
static volatile bool g_wifiReconfig = false;   // set by "wifi" cmd -> reopen WiFiManager portal
static void laEmit(const char *line, void *ctx) { (void)ctx; piSendRaw(line); }

static void laSimTask(void *pv) {
    (void)pv;
    static uint8_t samples[BS_NSAMPLES];
    static uint8_t col[256];
    uint32_t frame = 0;
    const TickType_t period = pdMS_TO_TICKS(1000 / (BS_FPS < 1 ? 1 : BS_FPS));
    for (;;) {
        if (!g_laRun) { vTaskDelay(pdMS_TO_TICKS(80)); continue; }
        la_sim_fill(samples, BS_NSAMPLES, frame++);
        int nc = bvm_pack_columns(samples, BS_NSAMPLES, col, BS_NCOLS);
        bvm_emit_frame(col, nc, 8, g_la_rate_hz, laEmit, nullptr);
        vTaskDelay(period);
    }
}


// Pull "HH:MM" that follows a "key":" in the JSON body; returns minutes or -1.
static int findTime(const String &body, const char *key) {
    int k = body.indexOf(String("\"") + key + "\":\"");
    if (k < 0) return -1;
    k += strlen(key) + 4;
    if (k + 5 > (int)body.length()) return -1;
    int hh = (body[k] - '0') * 10 + (body[k + 1] - '0');
    int mm = (body[k + 3] - '0') * 10 + (body[k + 4] - '0');
    if (hh < 0 || hh > 23 || mm < 0 || mm > 59) return -1;
    return hh * 60 + mm;
}

/* Fetch the five daily prayers and push them to the Pi as
 * "prayer <idx> <NAME> <HH:MM>". The Pi does the <=10-min auto-announce. */
static void fetchPrayerTimes(void) {
    WiFiClient client;
    HTTPClient http;
    String url = String("http://api.aladhan.com/v1/timingsByCity?city=")
               + PRAYER_CITY + "&country=" + PRAYER_COUNTRY + "&method=" + PRAYER_METHOD;
    if (!http.begin(client, url)) return;

    int  mins[5] = { -1, -1, -1, -1, -1 };
    bool any = false;
    if (http.GET() == 200) {
        String body = http.getString();
        const char *keys[5] = { "Fajr", "Dhuhr", "Asr", "Maghrib", "Isha" };
        for (int i = 0; i < 5; i++) {
            mins[i] = findTime(body, keys[i]);
            if (mins[i] >= 0) any = true;
        }
    }
    http.end();
    if (!any) return;

    const char *names[5] = { "FAJR", "DHUHR", "ASR", "MAGHRIB", "ISHA" };
    for (int i = 0; i < 5; i++)
        if (mins[i] >= 0)
            piCmd("prayer %d %s %02d:%02d", i, names[i], mins[i] / 60, mins[i] % 60);
}


/* ---------------------------------------------------------------------------
 * Single command dispatcher. The RemoteXY "Message" button AND the laSwitch/
 * scSwitch shortcuts all call this, so flipping a switch behaves exactly like
 * typing the command into the message box. Mode entries send the OTHER view's
 * "off" to the Pi and stop the other ESP-side source, so logic<->scope swaps
 * are clean and only one source ever streams.
 * ------------------------------------------------------------------------- */
void handleTextCmd(const char *text) {
    char buf[101];
    strncpy(buf, text, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    for (char *p = buf; *p; ++p)                      // strip stray CR/LF
        if (*p == '\r' || *p == '\n') *p = ' ';
    if (!buf[0]) return;
 
    char cmd[10] = {0}; int ci = 0;
    while (buf[ci] && buf[ci] != ' ' && buf[ci] != ':' && ci < 9) {
        cmd[ci] = (char)tolower((unsigned char)buf[ci]); ci++;
    }
    const char *rest = buf[ci] ? buf + ci + 1 : buf + ci;   // skip the separator
 
    if (!strcmp(cmd,"qr") || !strcmp(cmd,"qrfull") || !strcmp(cmd,"qrsmall")) {
        char qcmd[120];
        snprintf(qcmd, sizeof qcmd, "%s %s", cmd, rest);
        piSendRaw(qcmd);
        Serial.printf("[TX-QR] %s\n", qcmd);



    } else if (!strcmp(cmd,"la")) {                       // logic-analyzer view
        if (!strncmp(rest,"off",3)) {
            bs05_stop(); g_laRun = false; piSendRaw("la off");
            Serial.println("[LA] off");
        } else if (!strncmp(rest,"sim",3)) {
            bs05_stop(); g_laRun = true;                          // synthetic source
            Serial.println("[LA] sim");
        } else {
            g_laRun = false; bs05_start();                        // real BS05U capture
            Serial.println("[LA] on");
        }
 
    } else if (!strcmp(cmd,"scope")) {                   // oscilloscope view (CHA/CHB)
        if (!strncmp(rest,"off",3)) {
            bs05_stop(); piSendRaw("sc off");
            Serial.println("[SCOPE] off");
        } else {
            bool sim = !strncmp(rest,"sim",3);           // "scope sim" = synthetic
            const char *chsel = rest;
            if      (sim)                       chsel += 3;
            else if (!strncmp(rest,"hw",2))     chsel += 2;  // accept optional "hw"
            while (*chsel == ' ') chsel++;
            uint8_t mask = 0x03;                          // default both
            if      (chsel[0]=='a' && chsel[1]!='b') mask = 0x01;
            else if (chsel[0]=='b')                  mask = 0x02;
            g_laRun = false;
            if (sim) bs05_scope(mask); else bs05_scope_hw(mask);  // real by default
            Serial.printf("[SCOPE] %s mask=0x%02x\n", sim ? "sim" : "hw", mask);
        }






 
    } else if (!strcmp(cmd,"sig")) {                     // A0..A3 test-signal generator
        if (!strncmp(rest,"off",3)) {                    // "sig off" -> all channels off
            siggen_off(-1);
            Serial.println("[SIG] all off");
        } else {
            int idx = -1; char mode[8] = {0}; unsigned long f = 0;
            int got = sscanf(rest, "%d %7s %lu", &idx, mode, &f);
            if (got >= 2 && !strcmp(mode,"off")) {
                siggen_off(idx);
                Serial.printf("[SIG] A%d off\n", idx);
            } else if (got >= 3 && (!strcmp(mode,"sq") || !strcmp(mode,"square"))) {
                bool ok = siggen_square(idx, (uint32_t)f);
                Serial.printf("[SIG] A%d square %lu Hz%s\n", idx, f, ok?"":" (bad args)");
            } else if (got >= 3 && (!strcmp(mode,"sine") || !strcmp(mode,"sin"))) {
                bool ok = siggen_sine(idx, (uint32_t)f);
                Serial.printf("[SIG] A%d sine %lu Hz (needs RC LPF)%s\n", idx, f, ok?"":" (bad args)");
            } else {
                Serial.println("[SIG] usage: sig <0-3> sq|sine <freq> | sig <0-3> off | sig off");
            }
        }

    } else if (!strcmp(cmd,"wifi")) {
        g_wifiReconfig = true;
        Serial.println("[WIFI] reconfigure requested");
 
    } else {                                             // plain text -> marquee
        for (int i = 0; i < (int)sizeof(buf) && buf[i] != '\0'; i++)
            buf[i] = (char)toupper((unsigned char)buf[i]);
        piCmd("msg %s", buf);
        Serial.printf("[TX] %s\n", buf);
    }
}




/* ============================ TASK 1: RemoteXY ===========================
 * Owns the RemoteXY struct; turns GUI controls into text commands. Sends only
 * on change (the UART link is reliable, so no heartbeat resend needed).
 * ==========================================================================*/
/* ---- Servo + stepper from RemoteXY ----------------------------------------
 * Add these widgets in the RemoteXY editor, then set USE_SERVO_STEPPER_GUI 1.
 *   Switch  servoSwitch   -> servo run / stop
 *   Slider  servoSpeed    0..30   -> servo sweep speed (deg/tick)
 *   Switch  servoDir      -> servo sweep direction
 *   Switch  stepSwitch    -> stepper run / stop
 *   Slider  stepSpeed     1..1000 -> stepper steps/sec
 *   Switch  stepDir       -> stepper CW / CCW
 * (The editor regenerates RemoteXY_CONF and the RemoteXY struct to match.)   */
#define USE_SERVO_STEPPER_GUI 1

/* Map a RemoteXY knob (0..100) to a sample rate with a geometric sweep, so each
 * step is an equal *ratio* (the natural feel for a timebase). knob 0 = slowest
 * (most time/div, zoomed out); knob 100 = fastest (least time/div, zoomed in).  */
static uint32_t knob_rate(int knob, uint32_t rmin, uint32_t rmax) {
    if (knob < 0)   knob = 0;
    if (knob > 100) knob = 100;
    return (uint32_t)(rmin * powf((float)rmax / (float)rmin, knob / 100.0f));
}

void remotexyTask(void *pv) {
    uint8_t lastRun  = 0xFF;
    int     lastMs   = -1;
    int     lastMask = -1;
    int     lastMsg  = -1;
    int     lastLa   = -1;
    int     lastSc   = -1;
    uint8_t lastSet  = 0;
#if USE_SERVO_STEPPER_GUI
    uint8_t lsSv = 0xFF, lsSvD = 0xFF, lsSt = 0xFF, lsStD = 0xFF;
    int     lsSvSp = -1, lsStSp = -1;
#endif

    for (;;) {
        RemoteXY_Handler();

        // run / stop
        uint8_t run = RemoteXY.switch1 ? 1 : 0;
        if (run != lastRun) { piCmd(run ? "run" : "stop"); lastRun = run; }

        // sweep speed: slider 0..100  ->  1000..20 ms (right = faster)
        int spd = RemoteXY.slider1; if (spd < 0) spd = 0; if (spd > 100) spd = 100;
        int ms = 1000 - (spd * 980) / 100;
        if (ms != lastMs) { piCmd("speed %d", ms); lastMs = ms; }

        // manual LED mask (0 => auto sweep resumes)
        uint8_t mask = 0;
        if (RemoteXY.led1) mask |= (1 << 0);
        if (RemoteXY.led2) mask |= (1 << 1);
        if (RemoteXY.led3) mask |= (1 << 2);
        if (RemoteXY.led4) mask |= (1 << 3);
        if (RemoteXY.led5) mask |= (1 << 4);
        if (RemoteXY.led6) mask |= (1 << 5);
        if (RemoteXY.led7) mask |= (1 << 6);
        if (RemoteXY.led8) mask |= (1 << 7);
        if ((int)mask != lastMask) { piCmd("leds %d", mask); lastMask = mask; }

        
        // set the Pi clock on the rising edge of the Set-time button.
        // Prefer the phone's real time (realTime_01); fall back to the manual
        // hour/min sliders if the app hasn't synced a timestamp yet.
        if (RemoteXY.settime && !lastSet) {
            int64_t ts = RemoteXY.realTime_01.getAppTimeStamp();   // ms; 0 until synced
            if (ts != 0) {
                RemoteXYTime t = RemoteXY.realTime_01.getAppTime(); // phone local time
                piCmd("set %02d:%02d:%02d", t.hour, t.minute, t.second);
                Serial.printf("[CLK] phone %02d:%02d:%02d\n", t.hour, t.minute, t.second);
            } else {
                int hh = RemoteXY.hourval; if (hh < 0) hh = 0; if (hh > 23) hh = 23;
                int mm = RemoteXY.minval;  if (mm < 0) mm = 0; if (mm > 59) mm = 59;
                piCmd("set %02d:%02d:00", hh, mm);
                Serial.printf("[CLK] manual %02d:%02d\n", hh, mm);
            }
        }
        lastSet = RemoteXY.settime;

    // --- custom message button (fires once per press) ---
    static uint8_t lastSendBtn = 0;
    if (RemoteXY.buttonMsg && !lastSendBtn) {
        handleTextCmd(RemoteXY.msgText);     // same path as the switches
    }
    lastSendBtn = RemoteXY.buttonMsg;


    // --- laSwitch / scSwitch: a flip == typing the command via the message box ---
    // Editor: add two Switch widgets named laSwitch and scSwitch (regenerates the
    // RemoteXY struct). They are mutually exclusive — turning one on clears the
    // other in the app so the GUI mirrors the single active view; handleTextCmd
    // also sends the other view's "off" to the Pi.
    static uint8_t lsLaSw = 0, lsScSw = 0;
    uint8_t laSw = RemoteXY.laSwitch ? 1 : 0;
    if (laSw != lsLaSw) {
        if (laSw && RemoteXY.scSwitch) { RemoteXY.scSwitch = 0; lsScSw = 0; }
        handleTextCmd(laSw ? "la" : "la off");
        lsLaSw = laSw;
    }
    uint8_t scSw = RemoteXY.scSwitch ? 1 : 0;
    if (scSw != lsScSw) {
        if (scSw && RemoteXY.laSwitch) { RemoteXY.laSwitch = 0; lsLaSw = 0; }
        const char *v = (RemoteXY.scChan==1) ? "scope a" : (RemoteXY.scChan==2) ? "scope b" : "scope";
        handleTextCmd(scSw ? v : "scope off");
        lsScSw = scSw;
    }

    // --- laTimebase / scTimebase knobs: change the live capture rate ----------
    // The new rate re-plans the BitScope clock on the next frame and is sent to
    // the Pi in the `begin` line. Baselines start at 0 (the RemoteXY default) so
    // the boot-time knob position does NOT force the rate off its good default
    // (g_la_rate_hz=4MHz / g_sc_rate_hz=8MHz) — the rate only changes once you
    // actually move a knob. Tune the min/max ends to your probe needs.
    static int lastLaTb = 0, lastScTb = 0;
    if (RemoteXY.laTimebase != lastLaTb) {
        g_la_rate_hz = knob_rate(RemoteXY.laTimebase, 250000UL, 40000000UL);  // 250kHz..40MHz
        lastLaTb = RemoteXY.laTimebase;
        Serial.printf("[LA] rate=%lu Hz\n", (unsigned long)g_la_rate_hz);
    }
    if (RemoteXY.scTimebase != lastScTb) {
        g_sc_rate_hz = knob_rate(RemoteXY.scTimebase, 50000UL, 20000000UL);   // 50kHz..20MHz
        lastScTb = RemoteXY.scTimebase;
        Serial.printf("[SC] rate=%lu Hz\n", (unsigned long)g_sc_rate_hz);
    }

    // --- qr_switch / qrmode: show/hide a QR built from msgText ----------------
    // qrmode 1 = full-screen (qrfull), 0 = small band (qrsmall). msgText is the
    // payload — set it, then flip qr_switch. Re-issues if you change mode while on.
    static uint8_t lsQr = 0, lsQrMode = 0xFF;
    uint8_t qrSw = RemoteXY.qr_switch ? 1 : 0;
    if (qrSw != lsQr || (qrSw && RemoteXY.qrmode != lsQrMode)) {
        if (qrSw) {
            char q[180];
            snprintf(q, sizeof q, "%s %s",
                     RemoteXY.qrmode ? "qrfull" : "qrsmall", RemoteXY.msgText);
            piSendRaw(q);
            Serial.printf("[QR] %s\n", q);
        } else {
            piSendRaw("qr off");
            Serial.println("[QR] off");
        }
        lsQr = qrSw;
        lsQrMode = RemoteXY.qrmode;
    }



#if USE_SERVO_STEPPER_GUI
        // --- servo ---
        uint8_t sv = RemoteXY.servoSwitch ? 1 : 0;
        if (sv != lsSv) { piCmd(sv ? "servo run" : "servo stop"); lsSv = sv; }
        int svsp = RemoteXY.servoSpeed; if (svsp < 1) svsp = 1; if (svsp > 30) svsp = 30;
        if (svsp != lsSvSp) { piCmd("servo speed %d", svsp); lsSvSp = svsp; }
        uint8_t svd = RemoteXY.servoDir ? 1 : 0;
        if (svd != lsSvD) { piCmd("servo dir %d", svd); lsSvD = svd; }
        // --- stepper ---
        uint8_t st = RemoteXY.stepSwitch ? 1 : 0;
        if (st != lsSt) { piCmd(st ? "step run" : "step stop"); lsSt = st; }
        int stsp = RemoteXY.stepSpeed; if (stsp < 1) stsp = 1; if (stsp > 1000) stsp = 1000;
        if (stsp != lsStSp) { piCmd("step speed %d", stsp); lsStSp = stsp; }
        uint8_t std = RemoteXY.stepDir ? 1 : 0;
        if (std != lsStD) { piCmd("step dir %d", std); lsStD = std; }
#endif

        vTaskDelay(pdMS_TO_TICKS(5));   // service RemoteXY ~200 Hz
    }
}

/* ============================ TASK 2: network ============================
 * NTP wall-clock -> "set" once per minute, and the once-a-day Aladhan fetch
 * -> "prayer". The blocking HTTP GET runs on this (other) core.
 * ==========================================================================*/
void networkTask(void *pv) {
    bool ntpStarted   = false;
    int  lastMin      = -1;
    int  lastFetchDay = -1;

    piCmd("machine on");   // re-assert quiet mode once tasks are live

    for (;;) {
        if (g_wifiReconfig) {                       // user asked to switch hotspot
            g_wifiReconfig = false;
            WiFiManager wm;
            wm.setConfigPortalTimeout(180);
            wm.startConfigPortal("LogicLA-Setup");  // blocks THIS task until done/timeout
            ntpStarted = false;                      // re-sync time on the new network
        }
        if (WiFi.status() == WL_CONNECTED) {
            if (!ntpStarted) {
                configTime(TZ_OFFSET_SEC, DST_OFFSET_SEC, "pool.ntp.org", "time.nist.gov");
                ntpStarted = true;
            }
            time_t now = time(nullptr);
            if (now > 1000000000) {                 // clock has been set
                struct tm *lt = localtime(&now);
                if (lt->tm_min != lastMin) {
                    piCmd("set %02d:%02d:%02d", lt->tm_hour, lt->tm_min, lt->tm_sec);
                    lastMin = lt->tm_min;
                }
                if (lt->tm_yday != lastFetchDay) {
                    fetchPrayerTimes();
                    lastFetchDay = lt->tm_yday;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}


/* ============================ TASK 3: WebSocket =========================
 * WS client to the backend. On each text frame, forward the notification to
 * the Pi as a scrolling marquee. Reconnects automatically if the link drops.
 * ==========================================================================*/
WebSocketsClient webSocket;

// Forward a notification to the Pi as a scrolling marquee. Strips CR/LF and
// other control chars (the Pi link is line-based) and clamps the length.
static void forwardMarquee(const char *text) {
    char buf[MARQUEE_MAX + 1];
    int n = 0;
    for (const char *s = text; *s && n < MARQUEE_MAX; ++s) {
        char c = *s;
        if (c == '\r' || c == '\n' || c == '\t') c = ' ';
        if ((unsigned char)c < 0x20) continue;     // drop other control chars
        buf[n++] = c;
    }
    buf[n] = '\0';
    if (n == 0) return;
    piCmd("msg %s", buf);                          // -> Pi marquee
    Serial.printf("[WS->PI] msg %s\n", buf);
}

// Turn an incoming WS text frame into a marquee update. We act ONLY on the
// "netupdate" section; any other section is ignored. The nested "message"
// object {host,status,since} becomes a gated netwatch overlay sent as "netmsg"
// (case preserved, GPIO21-gated on the Pi). An empty/absent message sends a
// bare "netmsg" so the Pi falls back to the usual base marquee.
static void handleWsText(uint8_t *payload, size_t length) {
    static char in[512];
    size_t m = (length < sizeof(in) - 1) ? length : sizeof(in) - 1;
    memcpy(in, payload, m); in[m] = '\0';

    if (in[0] != '{') { forwardMarquee(in); return; }      // plain text -> marquee

    JsonDocument doc;
    if (deserializeJson(doc, in)) { forwardMarquee(in); return; }  // bad JSON -> raw

    const char *section = doc["section"] | "";
    if (strcmp(section, "netupdate") != 0) return;         // ignore other sections

    const char *host   = doc["message"]["host"]   | "";
    const char *status = doc["message"]["status"] | "";
    const char *since  = doc["message"]["since"]  | "";

    if (!host[0] && !status[0] && !since[0]) {              // empty -> show usual
        piCmd("netmsg");                                   // Pi reverts to g_base_msg
        Serial.println("[WS->PI] netmsg (empty) -> base");
        return;
    }

    char line[MARQUEE_MAX + 1];
    snprintf(line, sizeof line, "Host %s status %s since %s", host, status, since);

    char buf[MARQUEE_MAX + 1];                              // strip control chars
    int n = 0;
    for (const char *s = line; *s && n < MARQUEE_MAX; ++s) {
        char c = *s;
        if (c == '\r' || c == '\n' || c == '\t') c = ' ';
        if ((unsigned char)c < 0x20) continue;
        buf[n++] = c;
    }
    buf[n] = '\0';
    piCmd("netmsg %s", buf);                                // gated WS overlay
    Serial.printf("[WS->PI] netmsg %s\n", buf);
}

static void webSocketEvent(WStype_t type, uint8_t *payload, size_t length) {
    switch (type) {
        case WStype_CONNECTED:
            Serial.printf("[WS] connected %s:%d%s\n", WS_HOST, WS_PORT, WS_PATH);
            if (sizeof(WS_HELLO) > 1) webSocket.sendTXT(WS_HELLO);   // optional subscribe/auth
            break;
        case WStype_DISCONNECTED:
            Serial.println("[WS] disconnected");
            break;
        case WStype_TEXT:
            // full raw text exactly as it arrives (payload is NOT null-terminated)
            Serial.printf("[WS-RAW %u] %.*s\n", (unsigned)length, (int)length, (const char*)payload);
            handleWsText(payload, length);
            break;
        default:
            break;   // BIN / PING / PONG / ERROR are handled by the library
    }
}

void websocketTask(void *pv) {
    while (WiFi.status() != WL_CONNECTED) vTaskDelay(pdMS_TO_TICKS(200));  // wait for WiFiManager's link
#if WS_USE_SSL
    webSocket.beginSSL(WS_HOST, WS_PORT, WS_PATH);
#else
    webSocket.begin(WS_HOST, WS_PORT, WS_PATH);
#endif
    webSocket.onEvent(webSocketEvent);
    webSocket.setReconnectInterval(5000);           // retry every 5 s if dropped
    webSocket.enableHeartbeat(15000, 3000, 2);      // ping 15 s, 3 s pong timeout, drop after 2 misses
    for (;;) {
        webSocket.loop();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void setup() {
    #include <esp_log.h>
    // ...
    esp_log_level_set("*", ESP_LOG_DEBUG);   // surface USB HOST / cdc_acm / FT23x / VCP logs







    Serial.begin(115200);                          // USB debug console
    Serial1.setTxBufferSize(2048);
    Serial1.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);

    bs05_begin(&vcp);                 // starts USB host (manages hot-plug)
    siggen_begin();                   // A0..A3 test-signal generator ready (all off)
    xTaskCreatePinnedToCore(bs05_task, "bs05", 4096, (void*)piSendRaw, 1, NULL, 1);


    delay(200);
    Serial1.print("machine on\r\n");               // quiet the Pi link early
    Serial.println("\nStarting...");

    // ---- WiFi: WiFiManager owns provisioning + roaming (independent of RemoteXY/BLE) ----
    WiFi.mode(WIFI_STA);
    {
        WiFiManager wm;
        wm.setConfigPortalTimeout(180);            // 3-min captive portal, then give up
        // Connects using saved creds; if none (or you moved the board and they
        // fail), opens AP "LogicLA-Setup" where you scan + pick a hotspot. The
        // choice is saved to flash (NVS) and auto-used on the next boot.
        if (!wm.autoConnect("LogicLA-Setup"))
            Serial.println("WiFi not provisioned; continuing (BLE control still works)");
    }

    RemoteXY_Init();                               // RemoteXY over BLE; does not touch WiFi

    serialMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(remotexyTask, "remotexy", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(networkTask,  "network",  8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(websocketTask,"websocket",8192, NULL, 1, NULL, 0);
    xTaskCreatePinnedToCore(laSimTask,    "la_sim",   4096, NULL, 1, NULL, 1);
}

void loop() {
    // Mirror everything the Pi sends back (acks / response text) to USB Serial0.
    while (Serial1.available()) {
        Serial.write((uint8_t)Serial1.read());
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}