#include <Arduino.h>

//////////////////////////////////////////////
//        RemoteXY include library          //
//////////////////////////////////////////////

//#define REMOTEXY__DEBUGLOG

#define REMOTEXY_MODE__WIFI
#include <WiFi.h>
#include <HTTPClient.h>
#include <RemoteXY.h>
#include <time.h>
#include <stdarg.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

// RemoteXY connection settings
#define REMOTEXY_WIFI_SSID     "HIBAWIFI"
#define REMOTEXY_WIFI_PASSWORD "ab43215cde"
#define REMOTEXY_SERVER_PORT   6377

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

// RemoteXY GUI configuration
#pragma pack(push, 1)  
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 500 bytes V19 
  { 255,223,0,0,0,237,1,19,0,0,0,0,31,1,106,200,2,1,0,25,
  0,130,3,3,100,36,11,107,130,4,43,98,51,11,107,130,59,99,41,64,
  11,107,130,5,100,37,62,11,105,2,7,9,24,22,0,2,26,31,31,79,
  78,0,79,70,70,0,2,10,47,16,16,1,2,26,31,31,79,78,0,79,
  70,70,0,2,34,47,16,16,1,2,26,31,31,79,78,0,79,70,70,0,
  2,57,47,16,16,1,2,26,31,31,79,78,0,79,70,70,0,2,80,47,
  16,16,1,2,26,31,31,79,78,0,79,70,70,0,2,10,65,16,16,1,
  2,26,31,31,79,78,0,79,70,70,0,2,34,65,16,16,1,2,26,31,
  31,79,78,0,79,70,70,0,2,57,65,16,16,1,2,26,31,31,79,78,
  0,79,70,70,0,2,80,65,16,16,1,2,26,31,31,79,78,0,79,70,
  70,0,4,40,13,59,13,128,2,26,2,9,106,26,13,0,2,26,31,31,
  79,78,0,79,70,70,0,2,66,106,26,13,0,2,26,31,31,79,78,0,
  79,70,70,0,4,9,123,26,9,128,2,26,4,66,122,26,9,128,2,26,
  2,10,135,26,13,0,2,26,31,31,79,78,0,79,70,70,0,2,66,135,
  26,13,0,2,26,31,31,79,78,0,79,70,70,0,129,13,153,22,7,64,
  1,83,69,82,86,79,0,129,62,153,30,7,64,1,83,84,69,80,80,69,
  82,0,129,36,31,37,7,64,1,76,69,68,32,83,87,69,69,80,0,129,
  21,85,70,7,64,1,77,65,78,85,65,76,32,76,69,68,32,79,78,92,
  47,79,70,70,0,131,37,174,26,14,3,17,2,31,62,62,0,6,10,0,
  130,3,90,100,49,11,17,130,3,8,100,78,11,17,7,7,38,93,12,100,
  0,2,26,2,201,1,41,59,24,24,0,2,31,0,7,6,101,34,10,118,
  64,2,26,2,7,45,101,34,10,118,64,2,26,2,1,83,98,17,17,0,
  2,31,0,129,18,15,75,12,64,6,77,101,115,115,97,103,101,32,84,101,
  120,116,0,129,24,124,62,8,64,6,77,97,110,117,97,108,32,67,108,111,
  99,107,32,83,101,116,0,131,41,169,26,14,3,17,2,31,60,60,0,9 };
  
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
  uint8_t servoSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t stepSwitch; // =1 if switch ON and =0 if OFF, from 0 to 1
  int8_t servoSpeed; // from 0 to 100
  int8_t stepSpeed; // from 0 to 100
  uint8_t servoDir; // =1 if switch ON and =0 if OFF, from 0 to 1
  uint8_t stepDir; // =1 if switch ON and =0 if OFF, from 0 to 1
  char msgText[201]; // string UTF8 end zero
  uint8_t buttonMsg; // =1 if button pressed, else =0, from 0 to 1
  int16_t hourval; // -32768 .. +32767
  int16_t minval; // -32768 .. +32767
  uint8_t settime; // =1 if button pressed, else =0, from 0 to 1

    // other variable
  uint8_t connect_flag;  // =1 if wire connected, else =0

} RemoteXY;   
#pragma pack(pop)

/////////////////////////////////////////////
//           END RemoteXY include          //
/////////////////////////////////////////////

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
void piSendRaw(const char *s) {
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        Serial1.print(s);
        Serial1.print("\r\n");
        xSemaphoreGive(serialMutex);
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

void remotexyTask(void *pv) {
    uint8_t lastRun  = 0xFF;
    int     lastMs   = -1;
    int     lastMask = -1;
    int     lastMsg  = -1;
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

        // set clock on the rising edge of the Set-time button
        if (RemoteXY.settime && !lastSet) {
            int hh = RemoteXY.hourval; if (hh < 0) hh = 0; if (hh > 23) hh = 23;
            int mm = RemoteXY.minval;  if (mm < 0) mm = 0; if (mm > 59) mm = 59;
            piCmd("set %s", hh, mm);
        }
        lastSet = RemoteXY.settime;

    // --- custom message button (fires once per press) ---
    static uint8_t lastSendBtn = 0;
    if (RemoteXY.buttonMsg && !lastSendBtn) {
        char buf[201];
        strncpy(buf, RemoteXY.msgText, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';                 // hard 200-char clamp + null
        for (char *p = buf; *p; ++p)                 // strip stray CR/LF from keyboard
            if (*p == '\r' || *p == '\n') *p = ' ';
        if (buf[0]) {
            for (int i = 0; i < sizeof(buf) && buf[i] != '\0'; i++) buf[i] = toupper(buf[i]);
            piCmd("msg %s", buf);              // text as ARGUMENT, not as format
            Serial.printf("[TX] %s\n", buf);
        }
    }
    lastSendBtn = RemoteXY.buttonMsg;







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


void setup() {
    Serial.begin(115200);                          // USB debug console
    Serial1.begin(PI_BAUD, SERIAL_8N1, PI_RX_PIN, PI_TX_PIN);
    delay(200);
    Serial1.print("machine on\r\n");               // quiet the Pi link early
    Serial.println("\nStarting...");

    RemoteXY_Init();                               // RemoteXY manages the WiFi link

    serialMutex = xSemaphoreCreateMutex();
    xTaskCreatePinnedToCore(remotexyTask, "remotexy", 8192, NULL, 2, NULL, 1);
    xTaskCreatePinnedToCore(networkTask,  "network",  8192, NULL, 1, NULL, 0);
}

void loop() {
    // Mirror everything the Pi sends back (acks / response text) to USB Serial0.
    while (Serial1.available()) {
        Serial.write((uint8_t)Serial1.read());
    }

    vTaskDelay(pdMS_TO_TICKS(1000));
}
