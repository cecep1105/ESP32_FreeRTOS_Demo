/* siggen.cpp — square + PWM-sine test signals on A0..A3 (ESP32-S3 GPIO1..4).
 * See siggen.h for the sine RC-filter note. Works on Arduino-ESP32 2.x and 3.x
 * (the LEDC API changed at 3.0; both paths are handled below).
 */
#include "siggen.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* A0..A3 on the ESP32-S3 Arduino core map to GPIO1..GPIO4. */
static const int      SIG_PIN[4]   = { A0, A1, A2, A3 };
static const uint32_t SINE_CARRIER = 100000;   /* PWM carrier for sine mode (Hz) */
static const uint8_t  SINE_RES     = 8;        /* duty resolution for the sine    */

static volatile bool     sig_active [4] = { false, false, false, false };
static volatile bool     sig_is_sine[4] = { false, false, false, false };
static volatile uint32_t sig_freq   [4] = { 0, 0, 0, 0 };

/* full-period sine table, 0..255 (centred on 128) */
static uint8_t SINE256[256];
static void sine_lut_init(void){
    for(int i=0;i<256;i++){
        float a = (float)i * 6.2831853f / 256.0f;
        SINE256[i] = (uint8_t)(127.5f + 127.0f * sinf(a));
    }
}

/* ---- LEDC shim: Arduino-ESP32 3.x is pin-addressed, 2.x is channel-addressed */
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  static void sig_attach (int i, uint32_t f, uint8_t res){ ledcAttach(SIG_PIN[i], f, res); }
  static void sig_duty   (int i, uint32_t d){ ledcWrite(SIG_PIN[i], d); }
  static void sig_detach (int i){ ledcDetach(SIG_PIN[i]); }
#else
  static void sig_attach (int i, uint32_t f, uint8_t res){ ledcSetup(i, f, res); ledcAttachPin(SIG_PIN[i], i); }
  static void sig_duty   (int i, uint32_t d){ ledcWrite(i, d); }
  static void sig_detach (int i){ ledcDetachPin(SIG_PIN[i]); }
#endif

/* pick a duty resolution whose max frequency (80MHz / 2^res) covers `freq`,
 * so 50%-duty squares stay valid from a few Hz up to the low-MHz range. */
static uint8_t res_for_freq(uint32_t freq){
    if(freq < 1) freq = 1;
    uint8_t res = 14;                       /* S3 LEDC max is 14-bit */
    while(res > 1 && (80000000UL >> res) < freq) res--;
    return res;
}

static bool valid(int idx){ return idx >= 0 && idx < 4; }

/* helper task: sweep the duty of any sine-mode channel through the LUT.
 * ~1 kHz update — keep sine frequencies low (<= ~100 Hz) for a clean result. */
static void sigSineTask(void *pv){
    (void)pv;
    for(;;){
        uint32_t now = micros();
        for(int i=0;i<4;i++){
            if(sig_active[i] && sig_is_sine[i] && sig_freq[i]){
                uint32_t idx = (uint32_t)(((uint64_t)now * sig_freq[i] * 256ULL)
                                          / 1000000ULL) & 0xFF;
                sig_duty(i, SINE256[idx]);
            }
        }
        vTaskDelay(1);
    }
}

void siggen_begin(void){
    sine_lut_init();
    xTaskCreatePinnedToCore(sigSineTask, "siggen", 2048, NULL, 1, NULL, 1);
}

bool siggen_square(int idx, uint32_t freq){
    if(!valid(idx) || freq < 1) return false;
    uint8_t res = res_for_freq(freq);
    if(sig_active[idx]) sig_detach(idx);
    sig_is_sine[idx] = false;
    sig_freq[idx]    = freq;
    sig_attach(idx, freq, res);
    sig_duty(idx, (1UL << res) / 2);        /* 50% duty */
    sig_active[idx] = true;
    return true;
}

bool siggen_sine(int idx, uint32_t freq){
    if(!valid(idx) || freq < 1) return false;
    if(sig_active[idx]) sig_detach(idx);
    sig_freq[idx]    = freq;
    sig_is_sine[idx] = true;
    sig_attach(idx, SINE_CARRIER, SINE_RES); /* fixed carrier; task varies duty */
    sig_duty(idx, 128);
    sig_active[idx] = true;
    return true;
}

void siggen_off(int idx){
    if(idx < 0){ for(int i=0;i<4;i++) siggen_off(i); return; }
    if(!valid(idx)) return;
    if(sig_active[idx]){ sig_duty(idx, 0); sig_detach(idx); }
    sig_active[idx]  = false;
    sig_is_sine[idx] = false;
    sig_freq[idx]    = 0;
}