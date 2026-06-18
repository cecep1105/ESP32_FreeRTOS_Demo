/* esp_usb_vcp.cpp — IVcp backend that drives the BS05U's FTDI DIRECTLY.
 *
 * Why not the high-level USBHostSerial class: it calls VCP::open(), and when
 * the FTDI (FT23x) constructor throws, VCP::open swallows the error
 * (catch -> default: return nullptr) and USBHostSerial silently falls back to
 * a generic CDC-ACM open — which can't set FTDI baud ("line_coding_set not
 * supported"). Here we build FT23x ourselves so the real esp_err is visible,
 * retry with a settle delay, and never use the broken fallback.
 *
 * Uses the USB headers/impl BUNDLED in the USBHostSerial library (so it builds
 * on plain framework = arduino with the pioarduino core 3.x / IDF 5.x).
 * Same IVcp interface => bs05_logic.cpp etc. unchanged.
 */
#include "esp_usb_vcp.h"
#include "bs05_logic.h"          // BS_BAUD

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/stream_buffer.h"
#include "freertos/semphr.h"

#include <usb/usb_host.h>
#include <usb/cdc_acm_host.h>
#include <usb/vcp_ftdi.hpp>      // esp_usb::FT23x, FT232_PID
#include <esp_log.h>

using namespace esp_usb;
static const char *TAG = "bs05vcp";

static CdcAcmDevice         *s_dev   = nullptr;
static volatile bool         s_conn  = false;
static uint32_t              s_baud  = BS_BAUD;
static bool                  s_begun = false;
static StreamBufferHandle_t  s_rx    = nullptr;
static SemaphoreHandle_t     s_disc  = nullptr;

/* FT23x delivers RX already stripped of its 2-byte modem status. */
static bool on_rx(const uint8_t *data, size_t len, void *arg){
    (void)arg;
    if (s_rx) xStreamBufferSend(s_rx, data, len, 0);   // drop if full (bursty ok)
    return true;
}
static void on_event(const cdc_acm_host_dev_event_data_t *ev, void *ctx){
    (void)ctx;
    if (ev->type == CDC_ACM_HOST_DEVICE_DISCONNECTED){
        s_conn = false;
        if (s_disc) xSemaphoreGive(s_disc);
    }
}

static void usb_lib_task(void *arg){
    (void)arg;
    for (;;){
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void apply_baud(){
    if (!s_dev) return;
    cdc_acm_line_coding_t lc = {};
    lc.dwDTERate = s_baud; lc.bCharFormat = 0; lc.bParityType = 0; lc.bDataBits = 8;
    esp_err_t e = s_dev->line_coding_set(&lc);          // FT23x's FTDI baud path
    if (e != ESP_OK) ESP_LOGE(TAG, "set baud failed: %s", esp_err_to_name(e));
    s_dev->set_control_line_state(true, true);          // DTR, RTS
}

static void connect_task(void *arg){
    (void)arg;
    for (;;){
        cdc_acm_host_device_config_t cfg = {};
        cfg.connection_timeout_ms = 3000;               // wait for the device to appear
        cfg.out_buffer_size = 512;
        cfg.in_buffer_size  = 2048;
        cfg.event_cb = on_event;
        cfg.data_cb  = on_rx;
        cfg.user_arg = nullptr;

        FT23x *dev = nullptr;
        try {
            dev = new FT23x(FT232_PID, &cfg, 0);        // throws on failure -> real error
        } catch (esp_err_t e) {
            ESP_LOGE(TAG, "FT23x open failed: 0x%x (%s)", e, esp_err_to_name(e));
            dev = nullptr;
        } catch (...) {
            ESP_LOGE(TAG, "FT23x open failed: unknown exception");
            dev = nullptr;
        }
        if (!dev){ vTaskDelay(pdMS_TO_TICKS(500)); continue; }   // retry

        s_dev = dev;
        apply_baud();
        if (s_rx) xStreamBufferReset(s_rx);
        s_conn = true;
        ESP_LOGI(TAG, "BS05U FTDI connected @ %lu baud", (unsigned long)s_baud);

        xSemaphoreTake(s_disc, portMAX_DELAY);          // block until unplug
        s_conn = false;
        delete s_dev;                                   // closes the device
        s_dev = nullptr;
        ESP_LOGI(TAG, "BS05U disconnected");
    }
}

bool EspUsbVcp::begin(){
    if (s_begun) return true;
    s_begun = true;

    s_rx   = xStreamBufferCreate(8192, 1);
    s_disc = xSemaphoreCreateBinary();
    if (!s_rx || !s_disc) return false;

    usb_host_config_t host_cfg = {};
    host_cfg.intr_flags = ESP_INTR_FLAG_LEVEL1;
    esp_err_t e = usb_host_install(&host_cfg);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG, "usb_host_install: %s", esp_err_to_name(e));
        return false;
    }
    xTaskCreatePinnedToCore(usb_lib_task, "usb_lib", 4096, nullptr, 10, nullptr, 0);

    e = cdc_acm_host_install(nullptr);
    if (e != ESP_OK && e != ESP_ERR_INVALID_STATE){
        ESP_LOGE(TAG, "cdc_acm_host_install: %s", esp_err_to_name(e));
        return false;
    }
    xTaskCreatePinnedToCore(connect_task, "bs05_conn", 4096, nullptr, 5, nullptr, 0);
    return true;
}

bool EspUsbVcp::connected(){ return s_conn; }

void EspUsbVcp::setBaud(uint32_t baud){
    s_baud = baud;
    if (s_conn) apply_baud();
}

int EspUsbVcp::write(const void *buf, size_t len){
    if (!s_conn || !s_dev) return -1;
    uint8_t *p = (uint8_t *)buf;                        // tx_blocking wants non-const
    size_t off = 0; uint32_t t0 = millis();
    while (off < len){
        if (s_dev->tx_blocking(p + off, len - off, 200) == ESP_OK){ off = len; break; }
        if (millis() - t0 > 500) break;
        vTaskDelay(1);
    }
    return (int)off;
}

int EspUsbVcp::read(void *buf, size_t len, uint32_t timeout_ms){
    if (!s_rx) return 0;
    uint8_t *p = (uint8_t *)buf; size_t off = 0; uint32_t t0 = millis();
    while (off < len){
        size_t r = xStreamBufferReceive(s_rx, p + off, len - off, pdMS_TO_TICKS(10));
        if (r > 0){ off += r; t0 = millis(); }
        else if (millis() - t0 > timeout_ms) break;
    }
    return (int)off;
}