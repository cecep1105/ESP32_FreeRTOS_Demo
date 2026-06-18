/* esp_usb_vcp.h — IVcp backend built on bertmelis/USBHostSerial (Arduino-
 * native USB-host VCP for ESP32-S3). Stays on `framework = arduino`; no
 * esp-idf managed components, no dual framework. The library manages
 * connect/reconnect internally, so this adapter just maps IVcp onto its
 * Serial-style begin()/read()/write(). Implementation in esp_usb_vcp.cpp.
 */
#ifndef ESP_USB_VCP_H
#define ESP_USB_VCP_H
#include "bs05_logic.h"        /* IVcp */
#include <stdint.h>
#include <stddef.h>

class EspUsbVcp : public IVcp {
public:
    bool begin() override;                              /* install host + drivers, start connect task */
    bool connected() override;
    void setBaud(uint32_t baud) override;              /* applied on open / live  */
    int  write(const void *buf, size_t len) override;  /* tx_blocking             */
    int  read(void *buf, size_t len, uint32_t timeout_ms) override; /* drain RX   */
};

#endif
