/* bs05_logic.h — ESP32-S3 USB-host BitScope BS05U logic capture -> Pi.
 *
 * Pairs with bvm.h/.cpp (the protocol helpers, already host-validated). The
 * USB-host serial library is abstracted behind IVcp so you can bind either:
 *   - Espressif esp-usb VCP (usb_host_cdc_acm + usb_host_ftdi), or
 *   - bertmelis/USBHostSerial (Arduino/pioarduino) — bundles the FTDI VCP.
 * The BS05U enumerates as FT232R (VID 0x0403 / PID 0x6001).
 *
 * Lifecycle: bs05_begin(vcp) once; then bs05_start()/bs05_stop() from the
 * RemoteXY button or the "la on/off" serial command. The task free-runs at
 * BS_FPS, streaming one "la begin/d/end" frame per grab on Serial1.
 */
#ifndef BS05_LOGIC_H
#define BS05_LOGIC_H
#include <stdint.h>
#include <stddef.h>

/* ---- tunables (confirm BS_BAUD against your device on first bring-up) -- */
#define BS_BAUD     1000000     /* FT232R link rate; try 1M, fall back 115200 */
#define BS_NCOLS    240         /* display columns sent to the Pi (<=256)     */
#define BS_NSAMPLES 480         /* raw samples captured per grab (<=12288)    */
#define BS_RATE_HZ  4000000     /* target logic sample rate (40MHz/ticks)     */
#define BS_FPS      4           /* live-view frames per second (2..5)         */
#define BS_DIGITAL  0xFF        /* enabled logic channels bitmap (L0..L7)     */
#define BS_SC_NCOLS   240       /* scope display columns sent to the Pi       */
#define BS_SC_RATE_HZ 8000000   /* scope target sample rate (display only)    */
#define BS_SC_CONV_LO 0x0000    /* [HW] U0.16 ADC window bottom — TUNE on hw  */
#define BS_SC_CONV_HI 0xFFFF    /* [HW] U0.16 ADC window top    — TUNE on hw  */

/* Minimal USB-host VCP interface — bind to your chosen library (see .cpp). */
class IVcp {
public:
    virtual ~IVcp() {}
    virtual bool begin() = 0;                 /* init host stack            */
    virtual bool connected() = 0;             /* BS05U enumerated & open    */
    virtual void setBaud(uint32_t baud) = 0;  /* FTDI line coding           */
    virtual int  write(const void *buf, size_t len) = 0;
    /* read up to len bytes, blocking up to timeout_ms; returns bytes read   */
    virtual int  read(void *buf, size_t len, uint32_t timeout_ms) = 0;
};

void bs05_begin(IVcp *vcp);   /* register the VCP backend + reset device */
void bs05_start(void);        /* begin free-running LOGIC capture/stream */
void bs05_scope(uint8_t chmask); /* SCOPE mode (SIM source): bit0=A, bit1=B    */
void bs05_scope_hw(uint8_t chmask); /* SCOPE mode (REAL analog BVM capture)    */
void bs05_stop(void);         /* stop; Pi should also get "la off"/"sc off" */
bool bs05_running(void);

/* The FreeRTOS task body (create with xTaskCreate in setup()). It owns the
 * capture->pack->emit loop. 'pvSendLine' is a function that writes one wire
 * line to the Pi (your Serial1 + CRLF sender). */
typedef void (*bs05_send_fn)(const char *line);
void bs05_task(void *pv);     /* pv = (bs05_send_fn) line sender          */

#endif
