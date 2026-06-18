/* la_sim.h — synthetic animated 8-channel logic source. No USB / BitScope
 * needed: lets you prove the ESP32->UART->Pi->HDMI path from RemoteXY while
 * the real BS05U cable is pending. Pure C; compiles under plain Arduino.
 * When the BS05U arrives, swap la_sim_fill() for bs05_capture(). */
#ifndef LA_SIM_H
#define LA_SIM_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
/* Fill 'n' samples (1 byte/sample, bit i = channel i) with an animated scene
 * that scrolls by 'frame' so the HDMI view visibly moves between frames. */
void la_sim_fill(uint8_t *samples, int n, uint32_t frame);
#ifdef __cplusplus
}
#endif
#endif
