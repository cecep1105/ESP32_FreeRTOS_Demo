/* siggen.h — test-signal generator on A0..A3 (ESP32-S3: GPIO1..GPIO4).
 *
 * SQUARE  : clean 50%-duty LEDC output. Wire a pin to a BS05 logic input
 *           (L0..L7) to test the logic analyzer, or to CHA/CHB to see a
 *           square on the scope. No extra parts needed. ~5 Hz .. a few MHz.
 *
 * SINE    : the S3 has NO DAC, so this is a PWM approximation — a fixed HF
 *           carrier whose duty is swept through a sine table. It only becomes
 *           a real analog sine after an external RC low-pass on the pin, e.g.
 *           1.6k ohm in series + 100 nF to GND (~1 kHz cutoff). Cleanest below
 *           ~100 Hz; lower the RC cutoff for lower frequencies.
 *
 * Pins are driven by LEDC; the sine duty is updated by a small helper task.
 */
#ifndef SIGGEN_H
#define SIGGEN_H
#include <stdint.h>

void siggen_begin(void);                        /* init LUT + start sine task   */
bool siggen_square(int idx, uint32_t freq_hz);  /* idx 0..3 -> A0..A3, 50% duty */
bool siggen_sine  (int idx, uint32_t freq_hz);  /* idx 0..3 -> A0..A3 (needs RC)*/
void siggen_off   (int idx);                     /* idx < 0 = all channels off   */

#endif /* SIGGEN_H */