/* bvm.h — BitScope Virtual Machine helpers for the ESP32 logic-analyzer
 * bridge. Pure C++ (no Arduino/USB deps) so the protocol logic is unit-
 * testable on a host. Ported faithfully from the BS05-tested 'scopething'
 * register sequence.
 *
 *   BVM register-write grammar (what build_set_registers emits):
 *     "AA@"  set address register to hex AA
 *     "BB"   push byte value BB (hex);  "["  pushes 0
 *     "z"    store accumulator -> [addr], addr++   (store-and-advance)
 *     "n"    addr++                                 (skip a register)
 *     "s"    store accumulator -> [addr]            (final store)
 *   e.g. a 16-bit write of 0x0102 to addr 0x2e  ->  "2e@02z01s"
 *
 * Device facts (BS05/BS10 class): 40 MHz primary clock, 12 KiB buffer,
 * logic = 1 byte/sample, bit i = channel i. Enumerates as FT232R 0403:6001.
 */
#ifndef BVM_H
#define BVM_H
#include <stdint.h>
#include <stddef.h>

/* ---- VM register bases (subset needed for logic capture) ------------- */
enum {
    R_TriggerLogic = 0x05, R_TriggerMask = 0x06, R_SpockOption = 0x07,
    R_SampleAddress= 0x08, R_TriggerIntro= 0x32, R_TriggerOutro = 0x34,
    R_ClockTicks   = 0x2e, R_ClockScale  = 0x14, R_TraceMode    = 0x21,
    R_TraceIntro   = 0x26, R_TraceDelay  = 0x22, R_TraceOutro   = 0x2a,
    R_Timeout      = 0x2c, R_Prelude     = 0x3a, R_BufferMode   = 0x31,
    R_DumpMode     = 0x1e, R_DumpChan    = 0x30, R_DumpSend     = 0x18,
    R_DumpSkip     = 0x1a, R_DumpCount   = 0x1c, R_DumpRepeat   = 0x16,
    R_AnalogEnable = 0x37, R_DigitalEnable= 0x38,
    R_ConverterLo  = 0x64, R_ConverterHi = 0x66, R_TriggerLevel = 0x68,
    R_KitchenSinkA = 0x7b, R_KitchenSinkB= 0x7c,
    R_Cmd          = 0x46, R_Mode         = 0x47, R_Map5 = 0x99
};
enum { TRACE_LOGIC = 14, BUFMODE_SINGLE = 0, DUMP_RAW = 0 };
enum { TRACE_ANALOG = 0, TRACE_ANALOGCHOP = 2, BUFMODE_CHOP = 1,
       KSB_ANALOG_FILTER = 0x80 };
enum { TS_DONE = 0, TS_AUTO = 1, TS_WAIT = 2, TS_STOP = 3 };

#define BVM_PRIMARY_CLOCK 40000000UL
#define BVM_BUFFER_BYTES  (12u << 10)      /* 12288 logic samples */
#define BVM_FT_VID 0x0403
#define BVM_FT_PID 0x6001

/* One register to write (value is host-order; emitted little-endian). */
typedef struct { uint8_t base; uint8_t width; uint32_t value; } bvm_reg_t;

/* Build the BVM string that writes the given registers. 'regs' need NOT be
 * sorted. Returns strlen written into out (NUL-terminated). */
size_t bvm_build_set_registers(const bvm_reg_t *regs, int n, char *out, size_t outsz);

/* Capture parameters resolved for a target rate / sample count. */
typedef struct {
    uint16_t ticks;        /* ClockTicks (1..16384) */
    uint16_t nsamples;     /* samples captured (<= buffer) */
    uint32_t actual_rate;  /* effective samples/sec */
    uint16_t trace_intro, trace_outro, timeout;
} bvm_logic_cfg_t;

/* Resolve ticks/intro/outro for a free-running logic grab. */
void bvm_logic_plan(uint32_t target_rate, uint16_t nsamples, bvm_logic_cfg_t *c);

/* ---- analog (oscilloscope) capture builders ------------------------------
 * Free-running CHA/CHB grab. nch=1 single buffer (TRACE_ANALOG), nch=2 chop
 * buffer (TRACE_ANALOGCHOP); in chop mode each channel is dumped separately by
 * DumpChan (0=A,1=B), so no manual de-interleave. ConverterLo/Hi are the raw
 * U0.16 ADC window and WILL need per-device tuning (uncalibrated by default). */
void   bvm_scope_plan(uint32_t target_rate, uint16_t nsamples, int nch,
                      bvm_logic_cfg_t *c);
size_t bvm_scope_setup_cmd(const bvm_logic_cfg_t *c, uint8_t analog_mask, int nch,
                           uint16_t conv_lo, uint16_t conv_hi,
                           char *out, size_t outsz);
size_t bvm_scope_dump_cmd (const bvm_logic_cfg_t *c, uint32_t write_addr,
                           int dump_chan, int nch, char *out, size_t outsz);

/* Emit the register block for a logic capture (call ' >','U','D' after). */
size_t bvm_logic_setup_cmd(const bvm_logic_cfg_t *c, uint8_t digital_enable,
                           char *out, size_t outsz);

/* Emit the dump-setup register block; after this issue '>' then 'A', then
 * read c->nsamples raw bytes = the logic buffer. 'write_addr' is the value
 * read back from the device after the trace completes. */
size_t bvm_logic_dump_cmd(const bvm_logic_cfg_t *c, uint32_t write_addr,
                          char *out, size_t outsz);

/* Emit the register block that unmaps the internal clock from logic channel 5
 * (Map5=0) and stops the clock generator. Issue 'Z' after this. Without it,
 * a running clock generator drives L5 instead of the external probe. */
size_t bvm_stop_clock_cmd(char *out, size_t outsz);

/* ---- ESP32 -> Pi wire framing (the frozen contract) ------------------ */
/* Decimate 'nsamp' raw logic bytes down to 'ncols' columns (max 256) by
 * point sampling, into col[]. Returns ncols actually produced. */
int bvm_pack_columns(const uint8_t *samples, int nsamp, uint8_t *col, int ncols);

/* Emit one frame as line strings via the callback 'emit' (each call = one
 * line to send on Serial1, WITHOUT trailing CRLF). Produces:
 *   "la begin <ncols> <nch> <rate>", N x "la d <off> <hex>", "la end".  */
typedef void (*bvm_line_fn)(const char *line, void *ctx);
void bvm_emit_frame(const uint8_t *col, int ncols, int nch, uint32_t rate,
                    bvm_line_fn emit, void *ctx);

/* Emit one oscilloscope frame as "sc begin/d <ch> <off>/end" wire lines.
 * colA/colB are 8-bit sample columns (colB ignored when nch==1). */
void sc_emit_frame(const uint8_t *colA, const uint8_t *colB, int ncols, int nch,
                   uint32_t rate, bvm_line_fn emit, void *ctx);

#endif
