/* bs05_logic.cpp — ESP32-S3 side. Depends on bvm.h/.cpp (validated) and an
 * IVcp backend. Arduino/FreeRTOS. The capture sequence is a direct port of
 * the BS05-tested scopething flow:
 *     set logic registers -> '>' program -> 'U' configure -> 'D' trace
 *     -> poll status until != Wait -> read write-address
 *     -> set dump registers -> '>' -> 'A' -> read N raw logic bytes
 * Each grab is decimated to BS_NCOLS columns and streamed as one la-frame.
 *
 * HARDWARE-CONFIRM SEAMS are marked [HW]. Everything else is host-proven.
 */
#include "bs05_logic.h"
#include "bvm.h"
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static IVcp        *g_vcp = nullptr;
static volatile bool g_run = false;

/* Capture mode. LOGIC = real BVM logic grab (hardware-proven). SCOPE = CHA/CHB
 * traces; the source below is a synthetic generator so the command path, wire
 * format and Pi scope renderer can be brought up on hardware now — the real
 * analog BVM grab (TraceMode AnalogChop, chop de-interleave, ConverterLo/Hi)
 * drops in behind bs05_scope_source() later, exactly as la_sim -> bs05_capture. */
enum bs05_mode_t { BS_MODE_LOGIC = 0, BS_MODE_SCOPE = 1 };
static volatile bs05_mode_t g_mode = BS_MODE_LOGIC;
static volatile uint8_t     g_scmask = 0x03;   /* bit0=A, bit1=B */
static volatile bool        g_scope_hw = false; /* false=SIM source, true=real BVM */

/* 64-entry quarter-friendly sine table, 0..255 (avoids float in the task). */
static const uint8_t SINE64[64] = {
  128,140,152,165,176,188,198,208,218,226,234,240,245,250,252,254,255,254,252,250,
  245,240,234,226,218,208,198,188,176,165,152,140,128,116,104, 91, 80, 68, 58, 48,
   38, 30, 22, 16, 11,  6,  4,  2,  1,  2,  4,  6, 11, 16, 22, 30, 38, 48, 58, 68,
   80, 91,104,116
};

/* Fill one display column buffer with a synthetic waveform.
 *   kind 0 = sine (~3 cycles), kind 1 = triangle (~2 cycles). phase animates. */
static void bs05_scope_source(uint8_t *col, int n, int kind, uint32_t phase){
    for(int x=0;x<n;x++){
        if(kind==0){
            uint32_t idx=((uint32_t)x*3*64/(uint32_t)n + phase) & 63;
            col[x]=SINE64[idx];
        } else {
            uint32_t t=((uint32_t)x*2*256/(uint32_t)n + phase*4);
            uint32_t tri=t & 511;                 /* 0..511 triangle */
            col[x]=(uint8_t)(tri<256 ? tri : 511-tri);
        }
    }
}

/* ---- low-level VCP helpers ------------------------------------------- */
/* [HW] The BVM echoes every command verbatim (scopething issue() semantics):
 * after writing, read back exactly n echoed bytes before any real reply/data.
 * Without this the echo desyncs the status poll AND the sample dump. */
static bool vcp_issue(const char *s, size_t n){
    if(!g_vcp) return false;
    g_vcp->write(s, n);
    size_t off = 0; uint32_t t0 = millis();
    while(off < n){
        uint8_t buf[80];
        size_t want = n - off; if(want > sizeof buf) want = sizeof buf;
        int r = g_vcp->read(buf, want, 50);            /* discard the echo */
        if(r > 0){ off += (size_t)r; t0 = millis(); }
        else if(millis() - t0 > 300) return false;     /* echo timed out */
    }
    return true;
}
static bool vcp_issue1(char c){ return vcp_issue(&c, 1); }

/* scopething issue_reset: send '!' then read until the banner ends with '!'. */
static void vcp_reset(void){
    if(!g_vcp) return;
    char bang = '!';
    g_vcp->write(&bang, 1);
    uint8_t buf[64]; uint32_t t0 = millis();
    for(;;){
        int r = g_vcp->read(buf, sizeof buf, 50);
        if(r > 0 && buf[r-1] == '!') break;            /* reset banner ends with '!' */
        if(millis() - t0 > 600) break;
    }
}

/* Drain RX until the stream is quiet for quiet_ms (clears stray echoes/replies
 * so the next read starts clean). */
static void vcp_drain(uint32_t quiet_ms){
    if(!g_vcp) return;
    uint8_t junk[128];
    while(g_vcp->read(junk, sizeof junk, quiet_ms) > 0){ /* keep draining */ }
}

/* [HW] BVM reply framing: the VM echoes results as ASCII hex terminated by
 * CR. Read one hex token (skips spaces/echoes), returns its integer value,
 * or -1 on timeout. Confirm the exact terminator on your device. */
static long vcp_read_hex_token(uint32_t timeout_ms){
    char tok[12]; int n=0; uint8_t ch;
    uint32_t t0=millis();
    /* Data hex from the VM is LOWERCASE; command echoes (U,D,A,>) are uppercase
     * or symbols, so only 0-9/a-f count as token chars (echoes are skipped). */
    for(;;){
        if(g_vcp->read(&ch,1,10)==1){
            int hexd=(ch>='0'&&ch<='9')||(ch>='a'&&ch<='f');
            if(hexd){ tok[n++]=(char)ch; break; }
        }
        if(millis()-t0>timeout_ms) return -1;
    }
    while(n<11){
        if(g_vcp->read(&ch,1,10)==1){
            int hexd=(ch>='0'&&ch<='9')||(ch>='a'&&ch<='f');
            if(hexd){ tok[n++]=(char)ch; continue; }
            break;                                 /* CR / non-hex ends token */
        }
        if(millis()-t0>timeout_ms) break;
    }
    tok[n]=0; if(!n) return -1;
    return strtol(tok,nullptr,16);
}

/* ---- DIAGNOSTIC: read one VM register (scopething get_register) ---------
 * issue "<addr>@p" (set address + peek), then read 2 CR-terminated replies;
 * the value is the SECOND reply. All registers we dump are single-byte. */
static long vcp_read_reg(uint8_t addr){
    if(!g_vcp) return -1;
    static const char hx[] = "0123456789abcdef";
    char c[4] = { hx[(addr>>4)&0xF], hx[addr&0xF], '@', 'p' };
    vcp_drain(10);                              /* clean slate */
    if(!vcp_issue(c, 4)) return -1;             /* set addr + peek (+ echo) */
    long r0 = vcp_read_hex_token(200);          /* reply 1 (address)        */
    long r1 = vcp_read_hex_token(200);          /* reply 2 = value          */
    (void)r0;
    return r1;
}

/* Dump the registers that route peripherals onto the logic channels, so we
 * can see exactly what is driving L5 (and L4/L6/L7) on THIS device. */
void bs05_dump_regs(void){
    static const struct { const char *name; uint8_t addr; } R[] = {
        {"SpockOption ",0x07},{"LogicControl",0x74},
        {"KitchenSinkA",0x7b},{"KitchenSinkB",0x7c},
        {"Control     ",0x86},{"Cmd         ",0x46},{"Mode        ",0x47},
        {"Map0        ",0x94},{"Map1        ",0x95},{"Map2        ",0x96},
        {"Map3        ",0x97},{"Map4        ",0x98},{"Map5        ",0x99},
        {"Map6        ",0x9a},{"Map7        ",0x9b},
    };
    Serial.println("[bs05] ---- register dump ----");
    for(unsigned i=0;i<sizeof R/sizeof R[0];i++){
        long v = vcp_read_reg(R[i].addr);
        if(v < 0) Serial.printf("[bs05]   %s @0x%02x = (read failed)\n", R[i].name, R[i].addr);
        else      Serial.printf("[bs05]   %s @0x%02x = 0x%02x\n", R[i].name, R[i].addr, (unsigned)(v & 0xFF));
    }
    Serial.println("[bs05] ---------------------------");
}

/* ---- one logic grab into 'samples' (length cfg.nsamples) ------------- */
static bool bs05_capture(uint8_t *samples, bvm_logic_cfg_t *cfg){
    char cmd[640];
    vcp_drain(15);                           /* clear any stale bytes first */

    size_t n = bvm_logic_setup_cmd(cfg, BS_DIGITAL, cmd, sizeof cmd);
    if(!vcp_issue(cmd, n)) return false;     /* set logic regs (+ echo)   */
    if(!vcp_issue1('>'))   return false;     /* program spock registers   */
    if(!vcp_issue1('U'))   return false;     /* configure hardware        */
    if(!vcp_issue1('D'))   return false;     /* triggered trace           */

    /* [HW] poll status: replies are (code, timestamp) CR-terminated hex */
    long code = TS_WAIT, ts = 0, guard = 0;
    while(code == TS_WAIT && guard++ < 64){
        code = vcp_read_hex_token(200);
        ts   = vcp_read_hex_token(200);
        if(code < 0){ vcp_issue1('K'); return false; }  /* timeout -> cancel */
    }
    (void)ts;
    long addr = vcp_read_hex_token(200);                 /* write pointer */
    if(addr < 0) return false;

    n = bvm_logic_dump_cmd(cfg, (uint32_t)addr, cmd, sizeof cmd);
    if(!vcp_issue(cmd, n)) return false;     /* set dump regs (+ echo)    */
    if(!vcp_issue1('>'))   return false;     /* program                   */

    /* Flush every stray echo/reply, THEN issue the dump: after this the only
     * bytes in the pipe are the 'A' echo + the raw sample stream. */
    vcp_drain(15);
    if(!vcp_issue1('A')) return false;       /* dump cmd; consumes 'A' echo */

    int need = (int)cfg->nsamples, off = 0; uint32_t t0 = millis();
    while(off < need){
        int r = g_vcp->read(samples + off, need - off, 200);
        if(r > 0){ off += r; t0 = millis(); }
        else if(millis() - t0 > 800) break;             /* dump timeout */
    }
    return off == need;
}

/* ---- one analog grab into colA/colB (cfg.nsamples/nch samples each) ----
 * [HW] Real CHA/CHB capture. Mirrors bs05_capture: setup -> '>'/'U'/'D' ->
 * poll -> addr -> per-channel dump ('>' then 'A'). In chop mode each channel is
 * a separate DumpChan dump, so no de-interleave here. The ADC window
 * (BS_SC_CONV_LO/HI) is uncalibrated and needs tuning on real hardware. */
static bool bs05_scope_capture(uint8_t *colA, uint8_t *colB, int nch,
                               uint8_t mask, bvm_logic_cfg_t *cfg){
    char cmd[640];
    /* [HW bring-up] throttle a step trace to ~once/2s so the serial isn't
     * flooded while the task retries every frame period. */
    static uint32_t lastdbg = 0;
    bool dbg = (millis() - lastdbg > 2000); if(dbg) lastdbg = millis();
    #define SCDBG(...) do{ if(dbg) Serial.printf(__VA_ARGS__); }while(0)

    vcp_drain(15);
    size_t n = bvm_scope_setup_cmd(cfg, mask, nch, BS_SC_CONV_LO, BS_SC_CONV_HI,
                                   cmd, sizeof cmd);
    if(!vcp_issue(cmd, n)){ SCDBG("[scope] FAIL setup echo\n"); return false; }
    if(!vcp_issue1('>')) { SCDBG("[scope] FAIL '>'\n");        return false; }
    if(!vcp_issue1('U')) { SCDBG("[scope] FAIL 'U'\n");        return false; }
    if(!vcp_issue1('D')) { SCDBG("[scope] FAIL 'D'\n");        return false; }

    /* Poll trace status. Replies are CR-terminated lowercase hex (code,ts);
     * uppercase command echoes (U,D) are skipped by the reader. Free-run analog
     * waits for the comparator trigger, so we rely on the auto-timeout: loop
     * until a non-WAIT code (TS_AUTO/DONE) or a wall-clock deadline. */
    /* Free-running capture. Start the trace, give the circular buffer ~250ms to
     * fill, then cancel ('K') and read whatever's there. A real comparator
     * crossing completes it sooner (TS_DONE); otherwise the cancel gives TS_STOP
     * with a valid write pointer. Replies are lowercase hex (code,ts); the 'K'
     * echo (uppercase) is skipped by the reader. */
    long code = TS_WAIT; uint32_t pollstart = millis(); bool cancelled = false;
    while((millis() - pollstart) < 1500){
        long c0 = vcp_read_hex_token(100);
        if(c0 >= 0){
            (void)vcp_read_hex_token(100);       /* timestamp */
            code = c0;
            if(code != TS_WAIT) break;
        }
        if(!cancelled && (millis() - pollstart) > 250){
            char k = 'K'; g_vcp->write(&k, 1);   /* cancel -> grab the buffer */
            cancelled = true;
        }
    }
    SCDBG("[scope] trace code=%ld cancelled=%d\n", code, (int)cancelled);
    if(code == TS_WAIT){ SCDBG("[scope] FAIL no completion\n"); return false; }
    /* DONE(0)/AUTO(1)/STOP(3) all leave a usable buffer */

    long addr = vcp_read_hex_token(300);
    if(addr < 0){ SCDBG("[scope] FAIL addr read\n"); return false; }
    SCDBG("[scope] addr=%ld\n", addr);

    int asamples = (int)(cfg->nsamples / (nch>0?nch:1));
    for(int ch = 0; ch < nch; ch++){
        uint8_t *dst = (ch==0) ? colA : colB;
        n = bvm_scope_dump_cmd(cfg, (uint32_t)addr, ch, nch, cmd, sizeof cmd);
        if(!vcp_issue(cmd, n)){ SCDBG("[scope] FAIL dump%d echo\n", ch); return false; }
        if(!vcp_issue1('>'))  { SCDBG("[scope] FAIL dump%d '>'\n", ch); return false; }
        vcp_drain(15);
        if(!vcp_issue1('A'))  { SCDBG("[scope] FAIL dump%d 'A'\n", ch); return false; }
        int off = 0; uint32_t t0 = millis();
        while(off < asamples){
            int r = g_vcp->read(dst + off, asamples - off, 200);
            if(r > 0){ off += r; t0 = millis(); }
            else if(millis() - t0 > 800) break;
        }
        SCDBG("[scope] dump%d read %d/%d (first=%02x)\n", ch, off, asamples,
              off>0 ? dst[0] : 0);
        if(off != asamples) return false;
    }
    SCDBG("[scope] OK\n");
    #undef SCDBG
    return true;
}

/* ---- public control -------------------------------------------------- */
void bs05_begin(IVcp *vcp){
    g_vcp = vcp;
    g_vcp->begin();
    g_vcp->setBaud(BS_BAUD);
    /* VM reset is done in bs05_task on the first (re)connect, when the device
     * is actually present on the bus. */
}
void bs05_start(void){ g_mode = BS_MODE_LOGIC; g_run = true; }
void bs05_scope(uint8_t chmask){
    g_scmask = chmask ? (chmask & 0x03) : 0x03;   /* 0 => both A+B */
    g_scope_hw = false;                            /* synthetic source */
    g_mode   = BS_MODE_SCOPE;
    g_run    = true;
}
void bs05_scope_hw(uint8_t chmask){
    g_scmask = chmask ? (chmask & 0x03) : 0x03;
    g_scope_hw = true;                             /* real analog capture */
    g_mode   = BS_MODE_SCOPE;
    g_run    = true;
}
void bs05_stop (void){ g_run = false; }
bool bs05_running(void){ return g_run; }

/* ---- the streaming task --------------------------------------------- */
static bs05_send_fn g_send = nullptr;
static void emit_adapter(const char *line, void *ctx){ (void)ctx; if(g_send) g_send(line); }

void bs05_task(void *pv){
    g_send = (bs05_send_fn)pv;
    static uint8_t samples[BS_NSAMPLES];
    static uint8_t col[256];
    static uint8_t scA[BS_SC_NCOLS], scB[BS_SC_NCOLS];
    static uint32_t scphase = 0;
    const TickType_t period = pdMS_TO_TICKS(1000 / (BS_FPS < 1 ? 1 : BS_FPS));
    bool was_connected = false;

    for(;;){
        bool conn = g_vcp && g_vcp->connected();
        if(conn && !was_connected){ vcp_reset(); }   /* fast reset on (re)connect */
        was_connected = conn;

        if(!g_run || !conn){ vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        if(g_mode == BS_MODE_SCOPE){
            int nch = (g_scmask == 0x03) ? 2 : 1;
            if(g_scope_hw){
                /* real analog BVM capture; needs the device + register tuning */
                if(!conn){ vTaskDelay(pdMS_TO_TICKS(50)); continue; }
                bvm_logic_cfg_t cfg;
                bvm_scope_plan(BS_SC_RATE_HZ, (uint16_t)(BS_SC_NCOLS*nch), nch, &cfg);
                if(!bs05_scope_capture(scA, scB, nch, g_scmask, &cfg)){
                    static uint32_t fcnt = 0;
                    if((fcnt++ % 4) == 0)
                        Serial.println("[scope] hw capture failed (tune ConverterLo/Hi, check probes)");
                    vTaskDelay(period); continue;
                }
            } else {
                /* synthetic source (no hardware needed) */
                if(g_scmask == 0x03){
                    bs05_scope_source(scA, BS_SC_NCOLS, 0, scphase);   /* A=sine    */
                    bs05_scope_source(scB, BS_SC_NCOLS, 1, scphase);   /* B=triangle*/
                } else if(g_scmask == 0x02){
                    bs05_scope_source(scA, BS_SC_NCOLS, 1, scphase);   /* B only    */
                } else {
                    bs05_scope_source(scA, BS_SC_NCOLS, 0, scphase);   /* A only    */
                }
                scphase++;
            }
            sc_emit_frame(scA, scB, BS_SC_NCOLS, nch, BS_SC_RATE_HZ,
                          emit_adapter, nullptr);
            vTaskDelay(period);
            continue;
        }

        bvm_logic_cfg_t cfg;
        bvm_logic_plan(BS_RATE_HZ, BS_NSAMPLES, &cfg);

        if(bs05_capture(samples, &cfg)){
            /* TEMP: confirm the dump is now real logic (constant bytes when
             * idle), not echoed command text. Remove once verified. */
            static uint32_t dbgn = 0;
            if((dbgn++ % 8) == 0){
                Serial.printf("[bs05] raw:");
                for(int i = 0; i < 16; i++) Serial.printf(" %02x", samples[i]);
                Serial.println();
            }
            int nc = bvm_pack_columns(samples, cfg.nsamples, col, BS_NCOLS);
            bvm_emit_frame(col, nc, 8, cfg.actual_rate, emit_adapter, nullptr);
        }
        vTaskDelay(period);
    }
}
