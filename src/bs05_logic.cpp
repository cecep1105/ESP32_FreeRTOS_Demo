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
    /* skip leading non-hex noise */
    for(;;){
        if(g_vcp->read(&ch,1,10)==1){
            int hexd=(ch>='0'&&ch<='9')||(ch>='a'&&ch<='f')||(ch>='A'&&ch<='F');
            if(hexd){ tok[n++]=(char)ch; break; }
        }
        if(millis()-t0>timeout_ms) return -1;
    }
    while(n<11){
        if(g_vcp->read(&ch,1,10)==1){
            int hexd=(ch>='0'&&ch<='9')||(ch>='a'&&ch<='f')||(ch>='A'&&ch<='F');
            if(hexd){ tok[n++]=(char)ch; continue; }
            break;                                 /* CR / non-hex ends token */
        }
        if(millis()-t0>timeout_ms) break;
    }
    tok[n]=0; if(!n) return -1;
    return strtol(tok,nullptr,16);
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

/* ---- public control -------------------------------------------------- */
void bs05_begin(IVcp *vcp){
    g_vcp = vcp;
    g_vcp->begin();
    g_vcp->setBaud(BS_BAUD);
    /* VM reset is done in bs05_task on the first (re)connect, when the device
     * is actually present on the bus. */
}
void bs05_start(void){ g_run = true; }
void bs05_stop (void){ g_run = false; }
bool bs05_running(void){ return g_run; }

/* ---- the streaming task --------------------------------------------- */
static bs05_send_fn g_send = nullptr;
static void emit_adapter(const char *line, void *ctx){ (void)ctx; if(g_send) g_send(line); }

void bs05_task(void *pv){
    g_send = (bs05_send_fn)pv;
    static uint8_t samples[BS_NSAMPLES];
    static uint8_t col[256];
    const TickType_t period = pdMS_TO_TICKS(1000 / (BS_FPS < 1 ? 1 : BS_FPS));
    bool was_connected = false;

    for(;;){
        bool conn = g_vcp && g_vcp->connected();
        if(conn && !was_connected) vcp_reset();    /* reset VM once on (re)connect */
        was_connected = conn;

        if(!g_run || !conn){ vTaskDelay(pdMS_TO_TICKS(50)); continue; }

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
