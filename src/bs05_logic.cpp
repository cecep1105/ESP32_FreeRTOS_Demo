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
static void vcp_send(const char *s, size_t n){ if(g_vcp) g_vcp->write(s,n); }
static void vcp_send1(char c){ if(g_vcp) g_vcp->write(&c,1); }

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
    size_t n = bvm_logic_setup_cmd(cfg, BS_DIGITAL, cmd, sizeof cmd);
    vcp_send(cmd, n);
    vcp_send1('>');               /* program spock registers */
    vcp_send1('U');               /* configure hardware      */
    vcp_send1('D');               /* triggered trace (immediate: TriggerMask=0) */

    /* [HW] poll status: replies are (code, timestamp) hex; loop while Wait */
    long code = TS_WAIT, ts = 0, guard = 0;
    while(code == TS_WAIT && guard++ < 64){
        code = vcp_read_hex_token(200);
        ts   = vcp_read_hex_token(200);
        if(code < 0){ vcp_send1('K'); return false; }   /* timeout -> cancel */
    }
    (void)ts;
    long addr = vcp_read_hex_token(200);                 /* write pointer */
    if(addr < 0) return false;

    n = bvm_logic_dump_cmd(cfg, (uint32_t)addr, cmd, sizeof cmd);
    vcp_send(cmd, n);
    vcp_send1('>');
    vcp_send1('A');               /* binary buffer dump (logic = raw bytes) */

    /* read N raw bytes; the VCP read may deliver them in chunks */
    int need = (int)cfg->nsamples, off = 0; uint32_t t0 = millis();
    while(off < need){
        int r = g_vcp->read(samples + off, need - off, 200);
        if(r > 0) off += r;
        else if(millis() - t0 > 800) break;             /* dump timeout */
    }
    return off == need;
}

/* ---- public control -------------------------------------------------- */
void bs05_begin(IVcp *vcp){
    g_vcp = vcp;
    g_vcp->begin();
    g_vcp->setBaud(BS_BAUD);
    vcp_send1('!');                /* reset VM */
    vTaskDelay(pdMS_TO_TICKS(50));
    /* drain any reset banner */
    uint8_t junk[64]; while(g_vcp->read(junk,sizeof junk,20) > 0){}
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

    for(;;){
        if(!g_run || !g_vcp || !g_vcp->connected()){ vTaskDelay(pdMS_TO_TICKS(50)); continue; }

        bvm_logic_cfg_t cfg;
        bvm_logic_plan(BS_RATE_HZ, BS_NSAMPLES, &cfg);

        if(bs05_capture(samples, &cfg)){
            int nc = bvm_pack_columns(samples, cfg.nsamples, col, BS_NCOLS);
            bvm_emit_frame(col, nc, 8, cfg.actual_rate, emit_adapter, nullptr);
        }
        vTaskDelay(period);
    }
}
