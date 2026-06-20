/* bvm.cpp — see bvm.h. Pure C++; no Arduino/USB dependencies. */
#include "bvm.h"

static const char HEX[]="0123456789abcdef";
static inline void put2(char *&p, uint8_t b){ *p++=HEX[b>>4]; *p++=HEX[b&0xF]; }

/* Faithful port of scopething Vm.set_registers() command compression. */
size_t bvm_build_set_registers(const bvm_reg_t *regs, int n, char *out, size_t outsz){
    /* sort indices by base (small n -> insertion sort) */
    int idx[32]; if(n>32) n=32;
    for(int i=0;i<n;i++) idx[i]=i;
    for(int i=1;i<n;i++){ int k=idx[i]; int j=i-1;
        while(j>=0 && regs[idx[j]].base>regs[k].base){ idx[j+1]=idx[j]; j--; } idx[j+1]=k; }

    char *p=out, *end=out+outsz-2;
    int register0=-1, register1=-1; int have=0;
    for(int ri=0; ri<n; ri++){
        const bvm_reg_t *r=&regs[idx[ri]];
        for(int i=0;i<r->width && p<end-6;i++){
            uint8_t byte=(uint8_t)((r->value >> (8*i)) & 0xFF);   /* little-endian */
            if(have){ *p++='z'; register1++; }
            int address=r->base+i;
            if(register1<0 || address > register1+3){
                put2(p,(uint8_t)address); *p++='@'; register0=register1=address;
            } else {
                for(int k=0;k<address-register1;k++) *p++='n';
                register1=address;
            }
            if(byte!=register0){
                if(byte==0) *p++='[';
                else put2(p,byte);
                register0=byte;
            }
            have=1;
        }
    }
    if(have) *p++='s';
    *p=0;
    return (size_t)(p-out);
}

void bvm_logic_plan(uint32_t target_rate, uint16_t nsamples, bvm_logic_cfg_t *c){
    if(target_rate<1) target_rate=1;
    uint32_t ticks=(BVM_PRIMARY_CLOCK + target_rate/2)/target_rate;   /* round */
    if(ticks<1) ticks=1;
    if(ticks>16384) ticks=16384;
    if(nsamples<2) nsamples=2;
    if(nsamples>BVM_BUFFER_BYTES) nsamples=BVM_BUFFER_BYTES;
    c->ticks=(uint16_t)ticks;
    c->nsamples=nsamples;
    c->actual_rate=BVM_PRIMARY_CLOCK/ticks;
    /* free-run: capture entirely post-trigger, immediate trigger (mask=0) */
    c->trace_intro=0;
    c->trace_outro=(uint16_t)(nsamples-1);
    /* small auto-timeout so a stalled trace still returns (in auto-ticks) */
    c->timeout=1000;
}

size_t bvm_logic_setup_cmd(const bvm_logic_cfg_t *c, uint8_t digital_enable,
                           char *out, size_t outsz){
    bvm_reg_t regs[] = {
        { R_TraceMode,    1, TRACE_LOGIC },
        { R_BufferMode,   1, BUFMODE_SINGLE },
        { R_SampleAddress,3, 0 },
        { R_ClockTicks,   2, c->ticks },
        { R_ClockScale,   2, 1 },
        { R_TriggerLogic, 1, 0 },
        { R_TriggerMask,  1, 0 },          /* 0 = don't-care => immediate */
        { R_TraceIntro,   2, c->trace_intro },
        { R_TraceOutro,   2, c->trace_outro },
        { R_TraceDelay,   4, 0 },
        { R_Timeout,      2, c->timeout },
        { R_TriggerIntro, 2, 0 },
        { R_TriggerOutro, 2, 0 },
        { R_Prelude,      2, 0 },
        { R_SpockOption,  1, 0 },
        { R_AnalogEnable, 1, 0 },
        { R_DigitalEnable,1, digital_enable },
    };
    return bvm_build_set_registers(regs, (int)(sizeof regs/sizeof regs[0]), out, outsz);
}

size_t bvm_logic_dump_cmd(const bvm_logic_cfg_t *c, uint32_t write_addr,
                          char *out, size_t outsz){
    uint32_t start=(write_addr - c->nsamples) % BVM_BUFFER_BYTES;
    bvm_reg_t regs[] = {
        { R_SampleAddress,3, start },
        { R_DumpMode,     1, DUMP_RAW },
        { R_DumpChan,     1, 128 },        /* 128 = packed logic byte stream */
        { R_DumpCount,    2, c->nsamples },
        { R_DumpRepeat,   2, 1 },
        { R_DumpSend,     2, 1 },
        { R_DumpSkip,     2, 0 },
    };
    return bvm_build_set_registers(regs, (int)(sizeof regs/sizeof regs[0]), out, outsz);
}

/* ---- analog (scope) builders ----------------------------------------- */
void bvm_scope_plan(uint32_t target_rate, uint16_t nsamples, int nch,
                    bvm_logic_cfg_t *c){
    bvm_logic_plan(target_rate, nsamples, c);     /* same ticks math          */
    if(nch==2) c->nsamples &= ~1u;                /* chop needs an even count */
    c->trace_intro=0;
    c->trace_outro=(uint16_t)(c->nsamples-1);
    c->timeout = 400;     /* faster auto fallback (~<1s) if no trigger crossing */
}

size_t bvm_scope_setup_cmd(const bvm_logic_cfg_t *c, uint8_t analog_mask, int nch,
                           uint16_t conv_lo, uint16_t conv_hi,
                           char *out, size_t outsz){
    /* Arm the hardware comparator so the trace actually triggers. Prefer CHA;
     * use CHB only when A isn't enabled. Trigger at mid-scale "above": a
     * periodic signal crosses it every cycle -> TS_DONE; the auto-timeout is
     * the fallback for flat inputs. (scopething trigger='A'/'B', type='above'.) */
    bool trigA = (analog_mask & 0x01) != 0;
    uint8_t spock  = 0x01 | (trigA ? 0x00 : 0x04);   /* HW comparator + source  */
    uint8_t tlogic = trigA ? 0x80 : 0x40;
    uint8_t tmask  = (uint8_t)(0xff ^ tlogic);
    uint8_t ksa    = trigA ? 0x80 : 0x40;            /* comparator enable A/B   */
    bvm_reg_t regs[] = {
        { R_TraceMode,    1, (uint32_t)((nch==2)?TRACE_ANALOGCHOP:TRACE_ANALOG) },
        { R_BufferMode,   1, (uint32_t)((nch==2)?(int)BUFMODE_CHOP:(int)BUFMODE_SINGLE) },
        { R_SampleAddress,3, 0 },
        { R_ClockTicks,   2, c->ticks },
        { R_ClockScale,   2, 1 },
        { R_TriggerLevel, 2, 0x8000 },     /* mid-scale comparator threshold  */
        { R_TriggerLogic, 1, tlogic },
        { R_TriggerMask,  1, tmask },
        { R_TraceIntro,   2, c->trace_intro },
        { R_TraceOutro,   2, c->trace_outro },
        { R_TraceDelay,   4, 0 },
        { R_Timeout,      2, c->timeout },
        { R_TriggerIntro, 2, 0 },
        { R_TriggerOutro, 2, 2 },
        { R_Prelude,      2, 0 },
        { R_SpockOption,  1, spock },
        { R_ConverterLo,  2, conv_lo },    /* U0.16 ADC window (needs tuning) */
        { R_ConverterHi,  2, conv_hi },
        { R_KitchenSinkA, 1, ksa },
        { R_KitchenSinkB, 1, KSB_ANALOG_FILTER },
        { R_AnalogEnable, 1, analog_mask },
        { R_DigitalEnable,1, 0 },
    };
    return bvm_build_set_registers(regs, (int)(sizeof regs/sizeof regs[0]), out, outsz);
}

size_t bvm_scope_dump_cmd(const bvm_logic_cfg_t *c, uint32_t write_addr,
                          int dump_chan, int nch, char *out, size_t outsz){
    uint32_t start=(write_addr - c->nsamples) % BVM_BUFFER_BYTES;
    uint16_t asamples=(uint16_t)(c->nsamples/(nch>0?nch:1));
    bvm_reg_t regs[] = {
        { R_SampleAddress,3, start },
        { R_DumpMode,     1, DUMP_RAW },
        { R_DumpChan,     1, (uint32_t)dump_chan },  /* 0=A, 1=B */
        { R_DumpCount,    2, asamples },
        { R_DumpRepeat,   2, 1 },
        { R_DumpSend,     2, 1 },
        { R_DumpSkip,     2, 0 },
    };
    return bvm_build_set_registers(regs, (int)(sizeof regs/sizeof regs[0]), out, outsz);
}

/* ---- wire framing ---------------------------------------------------- */
int bvm_pack_columns(const uint8_t *samples, int nsamp, uint8_t *col, int ncols){
    if(ncols>256) ncols=256;
    if(ncols<1) ncols=1;
    if(nsamp<=0) return 0;
    for(int x=0;x<ncols;x++){
        /* point-sample the column's source index (could OR a window instead) */
        long si=(long)x*nsamp/ncols;
        if(si>=nsamp) si=nsamp-1;
        col[x]=samples[si];
    }
    return ncols;
}

static void utoa10(uint32_t v, char *&p){
    char tmp[10]; int n=0; if(v==0) tmp[n++]='0';
    while(v){ tmp[n++]=(char)('0'+v%10); v/=10; }
    while(n) *p++=tmp[--n];
}

void bvm_emit_frame(const uint8_t *col, int ncols, int nch, uint32_t rate,
                    bvm_line_fn emit, void *ctx){
    char line[128];
    /* begin */
    { char *p=line; const char *s="la begin "; while(*s)*p++=*s++;
      utoa10((uint32_t)ncols,p); *p++=' '; utoa10((uint32_t)nch,p); *p++=' ';
      utoa10(rate,p); *p=0; emit(line,ctx); }
    /* data chunks: <=56 columns/line keeps "la d <off> "+112hex under 128 */
    const int PER=56;
    for(int off=0; off<ncols; off+=PER){
        int cnt=(ncols-off<PER)?(ncols-off):PER;
        char *p=line; const char *s="la d "; while(*s)*p++=*s++;
        utoa10((uint32_t)off,p); *p++=' ';
        for(int i=0;i<cnt;i++) put2(p, col[off+i]);
        *p=0; emit(line,ctx);
    }
    /* end */
    { const char *s="la end"; char *p=line; while(*s)*p++=*s++; *p=0; emit(line,ctx); }
}

/* Emit one scope frame: "sc begin <ncols> <nch> <rate>", then per channel
 * "sc d <ch> <off> <hex>" (<=56 samples/line), then "sc end". */
void sc_emit_frame(const uint8_t *colA, const uint8_t *colB, int ncols, int nch,
                   uint32_t rate, bvm_line_fn emit, void *ctx){
    char line[128];
    { char *p=line; const char *s="sc begin "; while(*s)*p++=*s++;
      utoa10((uint32_t)ncols,p); *p++=' '; utoa10((uint32_t)nch,p); *p++=' ';
      utoa10(rate,p); *p=0; emit(line,ctx); }
    const int PER=56;
    for(int ch=0; ch<nch; ch++){
        const uint8_t *src=(ch==0)?colA:colB;
        for(int off=0; off<ncols; off+=PER){
            int cnt=(ncols-off<PER)?(ncols-off):PER;
            char *p=line; const char *s="sc d "; while(*s)*p++=*s++;
            utoa10((uint32_t)ch,p); *p++=' ';
            utoa10((uint32_t)off,p); *p++=' ';
            for(int i=0;i<cnt;i++) put2(p, src[off+i]);
            *p=0; emit(line,ctx);
        }
    }
    { const char *s="sc end"; char *p=line; while(*s)*p++=*s++; *p=0; emit(line,ctx); }
}

/* Unmap the internal clock from L5 and stop the clock generator (scopething
 * stop_clock: set_registers(Map5=0, Cmd=1, Mode=0) then issue 'Z'). */
size_t bvm_stop_clock_cmd(char *out, size_t outsz){
    const bvm_reg_t regs[] = {
        { R_Map5, 1, 0 },   /* peripheral-pin-select ch5 = none -> L5 is logic */
        { R_Cmd,  1, 1 },   /* clock-generator command vector = stop           */
        { R_Mode, 1, 0 },
    };
    return bvm_build_set_registers(regs, (int)(sizeof regs/sizeof regs[0]), out, outsz);
}