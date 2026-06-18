/* la_sim.c — animated 8-channel logic scene (see la_sim.h). */
#include "la_sim.h"

void la_sim_fill(uint8_t *s, int n, uint32_t frame){
    int ph = (int)(frame * 3);             /* scroll speed (px/frame) */
    int duty = 30 + (int)(frame % 40);     /* L6 PWM duty breathes 30..69% */
    for(int i=0; i<n; i++){
        int x = i + ph;                    /* scrolling coordinate */
        uint8_t b = 0;
        if((x/4)  & 1) b |= 1<<0;          /* L0 fast clock          */
        if((x/8)  & 1) b |= 1<<1;          /* L1 /2                  */
        if((x/16) & 1) b |= 1<<2;          /* L2 /4                  */
        if((x/32) & 1) b |= 1<<3;          /* L3 /8                  */
        /* L4: chip-select-like window that slides across the screen */
        { int w0=(int)((frame*2) % n), w1=w0 + n/3;
          int xi=i; if(!(xi>w0 && xi<w1)) b |= 1<<4; }
        /* L5: a short data burst that moves left to right */
        { int b0=(int)((frame*4) % n); int p=i-b0;
          if(p>=0 && p<48){ if((p/6)&1) b|=1<<5; } else if(i>b0) b|=1<<5; }
        if((x % 100) < duty) b |= 1<<6;    /* L6 breathing PWM       */
        /* L7: lone marker pulse sweeping across */
        { int m=(int)((frame*5) % n); if(i>=m && i<m+4) b |= 1<<7; }
        s[i] = b;
    }
}
