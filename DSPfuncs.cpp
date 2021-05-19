#include <Arduino.h>
#include <math.h>
#include "DSPfuncs.h"

// DCRemover: ------------------------------------

DCRemover::DCRemover(float alp) {
    y = 0;
    alpha = alp;
    wn = 0;
    wn1 = 0;
}

void DCRemover::reset() {
    wn = 0;
    wn1 = 0;
}

float DCRemover::filter(float x) {
    wn1 = wn;
    wn = x + alpha * wn1;
    y = wn - wn1;
    return y;
} 

// -----------------------------------------------


// Signal Generator: --------------------------------------------------------------------

SignalGenerator::SignalGenerator(float fsampling, float tsampling, int stdzlevel) {
    F_SAMPLE = fsampling;
    T_SAMPLE = tsampling;
    Z_LEVEL = stdzlevel;
    STD_Z_LEVEL = stdzlevel;
    levelscaler = 1;
    sincnst = 0;
    n = 0;
    ctinit = 0;
    cdeltai = 0;
    ctfim = 0;
    cdeltaf = 0;
    ff = 0;
    AA = 0;
    integ = 0;
    cA = 0;
    A2 = 0;
    AA2 = 0;
    integ2 = 0;
    cA2 = 0;
    type = 0;
    A = 0.0; // 0 a 256
    f = 10.0; // Em Hertz
    last = 0;
    enabled = false;
}

void SignalGenerator::setType(int tp, float Amp, float freq, int dclevel) {
    type = tp;
    Z_LEVEL = dclevel;
    levelscaler = ((float)dclevel) / ((float)STD_Z_LEVEL);
    Amp = levelscaler * Amp; 
    if (type == 0) {
        A = Amp;
    } else if ((type == 1) || (type == 2)) {
        A = Amp;
        f = freq;
        sincnst = 2.0 * M_PI * f * T_SAMPLE; 
        n = -1.0;
        integ = 0.0;
        cA = Amp/(2*M_PI*freq*2*M_PI*freq);
    }
    if (Amp == 0.0) { 
        enabled = false; 
        last = 0;
    } else { 
        enabled = true; 
    }
}

void SignalGenerator::setChirpParams(int ti, int di, int tf, int df, int a2) {
    ctinit = (float)(ti) * F_SAMPLE;
    cdeltai = (float)(di) * F_SAMPLE / 10.0;
    ctfim = (float)(tf) * F_SAMPLE;
    cdeltaf = (float)(df) * F_SAMPLE / 10.0;
    A2 = levelscaler * (float)a2;
    cA2 = A2/(2*M_PI*f*2*M_PI*f);
}

int SignalGenerator::next() {
      if (type == 1) {
        n = n + 1.0;
        // last = (int)round(A * sin(sincnst*n)) + 128;
        last = Z_LEVEL - (int)round(A * sin(sincnst*n));
        return last;
      } else if (type == 0) {
        last = (int)round(A);
        // last = random( (last<<1) + 1 ) + (128 - last);
        last = Z_LEVEL - (random( (last<<1) + 1 ) - last);
        return last;
      } else if (type == 2) {
        n = n + 1.0;
        if ( (n < ctinit) || (n > (ctfim+cdeltaf)) ) {
          ff = 0;
        } else if ( n <= (ctinit+cdeltai) ) {
          ff = f * (n-ctinit) / cdeltai;
        } else if ( (ctfim != 0) && (n >= ctfim) && ( n <= (ctfim+cdeltaf) ) ) {
          ff = f - f * (n-ctfim) / cdeltaf;
        }
        AA = 4.0*(M_PI*ff*M_PI*ff) * cA;
        AA2 = 4.0*(M_PI*ff*M_PI*ff) * cA2;
        integ = integ + 2.0 * M_PI * ff * T_SAMPLE;
        //integ2 = integ2 + 2.0 * M_PI * (2*ff) * T_SAMPLE;
        //last = (int)round( AA * sin(integ) ) + (int)round(AA2 * sin(2*integ))  + 128;
        last = Z_LEVEL - ( (int)round( AA * sin(integ) ) + (int)round(AA2 * sin(2*integ)) );
        return last;
      }
}

// --------------------------------------------------------------------------------------