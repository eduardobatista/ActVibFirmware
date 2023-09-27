#include <Arduino.h>
#include <math.h>
#include "DSPfuncs.h"


float evalPoly(float x, int order, float* coefs) {
  float y = *(coefs+order);
  float xp = 1.0;
  for (int i = (order-1); i >= 0; i--) {
    xp = xp * x;
    y = y + *(coefs+i) * xp;
  }
  return y;
}


// OutputScaler: ---------------------------------
OutputScaler::OutputScaler(int stdzlevel) {
  STD_Z_LEVEL = stdzlevel;
  dclevel = stdzlevel;
  levelscaler = (float)(dclevel-1);
}

void OutputScaler::adjust(int newdclevel) {
  dclevel = newdclevel;
  levelscaler = (float)(dclevel - 1);
}

int OutputScaler::evalOut(float valf) {
  lastwrittenout = ((int)roundf(levelscaler * valf));
  return lastwrittenout;
}

int OutputScaler::evalOutNoReg(float valf) {
  return ((int)roundf(levelscaler * valf));
}


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

SignalGenerator::SignalGenerator(float fsampling, float tsampling) {
    F_SAMPLE = fsampling;
    BASE_F_SAMPLE = fsampling;
    T_SAMPLE = tsampling;
    BASE_T_SAMPLE = tsampling;
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
    lastf = 0.0;
    enabled = false;
}

void SignalGenerator::setType(int tp, float Amp, float freq, float freqmult, float basefreq) {
  BASE_F_SAMPLE = basefreq;
  BASE_T_SAMPLE = 1.0/basefreq;
  setType(tp, Amp, freq, freqmult);
}

void SignalGenerator::setType(int tp, float Amp, float freq, float freqmult) {
    F_SAMPLE = freqmult * BASE_F_SAMPLE;
    T_SAMPLE = BASE_T_SAMPLE / freqmult;
    type = tp;
    if (type == 0) { // Noise
        A = Amp;
    } else if ((type == 1) || (type == 2)) { // Harmonic and Chirp
        A = Amp;
        f = freq;
        sincnst = 2.0 * M_PI * f * T_SAMPLE; 
        n = -1.0;
        integ = 0.0;
        cA = Amp/(2*M_PI*freq*2*M_PI*freq);
    } else if (type == 4) { // Square
        A = Amp;
        f = freq; 
        Psize2 = ((int)round(1.0/freq / T_SAMPLE)) >> 1; 
        n = -1.0;
        lastf = A;
    } 
    if (Amp == 0.0) { 
        enabled = false; 
        lastf = 0.0;
    } else { 
        enabled = true; 
    }
}

void SignalGenerator::setBaseFreq(float freq) {
    BASE_F_SAMPLE = freq;
    setFreqMult(1.0);  
}

void SignalGenerator::setFreqMult(float freqmult, float freq) {
    BASE_F_SAMPLE = freq;
    setFreqMult(freqmult);
}

void SignalGenerator::setFreqMult(float freqmult) {
    F_SAMPLE = freqmult * BASE_F_SAMPLE;
    T_SAMPLE = BASE_T_SAMPLE / freqmult;
    if ((type == 1) || (type == 2)) {
        sincnst = 2.0 * M_PI * f * T_SAMPLE; 
    } else if (type == 4) {
        Psize2 = ((int)round(1.0/f / T_SAMPLE)) >> 1; 
        lastf = A;
    }    
}

void SignalGenerator::setChirpParams(int ti, int di, int tf, int df, int a2) {
    ctinit = (float)(ti) * F_SAMPLE;
    cdeltai = (float)(di) * F_SAMPLE / 10.0;
    ctfim = (float)(tf) * F_SAMPLE;
    cdeltaf = (float)(df) * F_SAMPLE / 10.0;
    // A2 = levelscaler * (float)a2; // TODO!!!
    cA2 = A2/(2*M_PI*f*2*M_PI*f);
}

float SignalGenerator::next() {
      if (type == 1) {
        n = n + 1.0;
        lastf = A * sin(sincnst*n);
      } else if (type == 0) {
        lastaux = (int)roundf(A*4096.0);
        lastaux = random( (lastaux<<1) + 1 ) - lastaux;
        lastf = ((float)lastaux) / 4096.0;
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
        // TODO: Fix this to float output:
        // last = ( (int)round( AA * sin(integ) ) + (int)round(AA2 * sin(2*integ)) );
        lastf = ( roundf( AA * sin(integ) ) + roundf(AA2 * sin(2*integ)) );
        // lastforout = Z_LEVEL - last;
        // return last;
      } else if (type == 4) {
        n = n + 1.0;
        if ( ((int)n) % Psize2 == 0 ) {
          A = -A;
        }
        lastf = A;        
      } 
      return lastf;
}

// --------------------------------------------------------------------------------------