#include "Algorithms.h"


// FIR Filter: -----------------------------------------------

FIRFilter::FIRFilter() {
    wptr = nullptr;
    x = nullptr;
    N = 0;
}

FIRFilter::FIRFilter(int mem, float *ptrw, float *ptrx) {
    wptr = ptrw;
    x = ptrx;
    N = mem;
    reset();
}

void FIRFilter::setMem(int mem) {
    N = mem;
    reset();
}

void FIRFilter::setMemAndPointers(int mem, float *ptrw, float *ptrx) {
    wptr = ptrw;
    x = ptrx;
    N = mem;
    reset();
}

void FIRFilter::reset() {
    y = 0;
    for (int k = 0; k < N; k++) { *(x+k) = 0; }
    ptr = N-1;
}

float FIRFilter::filter(float xn) {
    ptr++;
    if (ptr >= N) { ptr = 0; }
    *(x+ptr) = xn;
    y = 0;
    for (int k = 0; k < N; k++) {
    y = y + *(x+((ptr-k+N)%N)) * *(wptr+k);
    //x[(ptr-k+N)%N] * *(wptr+k);
    }
    return y;
}

// FIR SVD Filter: -------------------------------------------------

FIRFilterSVD::FIRFilterSVD() {
    wptr = nullptr;
    x = nullptr;
    N = 0;
    R = 0;
    C = 0;
    B = 0;
    ptr = 0;
}

FIRFilterSVD::FIRFilterSVD(int mem, int nbranches, int nR, int nC, float *ptrw, float *ptrx) {
    wptr = ptrw;
    x = ptrx;
    N = mem;
    R = nR;
    C = nC;
    B = nbranches;
    reset();
}

void FIRFilterSVD::setAllParams(int mem, int nbranches, int nR, int nC, float *ptrw, float *ptrx) {
    wptr = ptrw;
    x = ptrx;
    N = mem;
    R = nR;
    C = nC;
    B = nbranches;
    reset();
}

void FIRFilterSVD::setParams(int mem, int nbranches, int nR, int nC) {
    N = mem;
    R = nR;
    C = nC;
    B = nbranches;
    reset();
}

void FIRFilterSVD::reset() {
    y = 0;
    for (int k = 0; k < N; k++) { *(x+k) = 0; }
    ptr = N-1;
    for (int k = 0; k < B; k++) {
        firbranches[k].setMemAndPointers(R, wptr + B*C + k*R, buffers + k*R);
    }
}

float FIRFilterSVD::filter(float xn) {
    ptr++;
    if (ptr >= N) { ptr = 0; }
    *(x+ptr) = xn;
    y = 0;
    for (int bb = 0; bb < B; bb++) {
        float aux = 0.0;
        float * wcptr = wptr + bb*C;
        for (int cc = 0; cc < C; cc++) {
            aux = aux + *(x+((ptr-(cc*R)+N)%N)) * *(wcptr++);
        }
        y = y + firbranches[bb].filter(aux);
    }
    return y;
}


// FxNLMS algorithm: -----------------------------------------------

FxNLMS::FxNLMS(int mem, float *wa, float *xa, float *xasec, float (*fsecfilter)(float), float *deltawa) {
    N = mem;
    mu = 0.25;
    fi = 1e-4;
    w = wa;
    deltaw = deltawa;
    x = xa;
    xsec = xasec;
    filtsec =  fsecfilter;
    reset();
}

void FxNLMS::setParameters(int mem, float muu, float fii) {
    N = mem;
    mu = muu;
    fi = fii;
    reset();
}

void FxNLMS::reset() {
    for (int k = 0; k < N; k++) {
    *(x+k) = 0;
    *(xsec+k) = 0;
    *(w+k) = 0; 
    *(deltaw+k) = 0;
    }
    y = 0;
    ptr = N-1;
    // filtsec->reset();
}

float FxNLMS::filter(float xn) {      
    ptr++;
    if (ptr >= N) { ptr = 0; }
    *(xsec+ptr) = (*filtsec)(xn);
    *(x+ptr) = xn;
    y = 0;
    for (int k = 0; k < N; k++) {
    y = y + *(x+((ptr-k+N)%N)) * *(w+k);
    }
    return y;
}

void FxNLMS::update(float en) {
    normterm = fi;
    for (int k = 0; k < N; k++) {
    aux = *(xsec+k);
    normterm = normterm + aux*aux;
    }
    aux = mu * (en / normterm);
    for (int k = 0; k < N; k++) {
    *(w+k) = *(w+k) - aux * *(xsec+((ptr-k+N)%N));
    }
}

void FxNLMS::updateStep1() {
    normterm = fi;
    for (int k = 0; k < N; k++) {
        aux = *(xsec+k);
        normterm = normterm + aux*aux;
    }
    aux = mu / normterm;
    for (int k = 0; k < N; k++) {
        *(deltaw+k) = aux * *(xsec+((ptr-k+N)%N));
    }
}

void FxNLMS::updateStep2(float en) {
    for (int k = 0; k < N; k++) {
        *(w+k) = *(w+k) - en * *(deltaw+k);
    }
}


// TAFxNLMS algorithm: -----------------------------------------------

CVAFxNLMS::CVAFxNLMS(int mem, float *wa, float *xa, float *wa2, float *xa2, float *xasec, float *xasec2, FIRFilter *fsec, FIRFilter *fsec2, float *deltawa, float *deltawa2) {
    N = mem;
    mu = 0.25;
    fi = 1e-4;
    w = wa;
    w2 = wa2;
    deltaw = deltawa;
    deltaw2 = deltawa2;
    x = xa;
    x2 = xa2;
    xsec = xasec;
    xsec2 = xasec2;
    filtsec = fsec;
    filtsec2 = fsec2;
    reset();
}

void CVAFxNLMS::setParameters(int mem, float muu, float fii) {
    N = mem;
    mu = muu;
    fi = fii;
    reset();
}

void CVAFxNLMS::reset() {
    for (int k = 0; k < N; k++) {
        *(x+k) = 0;
        *(x2+k) = 0;
        *(xsec+k) = 0;
        *(xsec2+k) = 0;
        *(w+k) = 0; 
        *(w2+k) = 0;
        *(deltaw+k) = 0; 
        *(deltaw2+k) = 0;
    }
    y = 0;
    ptr = N-1;
}

float CVAFxNLMS::filter(float xn,float xn2) {      
    ptr++;
    if (ptr >= N) { ptr = 0; }
    *(xsec+ptr) = (*filtsec).filter(xn);
    *(xsec2+ptr) = (*filtsec2).filter(xn2);
    *(x+ptr) = xn;
    *(x2+ptr) = xn2;
    y = 0;
    for (int k = 0; k < N; k++) {
        y = y + *(x+((ptr-k+N)%N)) * *(w+k) + *(x2+((ptr-k+N)%N)) * *(w2+k);
    }
    return y;
}

void CVAFxNLMS::update(float en) {
    normterm = fi;
    for (int k = 0; k < N; k++) {
        aux = *(xsec+k);
        aux2 = *(xsec2+k);
        normterm = normterm + aux*aux + aux2*aux2;
    }
    aux = mu * (en / normterm);
    for (int k = 0; k < N; k++) {
        *(w+k) = *(w+k) - aux * *(xsec+((ptr-k+N)%N));
        *(w2+k) = *(w2+k) - aux * *(xsec2+((ptr-k+N)%N));
    }
}

void CVAFxNLMS::updateStep1() {
    normterm = fi;
    for (int k = 0; k < N; k++) {
        aux = *(xsec+k);
        aux2 = *(xsec2+k);
        normterm = normterm + aux*aux + aux2*aux2;
    }
    aux = mu / normterm;
    for (int k = 0; k < N; k++) {
        *(deltaw+k) = aux * *(xsec+((ptr-k+N)%N));
        *(deltaw2+k) = aux * *(xsec2+((ptr-k+N)%N));
    }
}

void CVAFxNLMS::updateStep2(float en) {
    for (int k = 0; k < N; k++) {
        *(w+k) = *(w+k) - en * *(deltaw+k);
        *(w2+k) = *(w2+k) - en * *(deltaw2+k);
    }
}