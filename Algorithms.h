#include <Arduino.h>

#ifndef Algorithms_h
#define Algorithms_h

class FIRFilter {

    private:
        int ptr = 0;

    public:
        float *wptr;
        float *x;
        float y;
        int N = 5;
        FIRFilter(int mem, float *ptrw, float *ptrx);
        void setMem(int mem);
        void reset();
        float filter(float xn);

};



class FxNLMS {

    private:
        int ptr = 0;

    public:
        float *wptr;
        float *x;
        float *xsec;
        float *w;
        float y;
        float mu;
        float fi;
        int N = 5;
        int Nsec = 5;
        FIRFilter *filtsec;
        float normterm;
        float aux;
        FxNLMS(int mem, float *wa, float *xa, float *xasec, FIRFilter *fsec);
        void setParameters(int mem, float muu, float fii);
        void reset();
        float filter(float xn);
        void update(float en);

};




class CVAFxNLMS {
    
    private:
        int ptr = 0;

    public:
        float *x;
        float *x2;
        float *xsec;
        float *xsec2;
        float *w;
        float *w2;
        float y;
        float mu;
        float fi;
        int N = 5;
        int Nsec = 5;
        FIRFilter *filtsec;
        FIRFilter *filtsec2;
        float normterm;
        float aux,aux2;
        CVAFxNLMS(int mem, float *wa, float *xa, float *wa2, float *xa2, float *xasec, float *xasec2, FIRFilter *fsec, FIRFilter *fsec2);
        void setParameters(int mem, float muu, float fii);
        void reset();
        float filter(float xn,float xn2);
        void update(float en);    

};



#endif