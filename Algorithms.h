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
        FIRFilter();
        FIRFilter(int mem, float *ptrw, float *ptrx);
        void setMem(int mem);
        void setMemAndPointers(int mem, float *ptrw, float *ptrx);
        void reset();
        float filter(float xn);

};

class FIRFilterSVD {

    private:
        int ptr = 0;
        FIRFilter firbranches[10] = {FIRFilter()};

    public:
        float *wptr;
        float *x;
        float y;
        int N = 5;
        int R = 10;
        int C = 10;
        int B = 1;
        float buffers[500] = {0};
        FIRFilterSVD();
        FIRFilterSVD(int mem, int nbranches, int nR, int nC, float *ptrw, float *ptrx);
        void setAllParams(int mem, int nbranches, int nR, int nC, float *ptrw, float *ptrx);
        void setParams(int mem, int nbranches, int nR, int nC);
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
        float *deltaw;
        float y;
        float mu;
        float fi;
        int N = 5;
        int Nsec = 5;
        float (*filtsec)(float);
        float normterm;
        float aux;
        FxNLMS(int mem, float *wa, float *xa, float *xasec, float (*fsecfilter)(float), float *deltawa);
        void setParameters(int mem, float muu, float fii);
        void reset();
        float filter(float xn);
        void update(float en);
        void updateStep1();  
        void updateStep2(float en); 

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
        float *deltaw;
        float *deltaw2;
        float y;
        float mu;
        float fi;
        int N = 5;
        int Nsec = 5;
        FIRFilter *filtsec;
        FIRFilter *filtsec2;
        float normterm;
        float aux,aux2;
        CVAFxNLMS(int mem, float *wa, float *xa, float *wa2, float *xa2, float *xasec, float *xasec2, FIRFilter *fsec, FIRFilter *fsec2, float *deltawa, float *deltawa2);
        void setParameters(int mem, float muu, float fii);
        void reset();
        float filter(float xn,float xn2);
        void update(float en);    
        void updateStep1();  
        void updateStep2(float en); 

};



#endif