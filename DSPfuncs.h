#ifndef DSPfuncs_h
#define DSPfuncs_h

    float evalPoly(float x, int order, float* coefs);

    class OutputScaler {
        private:
            int STD_Z_LEVEL;             
            float levelscaler;
        public:
            int lastwrittenout;
            int dclevel;
            OutputScaler(int stdzlevel);
            void adjust(int newdclevel);
            int evalOut(float valf);
            int evalOutNoReg(float valf);
    };

    class DCRemover {
        public:
            float y;
            float wn,wn1;
            float alpha;
            DCRemover(float alp);
            void reset();
            float filter(float x);
    };

    class SignalGenerator {

        private:
            float BASE_F_SAMPLE;
            float F_SAMPLE;
            float BASE_T_SAMPLE;
            float T_SAMPLE;            
            // int STD_Z_LEVEL;            
            int type;
            float A;
            float f;
            float sincnst;
            float n;
            float ctinit;
            float cdeltai;
            float ctfim;
            float cdeltaf;
            float ff;
            float AA;
            float integ;
            float cA;
            float A2;
            float AA2;
            float integ2;
            float cA2;
            int Psize2;
            int lastaux;

        public:
            // int last;
            float lastf;
            // float levelscaler;
            // int lastwrittenout;
            bool enabled;
            // int Z_LEVEL;
            SignalGenerator(float fsampling, float tsampling);
            void setType(int tp, float Amp, float freq, float freqmult);
            void setType(int tp, float Amp, float freq, float freqmult, float basefreq);
            void setBaseFreq(float freq);
            void setFreqMult(float freqmult);
            void setFreqMult(float freqmult, float freq);
            void setChirpParams(int ti, int di, int tf, int df, int a2);
            float next();
            
    };

#endif