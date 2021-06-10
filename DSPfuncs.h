#ifndef DSPfuncs_h
#define DSPfuncs_h

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
            float F_SAMPLE;
            float T_SAMPLE;            
            int STD_Z_LEVEL;
            float levelscaler;
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

        public:
            int last;
            bool enabled;
            int Z_LEVEL;
            SignalGenerator(float fsampling, float tsampling, int stdzlevel);
            void setType(int tp, float Amp, float freq, int dclevel);
            void setChirpParams(int ti, int di, int tf, int df, int a2);
            int next();
            
    };

#endif