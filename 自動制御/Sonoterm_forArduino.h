#ifndef _SONOTERM_H_
#define _SONOTERM_H_

#include <Arduino.h>

class Sonoterm {
public:

    Sonoterm(int);
    static void funcPass();
    void init();
    void txvalue(unsigned char );
    void dnl();
    void putstr(char *);
    void disp(long int );

    void comma();
    int advancedDisp(double , int );

    void DebugToW(double * ,double *);
    void DebugThW(double * ,double * ,double *);
    void DebugFW(double * ,double * ,double * ,double *);
    void DebugPID(double * ,double * ,double *);
    void DebugEnd();

    class Button {
    public:
    	int press;
	    int count;
	    int flg;
	    int toggle;
    };
    
    Button Up;
    Button Down;
    Button Right;
    Button Left;
    Button shift;
    Button enter;
    Button control;
    Button tab;
    Button del;
    Button alt;
    Button backspace;
    Button esc;
    Button key_q;
    Button key_a;
    Button key_z;
    Button key_w;
    Button key_s;
    Button key_x;
    Button key_e;
    Button key_d;
    Button key_c;
    Button key_r;
    Button key_f;
    Button key_v;
    Button key_ta;
    Button key_g;
    Button key_b;
    Button key_y;
    Button key_h;
    Button key_n;
    Button key_u;
    Button key_j;
    Button key_m;
    Button key_i;
    Button key_k;
    Button key_o;
    Button key_l;
    Button key_p;
    Button one;
    Button two;
    Button three;
    Button four;
    Button five;
    Button six;
    Button seven;
    Button eight;
    Button nine;
    Button zero;

    void receiveInt();
    void txd(char);
private:

    static Sonoterm* pass;
    #define INTLIMIT 255
    #define DECLIMIT 100000
    #define BIT 8

    int bps;
    
    int bitCount(int);

    void FractionTx(double);
    uint64_t byteSwap(uint64_t);
    int RangeFlood(double);
    void getKey(char *);
    int pushCount(Button *);
    void getGain(char *);
    int is(int);

    
    char rx_buff = 0;
    bool start_sigh_flg = false;
    bool debug_request_flg = false;
    bool debug_flg = false;
    int debug_call_count[4] = {};
    double *gain_addres[4][4][4];
    
};

#endif