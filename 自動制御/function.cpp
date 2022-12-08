#include "function.h"
// #include <mbed.h>
#include <Arduino.h>
#include <math.h>
// #include "init.h"
#include "define.h"

#define PI 3.14159265358979
// double cir(double sita){
//     double a,b;
//     if(sita>=0 && sita<90)a = 1;
//     if(sita>=90 && sita<180)b = 1;
//     if(sita>=180 && sita<270)a = -1;
//     if(sita>=270 && sita<360)b = -1;
//     //flag

//     if(b==1 || b==-1)a = a*(1+tan(sita))/(1-tan(sita));
//     if(a==1 || a==-1)b = b*(1-tan(sita))/(1+tan(sita));
// }

double Sita(double x, double y){
	double rad;
	
    if(x == 100)x = sqrt(20000 - y*y);
    if(x == -100)x = -sqrt(20000 - y*y);
    if(y == 100)y = sqrt(20000 - x*x);
    if(y == -100)y = -sqrt(20000 - x*x);
    //20000 = 100*100*RUTO2*RUTO2  
	// pc.printf("%lf %lf\n",x ,y);
	if(x == 0 && y == 0){
		rad = 0;
	}else{
		rad = atan(y/x);
		if(x<0)rad += (y>0) ? PI : -PI;
	}
	return rad;
}

double Controll(double x ,double y){
	double diff;
	
	if((x*x+y*y)<=10000){
        diff = sqrt(x*x + y*y);
    }else{ 
        diff = 100;
    }

	return diff;
}

int i_abs(int num){
	return (num>=0) ? num:-num;
}

int is(int num){
	return !!num;
}

int map(double num, double from_min, double from_max, double to_min, double to_max){
	return (num - from_min) * (to_max - to_min) / (from_max - from_min) + to_min;
}

int Button_switch(int num){
	static int flag1 = 0;
    static int flag2 = 0;

    if(num == 1 && flag2 == 0){
        flag1 = 1;
        flag2 = 1;
    }else if(num == 0 && flag2 == 1){
        flag2 = 2;
    }else if(num == 1 && flag2 == 2){
        flag1 = 0;
        flag2 = 3;
    }else if(num == 0 && flag2 == 3){
        flag2 = 0;
    }
	return flag1;
}

void Search(double num){
    num = (num > 0) ? 1 : (num < 0) ? -1 : 0;
        // if(num > 0)num = 1;
        // if(num < 0)num = -1;
        // if(num == 0)num = 0;
}

void Pm(double* s, int* p, int* q){

    if(s[0]>0){
        p[0] = 1;
        q[0] = 1;
    }else if(s[0]<0){
        p[0] = -1;
        q[0] = -1;
    }else if(s[0]==0){
        p[0] = 0;
        q[0] = 0;
    }

    if(s[1]>0){
        p[1] = -1;
        q[1] = 1;    
    }else if(s[1]<0){
        p[1] = 1;
        q[1] = -1;
    }else if(s[1]==0){
        p[1] = 0;
        q[1] = 0;
    }

}

void MaxTime(double sita1,double* a){

    if((sita1>=0 && sita1<=PI/2) || (sita1>=-PI || sita1<=-PI*1/2))a[0]=1;
    else a[0]=0;

    if((sita1>=PI/2 && sita1<=PI) || (sita1>=-PI*1/2 || sita1<=0))a[1]=1;
    else a[1]=0;
}

void Rate(double* a, int* p, int* q, double sita){
    if(a[0] == 1 && a[1] == 0)a[1] = (q[0] - p[0]*tan(sita)) / (p[1]*tan(sita) - q[1]);
    if(a[0] == 0 && a[1] == 1)a[0] = (p[1]*tan(sita) - q[1]) / (q[0] - p[0]*tan(sita));
}

void Cir(double* s, double sita1, double sita, double* z){
    int p[2],q[2];
    double a[2];

    Pm(s, p, q);
    MaxTime(sita1,a);
    Rate(a, p, q, sita);
    z[0] = a[0];
    z[1] = a[1];
}