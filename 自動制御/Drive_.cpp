// #include <mbed.h>
#include <Arduino.h>
#include <math.h>
#include "Drive_.h"
// #include "init.h"
#include "function.h"
#include "define.h"
// #define PI 3.14159265358979
// #define pi 31415926
/*///////

      [0]
   [3]   [1]
      [2]

      //////*/

void anomalyDriveing(double x, double y, double r, double* velocity, double rad){

    #define TRANSLATIONAL_VELOCITY_RATIO    40.0
    #define ROTETIONAL_VELOCITY_RATIO       20.0

    #define SPEED_LIMIT 4000.0
    double diff;
    double sita,sita1;
    double s[2];
    double z[2];
    
    diff = Controll(x ,y);
    sita = Sita(x, y);
    sita1 = (PI/4) - Sita(x, y) - rad;
    // sita1 = (PI/4) - Sita(x, y);
    if(sita1>PI)sita1 = sita1 - 2*PI;
    // sita1 = (pi/4) - Sita(x, y)*pow(10,7);
    // sita1 /= pow(10,7);

    const double gainxy[WHEEL_COUNT] = { cos(sita1), sin(sita1), -cos(sita1), -sin(sita1)};
    const double gainr[WHEEL_COUNT] = { 1, 1, 1, 1};

    double spd[WHEEL_COUNT]={0};
    double max_spd = 0;
    int i;

    s[0] = -sin(sita1);
    s[1] = cos(sita1);
    s[0] = (s[0] > 0) ? 1 : (s[0] < 0) ? -1 : 0;
    s[1] = (s[1] > 0) ? 1 : (s[1] < 0) ? -1 : 0;
    // for(int j=0;j<2;j++){
    //     Search(s[j]);
    // }
    int p[2],q[2];
    double a[2];

    Pm(s, p, q);
    // MaxTime(sita1,a);
    if((sita1>=0 && sita1<=PI/2) || (sita1>=-PI && sita1<=-PI*1/2))a[0]=1;
    else a[0]=0;

    if((sita1>=PI/2 && sita1<=PI) || (sita1>=-PI*1/2 && sita1<=0))a[1]=1;
    else a[1]=0;

    if(a[0] == 1 && a[1] == 0)a[1] = (q[0] - p[0]*tan(sita)) / (p[1]*tan(sita) - q[1]);
    if(a[0] == 0 && a[1] == 1)a[0] = (p[1]*tan(sita) - q[1]) / (q[0] - p[0]*tan(sita));

    z[0] = a[0];
    z[1] = a[1];
    // Cir(s, sita1, sita, z);

    // const double gainxy[WHEEL_COUNT] = { s[1]*z[0], -s[0]*z[1], -s[1]*z[1], s[0]*z[0]};

    for(i=0; i<WHEEL_COUNT; i++){
        double fast;
        spd[i] = diff * gainxy[i] * TRANSLATIONAL_VELOCITY_RATIO + r * gainr[i] * ROTETIONAL_VELOCITY_RATIO;
        fast = (spd[i]>0) ? spd[i] : -spd[i];
    if(fast>max_spd) max_spd = fast;
    }
    if(SPEED_LIMIT < max_spd){
        for(i=0;i<WHEEL_COUNT;i++){
            spd[i] *= SPEED_LIMIT / max_spd;
        }
    }
    for(i=0; i<WHEEL_COUNT; i++){
        velocity[i] = spd[i];
        // velocity[0] = z[0];
        // velocity[1] = z[1];
        velocity[2] = sita1;
        // velocity[3] = sita;
    }
}

// void anomalyDriveing(double x, double y, double r, double* velocity){
//     #define WHEEL_COUNT 4

//     #define TRANSLATIONAL_VELOCITY_RATIO    40.0
//     #define ROTETIONAL_VELOCITY_RATIO       20.0

//     #define SPEED_LIMIT 4000.0

//     const double gainx[WHEEL_COUNT] = { 1, 0, -1, 0};
//     const double gainy[WHEEL_COUNT] = { 0, -1, 0, -1};
//     const double gainr[WHEEL_COUNT] = { 1, 1, 1, 1};

//     double spd[WHEEL_COUNT]={0};
//     double max_spd = 0;
//     int i;
//     for(i=0; i<WHEEL_COUNT; i++){
//         double fast;
//         spd[i] = (x*gainx[i]+y*gainy[i]) * TRANSLATIONAL_VELOCITY_RATIO + r*gainr[i] * ROTETIONAL_VELOCITY_RATIO;
//         fast = (spd[i]>0) ? spd[i] : -spd[i];
//     if(fast>max_spd) max_spd = fast;
//     }
//     if(SPEED_LIMIT < max_spd){
//         for(i=0;i<WHEEL_COUNT;i++){
//             spd[i] *= SPEED_LIMIT / max_spd;
//         }
//     }
//     for(i=0; i<WHEEL_COUNT; i++){
//         velocity[i] = spd[i];
//     }
// }




// void KUDOU(double* velocity){
//     int i = 0;
//     int identification[4];

//     for(i=0; i<4; i++){
//         if(velocity[i]>0){
//             identification[i] = 1;
//         }else if(velocity[i]<0){
//             identification[i] = -1;
//             velocity[i] = -velocity[i];
//         }else{
//             identification[i] = 0;
//         }
//     }
    
//     // for(i = 0; i< 4; i++){
//     //     if(identification[i] == 1){
//     //         moter[i * 2] = 1;
//     //         moter[i * 2 + 1] = 0;
//     //     }else  if(identification[i] == -1){
//     //         moter[i * 2] = 0;
//     //         moter[i * 2 + 1] = 1;
//     //     }else{
//     //         moter[i * 2] = 0;
//     //         moter[i * 2 + 1] = 0;
//     //     }
//     // }
    
//     // for(i = 0;i < 4;i++){
//     //     pwm[i] = map(velocity[i], 0, 5000, 0, 1.0);
//     // }
// }
