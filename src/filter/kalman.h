#pragma once
#include "includes.h"
#include "gyro.h"
#include "filter.h"

#define MAX_WINDOW_SIZE 512
#define DEF_WINDOW_SIZE 32
#define MIN_WINDOW_SIZE 3

// #define VARIANCE_SCALE 0.001
#define VARIANCE_SCALE 0.67f

typedef struct kalman
{
    float q;     //process noise covariance
    float r;     //measurement noise covariance
    float p;     //estimation error covariance matrix
    float k;     //kalman gain
    float x;     //state
    float lastX; //previous state
    float e;
    float s;
    float axisVar;
    uint16_t windex;
    float axisWindow[MAX_WINDOW_SIZE];
    float varianceWindow[MAX_WINDOW_SIZE];
    float axisSumMean;
    float axisMean;
    float axisSumVar;
    float inverseN;
    uint16_t w;
} kalman_t;

extern void kalman_init(void);
extern float kalman_update(float input, float setpoint, int axis);
