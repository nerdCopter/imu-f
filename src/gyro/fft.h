#pragma once

#include "filter.h"
#include "arm_math.h"

#define FFT_MIN_HZ            80
#define FFT_MAX_HZ            500 
#define NOTCH_WIDTH           40 
#define NOTCH_MIN             80
#define NOTCH_MAX             386
#define AXIS_AMOUNT           3
#define FFT_BUFFS             2

extern void init_fft(void);
extern void update_fft(void);
extern void increment_fft_state(void);
extern void insert_gyro_data_for_fft(filteredData_t* filteredData);
