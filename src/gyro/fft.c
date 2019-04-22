#include <stdbool.h>
#include "includes.h"
#include "gyro.h"
#include "biquad.h"
#include "filter.h"
#include "imu.h"
#include "fft.h"

#include "arm_math.h"
#include "arm_common_tables.h"

#define FFT_DATA_SET_SIZE (64)	// fft size will be 32 - always power a of 2
#define FFT_SIZE          (FFT_DATA_SET_SIZE >> 1)

#define BQQ 0.707f //butterworth response 1/sqrt(2)

// used for downsampling from 32 khz to 1 khz
// thus we feed 1 khz data into the fft
int sampleCount = 0;
float sampleGyroDataX = 0.0f;
float sampleGyroDataY = 0.0f;
float sampleGyroDataZ = 0.0f;

typedef enum fftUpdateState
{
    FFT_STATE_CALCULATE_X = 0,
    FFT_STATE_CALCULATE_X_DONE = 1,
    FFT_STATE_CALCULATE_Y = 2,
    FFT_STATE_CALCULATE_Y_DONE = 3,
    FFT_STATE_CALCULATE_Z = 4,
    FFT_STATE_CALCULATE_Z_DONE = 5,
} fftUpdateState_t;

volatile fftUpdateState_t fftUpdateState;

static biquad_axis_state_t centerFrqFiltX;
static biquad_axis_state_t centerFrqFiltY;
static biquad_axis_state_t centerFrqFiltZ;

//gyro data pointer
unsigned int fftGyroDataPtr;

//data for use in FFT stored here
float fftGyroDataX[FFT_DATA_SET_SIZE];
float fftGyroDataY[FFT_DATA_SET_SIZE];
float fftGyroDataZ[FFT_DATA_SET_SIZE];

// the output bin's - it's half the size
float rfftGyroDataX[FFT_SIZE];
float rfftGyroDataY[FFT_SIZE];
float rfftGyroDataZ[FFT_SIZE];

//fft arm dsp instance
static arm_rfft_fast_instance_f32 fftInstance;

//number of "bins" which contain the fft magnitude
static const uint32_t fftBinCount = ((FFT_DATA_SET_SIZE / 2 - 1) * FFT_MAX_HZ) / (FFT_MAX_HZ) + 1;


static void calculate_fft(float *fftData, float *rfftData, float* centerFrequency);



extern void arm_bitreversal_32(uint32_t * pSrc, const uint16_t bitRevLen, const uint16_t * pBitRevTable);
extern void stage_rfft_f32(arm_rfft_fast_instance_f32 * S, float32_t * p, float32_t * pOut);
extern void arm_cfft_radix8by2_f32( arm_cfft_instance_f32 * S, float32_t * p1);
extern void arm_cfft_radix8by4_f32( arm_cfft_instance_f32 * S, float32_t * p1);
extern void arm_radix8_butterfly_f32(float32_t * pSrc, uint16_t fftLen, const float32_t * pCoef, uint16_t twidCoefModifier);
extern void arm_cmplx_mag_f32(float32_t * pSrc, float32_t * pDst, uint32_t numSamples);

//triggered by gyro steps. This will run oncer every 32 cycles.
//This function sets the fftUpdate state machine
void increment_fft_state(void)
{
    switch(fftUpdateState)
    {
        case FFT_STATE_CALCULATE_X:
            //do nothing, let the main loop handle calculations, should never get here unless CPU load to too high
            break;
        case FFT_STATE_CALCULATE_X_DONE:
            //main loop done calculating, go ahead and tell the main loop it's okay to calc again by incrementing the state
            fftUpdateState = FFT_STATE_CALCULATE_Y;
            break;
        case FFT_STATE_CALCULATE_Y:
            //do nothing, let the main loop handle calculations, should never get here unless CPU load to too high
            break;
        case FFT_STATE_CALCULATE_Y_DONE:
            //main loop done calculating, go ahead and tell the main loop it's okay to calc again by incrementing the state
            fftUpdateState = FFT_STATE_CALCULATE_Z;
            break;
        case FFT_STATE_CALCULATE_Z:
            //do nothing, let the main loop handle calculations, should never get here unless CPU load to too high
            break;
        case FFT_STATE_CALCULATE_Z_DONE:
            //main loop done calculating, go ahead and tell the main loop it's okay to calc again by incrementing the state
            fftUpdateState = FFT_STATE_CALCULATE_X;
            break;
    }
}

//run by the main loop, check the state machine to  see when it's time to do it's job
void update_fft(void)
{
	float centerFrequency;
    switch(fftUpdateState)
    {
        case FFT_STATE_CALCULATE_X:
            //run calculations, bla bla ...
            calculate_fft(fftGyroDataX, rfftGyroDataX, &centerFrequency );
            //init new calculations
            biquad_init(centerFrequency, &axisX, REFRESH_RATE, FILTER_TYPE_NOTCH, BQQ);
            //set new state
            fftUpdateState = FFT_STATE_CALCULATE_X_DONE;
            break;
        case FFT_STATE_CALCULATE_X_DONE:
            //do nothing, wait for real-time functions to set state
            break;
        case FFT_STATE_CALCULATE_Y:
            //run calculations, calculate filter, runs at 333 Hz using 1000 Hz samples
            calculate_fft(fftGyroDataY, rfftGyroDataY, &centerFrequency );
            //init new calculations
            biquad_init(centerFrequency, &axisY, REFRESH_RATE, FILTER_TYPE_NOTCH, BQQ);
            //set new state
            fftUpdateState = FFT_STATE_CALCULATE_Y_DONE;
            break;
        case FFT_STATE_CALCULATE_Y_DONE:
            //do nothing, wait for real-time functions to set state
            break;
        case FFT_STATE_CALCULATE_Z:
            //run calculations, calculate filter, runs at 333 Hz using 1000 Hz samples
            calculate_fft(fftGyroDataZ, rfftGyroDataZ, &centerFrequency );
            //init new calculations
            biquad_init(centerFrequency, &axisZ, REFRESH_RATE, FILTER_TYPE_NOTCH, BQQ);
            //set new state
            fftUpdateState = FFT_STATE_CALCULATE_Z_DONE;
            break;
        case FFT_STATE_CALCULATE_Z_DONE:
            //do nothing, wait for real-time functions to set state
            break;
    }

}

void insert_gyro_data_for_fft(filteredData_t* filteredData)
{
#define MULTIPLIER (1.0f / 32.0f)
	// accumulate
	sampleGyroDataX += filteredData->rateData.x;
	sampleGyroDataY += filteredData->rateData.y;
	sampleGyroDataZ += filteredData->rateData.z;
	sampleCount++;
	if (32 == sampleCount)
	{
		// feed a new fft sample
		fftGyroDataX[fftGyroDataPtr]   = sampleGyroDataX * MULTIPLIER;
		fftGyroDataY[fftGyroDataPtr]   = sampleGyroDataY * MULTIPLIER;
		fftGyroDataZ[fftGyroDataPtr++] = sampleGyroDataZ * MULTIPLIER;

		// the complex part of the fft data - is zero
		fftGyroDataX[fftGyroDataPtr]   = 0.0f;
		fftGyroDataY[fftGyroDataPtr]   = 0.0f;
		fftGyroDataZ[fftGyroDataPtr++] = 0.0f;

		// reset the accumulator
		sampleGyroDataX = 0.0f;
		sampleGyroDataY = 0.0f;
		sampleGyroDataZ = 0.0f;
		sampleCount = 0;

		//prevent overflow and circularize the buffer
		if(fftGyroDataPtr==FFT_DATA_SET_SIZE)
		{
			fftGyroDataPtr = 0;
			// a new fft calculation run can be issued
			increment_fft_state();
		}
	}
}

//init fft anytime the filters are init. this happens in filter.c
void init_fft(void)
{
    //set pointer
    fftGyroDataPtr = 0;

    //60 hz lowpass on center frequency changes, samples update at 333 Hz, BQQ makes this butterworth
    memset(&centerFrqFiltX, 0, sizeof(centerFrqFiltX));
    memset(&centerFrqFiltY, 0, sizeof(centerFrqFiltY));
    memset(&centerFrqFiltZ, 0, sizeof(centerFrqFiltZ));
    biquad_init(60.0f, &centerFrqFiltX, 0.003003003f, FILTER_TYPE_LOWPASS, BQQ);
    biquad_init(60.0f, &centerFrqFiltY, 0.003003003f, FILTER_TYPE_LOWPASS, BQQ);
    biquad_init(60.0f, &centerFrqFiltZ, 0.003003003f, FILTER_TYPE_LOWPASS, BQQ);

    //set default  state for fft:
    fftUpdateState = FFT_STATE_CALCULATE_Z_DONE; // this will start caclulations going on axis X basically

    arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);
}

static void calculate_fft(float *fftData, float *rfftData, float* fftCenter )
{
    float    maxValue;             /* Max FFT value is stored here */
    uint32_t maxIndex;             /* Index in Output array where max value is */

    //pointer to Sint for using in bitreversal
    arm_cfft_instance_f32 * Sint = &(fftInstance.Sint);

    //ginormous, expensive function, need to figure out how long it takes to run this monster
    arm_radix8_butterfly_f32(fftData, FFT_SIZE, Sint->pTwiddle, 1);

    //asm code
    arm_bitreversal_32((uint32_t*)fftData, Sint->bitRevLength, Sint->pBitRevTable);

    //expensive function
    stage_rfft_f32(&fftInstance, fftData, rfftData);

    //lots of square roots
    arm_cmplx_mag_f32(rfftData, fftData, fftBinCount);

    /* Calculates maxValue and returns corresponding value */
    arm_max_f32(rfftData, FFT_SIZE, &maxValue, &maxIndex);

    /* calculate fft frequency for the found bin */
    const float centerFrequency = (float)FFT_MAX_HZ / (float)FFT_SIZE * (float)maxIndex;    //set result to 0

    *fftCenter = CONSTRAIN( centerFrequency, NOTCH_MIN, NOTCH_MAX);
}
