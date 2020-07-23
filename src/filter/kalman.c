#include "includes.h"
#include "gyro.h"
#include "kalman.h"
#include "filter.h"

kalman_t   kalmanFilterStateRate[3];

void init_kalman(kalman_t *filter, float q)
{
    memset(filter, 0, sizeof(kalman_t));
    filter->q = q * 0.001f;      //add multiplier to make tuning easier
    filter->r = 88.0f;           //seeding R at 88.0f
    filter->p = 30.0f;           //seeding P at 30.0f
    filter->e = 1.0f;
    filter->inverseN = 1.0f / (float)filterConfig.w;
}

void kalman_init(void)
{
    setPointNew = 0;
    init_kalman(&kalmanFilterStateRate[ROLL], filterConfig.roll_q);
    init_kalman(&kalmanFilterStateRate[PITCH], filterConfig.pitch_q);
    init_kalman(&kalmanFilterStateRate[YAW], filterConfig.yaw_q);
}

#pragma GCC push_options
#pragma GCC optimize("O3")
void update_kalman_covariance(kalman_t *kalmanState, float rate)
{
    kalmanState->axisWindow[kalmanState->windex] = rate;

    kalmanState->axisSumMean += kalmanState->axisWindow[kalmanState->windex];
    float varianceElement = kalmanState->axisWindow[kalmanState->windex] - kalmanState->axisMean;
    varianceElement = varianceElement * varianceElement;
    kalmanState->axisSumVar += varianceElement;
    kalmanState->varianceWindow[kalmanState->windex] = varianceElement;

    kalmanState->windex++;
    if (kalmanState->windex >= kalmanState->w) {
        kalmanState->windex = 0;
    }

    kalmanState->axisSumMean -= kalmanState->axisWindow[kalmanState->windex];
    kalmanState->axisSumVar -= kalmanState->varianceWindow[kalmanState->windex];

    //New mean
    kalmanState->axisMean = kalmanState->axisSumMean * kalmanState->inverseN;
    kalmanState->axisVar = kalmanState->axisSumVar * kalmanState->inverseN;

    float squirt;
    arm_sqrt_f32(kalmanState->axisVar, &squirt);
    kalmanState->r = squirt * VARIANCE_SCALE;
}

float kalman_process(kalman_t* kalmanState, volatile float input, volatile float target) {
    //project the state ahead using acceleration
    kalmanState->x += (kalmanState->x - kalmanState->lastX);

    //figure out how much to boost or reduce our error in the estimate based on setpoint target.
    //this should be close to 0 as we approach the sepoint and really high the futher away we are from the setpoint.
    //update last state
    kalmanState->lastX = kalmanState->x;

    if (target != 0.0f) {
        kalmanState->e = ABS(1.0f - (target / kalmanState->lastX));
    } else {
        kalmanState->e = 1.0f;
    }

    //prediction update
    kalmanState->p = kalmanState->p + (kalmanState->q * kalmanState->e);

    //measurement update
    kalmanState->k = kalmanState->p / (kalmanState->p + kalmanState->r);
    kalmanState->x += kalmanState->k * (input - kalmanState->x);
    kalmanState->p = (1.0f - kalmanState->k) * kalmanState->p;
    return kalmanState->x;
}


float kalman_update(float input, float setpoint, int axis)
{
    update_kalman_covariance(&kalmanFilterStateRate[axis], input);
    input = kalman_process(&kalmanFilterStateRate[axis], input, setpoint);

    return input;
}
#pragma GCC pop_options
