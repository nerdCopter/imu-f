#include "includes.h"
#include "ptnFilter.h"

// PTn Low Pass filter
void ptnFilterInit(float f_cut, ptnFilter_axis_t *filter, uint8_t order) {
	// Cutoff frequency fix: removed ScaleF correction to match Betaflight behavior
	// The configured cutoff now represents the per-stage cutoff, not the combined -3dB point
	// This provides more aggressive filtering and matches user expectations
	// For cascaded filters: combined -3dB point = f_cut * sqrt(2^(1/order) - 1)
	// Example: PT2 with 90Hz configured -> per-stage 90Hz, combined -3dB at ~58Hz
	int n;
	filter->order = order;
	for (n = 1; n <= filter->order; n++)
		filter->state[n] = 0.0f;
	filter->k = REFRESH_RATE / ((1.0f / (2.0f * M_PI_FLOAT * f_cut)) + REFRESH_RATE);
} // ptnFilterInit

void ptnFilterUpdate(float f_cut, ptnFilter_axis_t *filter) {
  // Cutoff frequency fix: removed ScaleF parameter
  // Now uses configured cutoff directly for filter coefficient update
  filter->k = REFRESH_RATE / ((1.0f / (2.0f * M_PI_FLOAT * f_cut)) + REFRESH_RATE);
}

float ptnFilterApply(float input, ptnFilter_axis_t *filter) {
int n;
	filter->state[0] = input;
	for (n = 1; n <= filter->order; n++)
		filter->state[n] += (filter->state[n - 1] - filter->state[n])
				* filter->k;
	return filter->state[filter->order];
} // ptnFilterApply
