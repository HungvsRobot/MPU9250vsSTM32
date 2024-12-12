/*
 * SimpleKalmanFilter.c
 *
 *  Created on: Sep 20, 2024
 *      Author: Admin
 */

#include "Kalmanfilter.h"
#include <math.h>


SimpleKalmanFilter Filter_Ax = {
		.err_measure = 0.02,
		.err_estimate = 0.01,
		.q = 0.002
};

SimpleKalmanFilter Filter_Ay = {
		.err_measure = 0.01,
		.err_estimate = 0.01,
		.q = 0.002
};

SimpleKalmanFilter Filter_Az = {
		.err_measure = 0.02,
		.err_estimate = 0.01,
		.q = 0.002
};

SimpleKalmanFilter Filter_Gz = {
		.err_measure = 0.26,
		.err_estimate = 0.3,
		.q = 0.05*0.26
};

SimpleKalmanFilter Filter_Gy = {
		.err_measure = 0.001,
		.err_estimate = 0.001,
		.q = 0.05
};

SimpleKalmanFilter Filter_Gx = {
		.err_measure = 0.001,
		.err_estimate = 0.001,
		.q = 0.05
};
SimpleKalmanFilter Filter_Anglex = {
		.err_measure = 0.0005,
		.err_estimate = 0.0005,
		.q = 0.05
};

SimpleKalmanFilter Filter_Angley = {
		.err_measure = 0.0005,
		.err_estimate = 0.0005,
		.q = 0.05
};

SimpleKalmanFilter Filter_Anglez = {
		.err_measure = 0.0005,
		.err_estimate = 0.0005,
		.q = 0.05
};

// Initialization function
void SimpleKalmanFilter_Init(SimpleKalmanFilter *filter, float mea_e, float est_e, float q) {
    filter->err_measure = mea_e;
    filter->err_estimate = est_e;
    filter->q = q;
    filter->kalman_gain = 0;
    filter->last_estimate = 0;
    filter->current_estimate = 0;
}

// Function to update the estimate based on the new measurement
float SimpleKalmanFilter_UpdateEstimate(SimpleKalmanFilter *filter, float mea) {
    filter->kalman_gain = filter->err_estimate / (filter->err_estimate + filter->err_measure);
    filter->current_estimate = filter->last_estimate + filter->kalman_gain * (mea - filter->last_estimate);
    filter->err_estimate = (1.0f - filter->kalman_gain) * filter->err_estimate +
                           fabsf(filter->last_estimate - filter->current_estimate) * filter->q;
    filter->last_estimate = filter->current_estimate;

    return filter->current_estimate;
}

// Setters for the filter parameters
void SimpleKalmanFilter_SetMeasurementError(SimpleKalmanFilter *filter, float mea_e) {
    filter->err_measure = mea_e;
}

void SimpleKalmanFilter_SetEstimateError(SimpleKalmanFilter *filter, float est_e) {
    filter->err_estimate = est_e;
}

void SimpleKalmanFilter_SetProcessNoise(SimpleKalmanFilter *filter, float q) {
    filter->q = q;
}

// Getters for the Kalman Gain and Estimate Error
float SimpleKalmanFilter_GetKalmanGain(SimpleKalmanFilter *filter) {
    return filter->kalman_gain;
}

float SimpleKalmanFilter_GetEstimateError(SimpleKalmanFilter *filter) {
    return filter->err_estimate;
}



