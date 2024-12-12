/*
 * SimpleKalmanFilter.h
 *
 *  Created on: Sep 20, 2024
 *      Author: Admin
 */

#ifndef INC_SIMPLEKALMANFILTER_H_
#define INC_SIMPLEKALMANFILTER_H_

typedef struct {
    float err_measure;
    float err_estimate;
    float q;
    float kalman_gain;
    float last_estimate;
    float current_estimate;
} SimpleKalmanFilter;


// Function declarations
void SimpleKalmanFilter_Init(SimpleKalmanFilter *filter, float mea_e, float est_e, float q);
float SimpleKalmanFilter_UpdateEstimate(SimpleKalmanFilter *filter, float mea);
void SimpleKalmanFilter_SetMeasurementError(SimpleKalmanFilter *filter, float mea_e);
void SimpleKalmanFilter_SetEstimateError(SimpleKalmanFilter *filter, float est_e);
void SimpleKalmanFilter_SetProcessNoise(SimpleKalmanFilter *filter, float q);
float SimpleKalmanFilter_GetKalmanGain(SimpleKalmanFilter *filter);
float SimpleKalmanFilter_GetEstimateError(SimpleKalmanFilter *filter);



#endif /* INC_SIMPLEKALMANFILTER_H_ */
