/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics-> All rights reserved->

  This software may be distributed and modified under the terms of the GNU
  General Public License version 2 (GPL2) as published by the Free Software
  Foundation and appearing in the file GPL2->TXT included in the packaging of
  this file-> Please note that GPL2 Section 2[b] requires that all works based
  on this software must also be made publicly available under the terms of
  the GPL2 ("Copyleft")->

  Contact information
  -------------------

  Kristian Lauszus, TKJ Electronics
  Web      :  http://www->tkjelectronics->com
  e-mail   :  kristianl@tkjelectronics->com
*/
#include "kalman.h"

void Kalman_Init(struct Kalman* klm)
{
    /* We will set the variables like so, these can also be tuned by the user */
    klm->Q_angle = 0.001;
    klm->Q_bias = 0.003;
    klm->R_measure = 0.03;

    klm->angle = 0; // Reset the angle
    klm->bias = 0; // Reset bias

    klm->P[0][0] = 0; // Since we assume that the bias is 0 and we know the starting angle (use setAngle), the error covariance matrix is set like so - see: http://en->wikipedia->org/wiki/Kalman_filter#Example_application->2C_technical
    klm->P[0][1] = 0;
    klm->P[1][0] = 0;
    klm->P[1][1] = 0;
}

// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
double Kalman_getAngle(struct Kalman * klm, double newAngle, double newRate, double dt)
{
    // KasBot V2  -  Kalman filter module - http://www->x-firm->com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog->tkjelectronics->dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    klm->rate = newRate - klm->bias;
    klm->angle += dt * klm->rate;

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    klm->P[0][0] += dt * (dt*klm->P[1][1] - klm->P[0][1] - klm->P[1][0] + klm->Q_angle);
    klm->P[0][1] -= dt * klm->P[1][1];
    klm->P[1][0] -= dt * klm->P[1][1];
    klm->P[1][1] += klm->Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    klm->S = klm->P[0][0] + klm->R_measure;
    /* Step 5 */
    klm->K[0] = klm->P[0][0] / klm->S;
    klm->K[1] = klm->P[1][0] / klm->S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    klm->y = newAngle - klm->angle;
    /* Step 6 */
    klm->angle += klm->K[0] * klm->y;
    klm->bias += klm->K[1] * klm->y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    klm->P[0][0] -= klm->K[0] * klm->P[0][0];
    klm->P[0][1] -= klm->K[0] * klm->P[0][1];
    klm->P[1][0] -= klm->K[1] * klm->P[0][0];
    klm->P[1][1] -= klm->K[1] * klm->P[0][1];

    return klm->angle;
}

// Used to set angle, this should be set as the starting angle
void Kalman_setAngle(struct Kalman* klm, double newAngle)
{
	klm->angle = newAngle;
}
double Kalman_getRate(struct Kalman* klm)
{
	return klm->rate;
} // Return the unbiased rate

/* These are used to tune the Kalman filter */
void Kalman_setQangle(struct Kalman* klm, double newQ_angle)
{
	klm->Q_angle = newQ_angle;
}
void Kalman_setQbias(struct Kalman* klm, double newQ_bias)
{
	klm->Q_bias = newQ_bias;
}
void Kalman_setRmeasure(struct Kalman* klm, double newR_measure)
{
	klm->R_measure = newR_measure;
}

double Kalman_getQangle(struct Kalman* klm)
{
	return klm->Q_angle;
}
double Kalman_getQbias(struct Kalman* klm)
{
	return klm->Q_bias;
}
double Kalman_getRmeasure(struct Kalman* klm)
{
	return klm->R_measure;
}
