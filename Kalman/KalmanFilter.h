#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#include "Matrices.h"
#include <stdlib.h>

void KF_init(double *initialState, int nvars, double timestep);
void KF_tidyup();
void KF_step(double *control, double *measure);
double* KF_getState();
double** KF_getCovariance();

#endif