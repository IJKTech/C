#ifndef MATRICES_H
#define MATRICES_H

#include <math.h>
#include <stdlib.h>
#include <stdio.h>

void MatrixVectorMultiply(double **m, double *v, int n, double *vn);

void MatrixMatrixMultiply(double **m1, double **m2, int n, double **res);

void Inverse(double **a, double **inv_a, int n);

double Determinant(double **a,int n);

void CoFactor(double **a,int n,double **b);

void Transpose(double **a,int n);

#endif