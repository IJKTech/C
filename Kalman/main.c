#include "Matrices.h"
#include "KalmanFilter.h"
#include "stdlib.h"
#include "stdio.h"
#include <math.h>

void MatrixTests();
void Progress(double* measure);

static double dt = 0.1;


int main()
{

	double start[6] = {0.0,500.0,0.0, 70.0, 70.0,0.0};




	KF_init(start, 6, dt);

	double control[6] = {0.0, -5.0*dt*dt, 0.0, 0.0, -10.0*dt, 0.0};
	//double control[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	double measure[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
	int i = 0;
	
	FILE *file = NULL;
	file = fopen("results.txt", "w");

	for(i = 0; i<150; i++){
		
		Progress(measure);

		KF_step(control, measure);
		
		double *s = KF_getState();

		fprintf(file, "%.3f, %.3f, %.3f, %.3f, %.3f, %.3f\n", s[0], s[1], s[2], measure[0], measure[1], measure[2]);

	}
	
	fclose(file);
	KF_tidyup();
	return 0;
}


void Progress(double* measure)
{
	static double x = 0.0;
	static double y = 0.0;
	static double z = 0.0;
	static double vx = 70;
	static double vy = 70;
	static double vz = 0.0;
	double mean = 150;

	x += vx*dt;
	y += vy*dt;
	z += vz*dt;
	vy -= 10*dt;
	measure[0] = x -mean/2.0 + mean*(double)rand()/(double)RAND_MAX;
	measure[1] = y -mean/2.0 + mean*(double)rand()/(double)RAND_MAX;
	measure[2] = z;
	measure[3] = vx -mean/2.0 + mean*(double)rand()/(double)RAND_MAX;
	measure[4] = vy -mean/2.0 + mean*(double)rand()/(double)RAND_MAX;
	measure[5] = vz;
}


void MatrixTests()
{
	double **a;
	double **b;
	int i, j;
	double **id;

	a = malloc(3*sizeof(double));
	for (i=0; i < 3; i++)
		a[i] = malloc(3*sizeof(double));

	b = malloc(3*sizeof(double));
	for (i=0; i < 3; i++)
		b[i] = malloc(3*sizeof(double));

	id = malloc(3*sizeof(double));
	for (i=0; i < 3; i++)
		id[i] = malloc(3*sizeof(double));

	a[0][0] = 1; a[0][1] = 2; a[0][2] = 0;
	a[1][0] = -1; a[1][1] = 1; a[1][2] = 1;
	a[2][0] = 1; a[2][1] = 2; a[2][2] = 3;

	
	Inverse(a, b, 3);

	for (i=0; i < 3; i++)
	{
		for (j=0; j < 3; j++)
		{
			printf("%.3f\n",b[i][j] );		
		}
	} 

	MatrixMatrixMultiply(a,b, 3, id);

	for (i=0; i < 3; i++)
	{
		for (j=0; j < 3; j++)
		{
			printf("%.3f\n",id[i][j] );		
		}
	} 

	double v[3] = {1.0, 2.0, 3.0};
	double vn[3];

	MatrixVectorMultiply(a, v, 3, vn);
	for (i=0; i<3; i++)
		printf("%.3f\n", vn[i]);


	free(a); free(b); free(id);
}