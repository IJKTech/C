#include "KalmanFilter.h"

/*
* See: http://greg.czerniak.info/guides/kalman1/
*/

static double** A; //State Transition Matrix
static double** B; //Control Input Matrix
static double** H; //Transformation Matrix
static double* x; //state vector
static double** Q; //Process Noise covariance matrix
static double** R; //Measurement Covariance Matrix
static double** P; //The covariance matrix
static double** S; // innovation covariance
static double** K; // Kalman gain
static double* y; //innovation
static double dt; //Delta t
static int n;
static int SuccessfulInit = 0;

static double** malloc2d(size_t size)
{
	int i,j;
	double** a = malloc(size);
	for(i=0; i < size/sizeof(double); i++)
	{
		a[i] = malloc(size);
		for (j=0; j < size/sizeof(double); j++)
		{
			a[i][j] = 0.0;
		}
	}
	return a;

}


static void free2d(double** ptr)
{
	int i;
	for(i = 0; i < n; i++)
    	free(ptr[i]);
	free(ptr);
}

static void printMatrix(double **mat, int size)
{
	int i, j;

	for (i = 0; i < size; i++)
	{
		for (j=0; j< size; j++)
		{
			printf("%.3f\t", mat[i][j]);
		}
		printf("\n");
	} 
}

static void printVector(double* v, int size)
{
	int i;
	for (i=0; i< size; i++)
		printf("%.3f\n", v[i]);
}

static void statePrediction(double *control)
{
	double* v1;
	double* v2;
	v1 = malloc(n*sizeof(double));
	v2 = malloc(n*sizeof(double));

	MatrixVectorMultiply(A, x, n, v1);
	MatrixVectorMultiply(B, control, n, v2);

	int i;
	for (i=0; i<n; i++)
	{
		x[i] = v1[i] + v2[i];
	}
	free(v1);
	free(v2);

}

static void covariancePrediction()
{

	int i, j;
	double** At = malloc2d(n*sizeof(double));

	for(i=0; i < n; i++)
	{
		for(j=0; j < n; j++)
		{
			At[i][j] = A[j][i];
		}
	}

	double** temp = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(P, At, n, temp);
	double** temp2 = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(A, temp, n, temp2);

	for (i=0; i<n; i++)
	{
		for(j=0; j<n; j++)
		{
			P[i][j] = temp2[i][j] + Q[i][j];
		}
	}

	free2d(At);
	free2d(temp);
	free2d(temp2);

}

static void innovation(double *measure)
{

	double *temp = malloc(n*sizeof(double));

	MatrixVectorMultiply(H,x,n, temp);
	int i;
	for (i=0; i<n; i++)
	{
		y[i] = measure[i] - temp[i];
	}

	free(temp);

}

static void innovationCovariance()
{
	int i, j;
	double** Ht = malloc2d(n*sizeof(double));

	for(i=0; i < n; i++)
	{
		for(j=0; j < n; j++)
		{
			Ht[i][j] = H[j][i];
		}
	}

	double** temp = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(P, Ht, n, temp);
	double** temp2 = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(H, temp, n, temp2);

	for (i=0; i<n; i++)
	{
		for(j=0; j<n; j++)
		{
			S[i][j] = temp2[i][j] + R[i][j];
		}
	}

	free2d(Ht);
	free2d(temp);
	free2d(temp2);

}

static void KalmanGain()
{
	int i, j;

	double** Sinv = malloc2d(n*sizeof(double));
	Inverse(S, Sinv, n);

	double** Ht = malloc2d(n*sizeof(double));
	
	for(i=0; i < n; i++)
	{
		for(j=0; j < n; j++)
		{
			Ht[j][i] = H[i][j];
		}
	}

	double** temp = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(Ht, Sinv, n, temp);
	MatrixMatrixMultiply(P,temp, n, K);
	free2d(Sinv);
	free2d(Ht);
	free2d(temp);

}

static void stateUpdate()
{

	int i;
	double* v1 = malloc(n*sizeof(double));
	MatrixVectorMultiply(K, y, n, v1);

	for (i = 0; i < n; i++)
	{
		x[i] += v1[i];
	}

	free(v1);

}

static void covarianceUpdate()
{
	
	int i,j;
	double** Identity = malloc2d(n*sizeof(double));
	for(i=0; i< n; i++)
		Identity[i][i] = 1.0;

	
	double** KH = malloc2d(n*sizeof(double));
	MatrixMatrixMultiply(K,H,n,KH);

	for(i=0; i<n; i++)
		for(j=0; j<n; j++)
			Identity[i][j]-=KH[i][j];

	
	double** Ptemp = malloc2d(n*sizeof(double));
	for(i=0; i<n;i++)
		for(j=0;j<n;j++)
			Ptemp[i][j] = P[i][j];

	MatrixMatrixMultiply(Identity, Ptemp, n, P);

	free2d(Identity);
	free2d(KH);
	free2d(Ptemp);
	
}


void KF_init(double *initialState, int nvars, double timestep)
{
	int i;
	dt = timestep;
	n = nvars;
	//Initial state setup
	x = malloc(nvars*sizeof(double));
	for (i = 0; i < nvars; i++)
		x[i] = initialState[i];
	
	y = malloc(nvars*sizeof(double));
	for (i = 0; i < nvars; i++)
		y[i] = 0.0;
	

	size_t matrixSize = nvars*sizeof(double);
	//State Transition Setup
	
	A = malloc2d(matrixSize);
	B = malloc2d(matrixSize);	
	H = malloc2d(matrixSize);
	Q = malloc2d(matrixSize);
	R = malloc2d(matrixSize);
	P = malloc2d(matrixSize);
	S = malloc2d(matrixSize);
	K = malloc2d(matrixSize);

	for(i = 0; i < nvars; i++)
	{
		A[i][i] = 1.0;
		B[i][i] = 1.0;
		H[i][i] = 1.0;
		P[i][i] = 1.0;
		R[i][i] = 0.2;
	}
	
	//Horrible I know...
	
	A[0][3] = dt; A[1][4] = dt; A[2][5] = dt;
	

	printMatrix(A, 6);

	SuccessfulInit = 1;
}



void KF_tidyup()
{

	if (SuccessfulInit)
	{
		free2d(A);
		free2d(B);
		free2d(H);
		free(x);
		free(y);
		free2d(Q);
		free2d(R);
		free2d(P);
		free2d(S);
		free2d(K);
	}

}


void KF_step(double *control, double *measure)
{
	//printf("- X-original --\n");
	//printVector(x, n);
	//printf("- X-New --\n");
	statePrediction(control);
	//printVector(x, n);
	//getchar();
	

	//printf("- P-original --\n");
	//printMatrix(P, n);
	//printf("- P-New --\n");
	covariancePrediction();
	//printMatrix(P, n);
	//getchar();

	//printf("- Y-original --\n");
	//printVector(y, n);
	//printf("- Y-New --\n");
	innovation(measure);
	//printVector(y, n);
	//getchar();

	//printf("- S-original --\n");
	//printMatrix(S, n);
	//printf("- S-new --\n");
	innovationCovariance();
	//printMatrix(S, n);
	//getchar();


	//printf("- K-original --\n");
	//printMatrix(K, n);
	//printf("- K-New --\n");
	KalmanGain();
	//printMatrix(K, n);
	//getchar();

	//printf("- X-original --\n");
	//printVector(x, n);
	//printf("- X-New --\n");
	stateUpdate();
	//printVector(x, n);
	//getchar();

	//printf("- P-original --\n");
	//printMatrix(P, n);
	//printf("- P-New --\n");
	covarianceUpdate();
	//printMatrix(P, n);
	//getchar();


}

double* KF_getState()
{
	if (SuccessfulInit)
		return x;
	else
		return NULL;
}

double** KF_getCovariance()
{
	if(SuccessfulInit)
		return P;
	else
		return NULL;
}