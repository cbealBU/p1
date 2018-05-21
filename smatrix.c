/*
 * File: smatrix.c
 * Author: Jihan Ryu
 * Version: 1.2
 * Date: 07/10/2003
 * Abstract:
 *	Simple matrix operation library.
 */

#include "smatrix.h"

static void sMatAdd(real_T *A, real_T *B, real_T *C, int_T m, int_T n)
{
	int_T i;

	for (i = 0; i < m*n; i++) {
		C[i] = A[i] + B[i];
	}
}

static void sMatSub(real_T *A, real_T *B, real_T *C, int_T m, int_T n)
{
	int_T i;

	for (i = 0; i < m*n; i++) {
		C[i] = A[i] - B[i];
	}
}

static void sMatMult(real_T *A, real_T *B, real_T *C, int_T l, int_T m, int_T n)
{
	int_T i, j, k;
	real_T temp;

	for (i = 0; i < l; i++) {
		for (j = 0; j < n; j++) {
			temp = 0;
			for (k = 0; k < m; k++) {
				temp += A[i*m+k]*B[k*n+j];
			}
			C[i*n+j] = temp;
		}
	}
}

static void sMatInv(real_T *A, real_T *B, int_T m)
{
	real_T det;

	if (m == 1) {
		if (A[0] == 0) {
			printf("Singular... [sMatInv] \n");
			B[0] = 0;
			return;
		}
		B[0] = 1 / A[0];
	}
	else if (m == 2) {
		det = A[0*m+0]*A[1*m+1] - A[0*m+1]*A[1*m+0];
		if (det == 0) {
			printf("Singular... [sMatInv] \n");
			B[0*m+0] = B[0*m+1] = B[1*m+0] = B[0*m+1] = 0;
			return;
		}
		B[0*m+0] = A[1*m+1] / det;
		B[0*m+1] = -A[0*m+1] / det;
		B[1*m+0] = -A[1*m+0] / det;
		B[1*m+1] = A[0*m+0] / det;
	}
	else {
		printf("Not implemented... [sMatInv] \n");
	}
}

static void sMatTrsp(real_T *A, real_T *B, int_T m, int_T n)
{
	int_T i, j;

	for (i = 0; i < m; i++) {
		for (j = 0; j < n; j++) {
			B[j*m+i] = A[i*n+j];
		}
	}	
}

static void sMatEye(real_T *A, int_T m)
{
	int_T i, j;
	
	for (i = 0; i < m; i++) {
		for (j = 0; j < m; j++) {
			A[i*m+j] = i == j ? 1 : 0;
		}
	}
}
