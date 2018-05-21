#ifndef __S_MATRIX_H__
#define __S_MATRIX_H__

static void sMatAdd(real_T *A, real_T *B, real_T *C, int_T m, int_T n);
static void sMatSub(real_T *A, real_T *B, real_T *C, int_T m, int_T n);
static void sMatMult(real_T *A, real_T *B, real_T *C, int_T l, int_T m, int_T n);
static void sMatInv(real_T *A, real_T *B, int_T m);
static void sMatTrsp(real_T *A, real_T *B, int_T m, int_T n);
static void sMatEye(real_T *A, int_T m);

#endif
