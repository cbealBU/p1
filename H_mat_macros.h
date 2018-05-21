/************    Matrix access macros   ************ */
// By Shengbo LI
// 2008.05.10

#define CLRslr slrVar=0.0

/* Function of vector product: Scalar=Vector * Vector
 * a - vector a, b - vector, num - dim of vector
*/
#define VVP(a, b, num)   CLRslr; for (j=0; j < num; j++)  slrVar += a[j]*b[j]

/* Function of dot product: Vector=Vector * Vector
 * a - vector a, b - vector, num - dim of vector
*/
#define VVD(a, b, num)   for (j=0; j < num; j++)  vecVar[j] = a[j]*b[j]

/* Vector = Matrix * Vector 
 *rowA - row number of matrix, colA - col number of matrix 
 */
#define MVP(A, b, row, col)   for (i=0;i<row;i++) {CLRslr;   for (j=0; j < col; j++)  slrVar += A[i+row*j]*b[j];  vecVar[i]=slrVar; }

/* Vector = Vector * Matrix 
 *rowA - row number of matrix, colA - col number of matrix 
 */
#define VMP(b, A, row, col)   for (j=0; j<col; j++) {CLRslr;   for (i=0; i < row; i++)  slrVar += A[i+row*j]*b[i];  vecVar[j]=slrVar; }

/* Get the n-th row of matrix
 * GROW(A, n, row, col) - n represnts the n-th row 
 */
#define GROW(A, n, row, col) for (i=0; i<col; i++) vecVar[i]=A[n+row*i]

/* Get the n-th col of matrix
 * GCOL(A, n, row,col) - n represnts the n-th row 
 */
#define GCOL(A, n, row, col) for (i=0; i<row; i++) vecVar[i]=A[n*row+i]

 /*  Matrix Transportation*/
//    MATT(A, row, col)
#define MATT(A, row, col) for (i=0; i<row; i++) { for (j=0; j < col; j++)  matVar[i*col+j]= A[i+row*j]; }


 /*  Matrix plus: matrix(or vector)= matrix (vector) + matrix(vector) */
//    MMA(A, B, row, col)
#define MMA(A, B, row, col) for (i=0; i<row; i++) { for (j=0; j < col; j++)  matVar[i+row*j]= A[i+row*j] + B[i+row*j] ; }

 /*  Matrix subtract: matrix(or vector)= matrix (vector) - matrix(vector) */
//    MMS(A, B, row, col)
#define MMS(A, B, row, col) for (i=0; i<row; i++) { for (j=0; j < col; j++)  matVar[i+row*j]= A[i+row*j] - B[i+row*j] ; }

/* Matrix product: Matrix = Matrix * Matrix
* rowA, colA, rowB, colB
 *Note: colA must equal rowA
*/
#define MMP(A, B, rowA, colArowB, colB) for (i=0; i<rowA; i++) {for(j=0; j<colArowB; j++) tempMat[i+rowA*j]=A[i+rowA*j]; } for (i=0; i<rowA; i++) {for(j=0; j<colB; j++) {CLRslr; for (k=0; k < colArowB; k++)  slrVar += tempMat[i+rowA*k]*B[j*colArowB+k]; matVar[i+rowA*j]=slrVar; } }
/*
#define MMP(A, B, rowA, colArowB, colB) 
for (i=0; i<rowA; i++) 
{
	for(j=0; j<colArowB; j++) 
	tempMat[i+rowA*j]=A[i+rowA*j]; 
} 
for (i=0; i<rowA; i++) 
{
	for(j=0; j<colB; j++) 
	{
		CLRslr;
		for (k=0; k < colArowB; k++)  
		{
			slrVar += tempMat[i+rowA*k]*B[j*colArowB+k]; 
		}
		matVar[i+rowA*j]=slrVar; 
	} 
}
*/


/* Debugging */
// n - row numbers
// m - col numbers
#define dispVEC(vec,n,name)   printf("%s=[",name); for (i=0;i<n;i++) printf("%g,\t",(real_T)(vec[i])); printf("]\n")          
#define dispMAT(mat,n,m,name) printf("%s=\n",name); for (i=0;i<n;i++) {for (j=0;j<m;j++) printf("%g,\t",(real_T)(mat[i+n*j])); printf("\n");} printf("\n")
#define dispSLR(slr, name) printf("%s=",name); printf("%g \n", (real_T)slr)
#define dispSTR(name) printf("%s\n", name)
#define dispCtrlPercent(slr) printf("%d per\n", slr)