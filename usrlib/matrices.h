/******************************************************************/
//This file is part of The AHRS Project.
//
//Copyright © 2020 By Nicola di Gruttola Giardino. All rights reserved.
//@mail: nicoladgg@protonmail.com
//
//SOC-SPKF is free software: you can redistribute it and/or modify
//it under the terms of the GNU General Public License as published by
//the Free Software Foundation, either version 3 of the License, or
//(at your option) any later version.
//
//AHRS is distributed in the hope that it will be useful,
//but WITHOUT ANY WARRANTY; without even the implied warranty of
//MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//GNU General Public License for more details.
//
//You should have received a copy of the GNU General Public License
//along with The AHRS Project.  If not, see <https://www.gnu.org/licenses/>.
//
//In case of use of this project, I ask you to mention me, to whom it may concern.
/******************************************************************/

#ifndef matrices_h
#define matrices_h

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct Matrix {
    float **matrix;
    int c;
    int r;
}Matrix;

int ZeroMat(Matrix*);//
Matrix* Create(int,int);//
int Destroy(Matrix*);//
int sum(Matrix*,Matrix*);//
Matrix* multiply(Matrix*,Matrix*);//
int subtract(Matrix*,Matrix*);//
int sc_multiply(Matrix*,float);//
Matrix* inverse(Matrix*);//
Matrix* identity(int);//
Matrix* transpose(Matrix*);//
int equals(Matrix*,Matrix*);//
float determinant(Matrix*);//
int print(Matrix*);//
float* eigen(Matrix*);//
Matrix* copy(Matrix*);//
int rowSwap(Matrix*,int,int);//
int reduce(Matrix*,int,int,float);//
Matrix* Chol(Matrix*);//
int LU(Matrix*,Matrix*,Matrix*);//
Matrix* sqrtm(Matrix*);
Matrix* msum(Matrix*,Matrix*);//
Matrix* m_sc_multiply(Matrix* m1, float f);//
Matrix* expm(Matrix*,float);
Matrix* msubtract(Matrix*,Matrix*);//
Matrix* blkdiag(Matrix*,Matrix*,Matrix*);//
Matrix* diag(Matrix*);//
#endif /* matrices_h */
