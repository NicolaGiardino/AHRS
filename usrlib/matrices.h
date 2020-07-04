/***********************************************************************************
* This file is part of The AHRS Project.                                           *
*                                                                                  *
* Copyright © 2020 By Nicola di Gruttola Giardino. All rights reserved.            *
* @mail: nicoladgg@protonmail.com                                                  *
*                                                                                  *
* AHRS is free software: you can redistribute it and/or modify                     *
* it under the terms of the GNU General Public License as published by             *
* the Free Software Foundation, either version 3 of the License, or                *
* (at your option) any later version.                                              *
*                                                                                  *
* AHRS is distributed in the hope that it will be useful,                          *
* but WITHOUT ANY WARRANTY; without even the implied warranty of                   *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the                    *
* GNU General Public License for more details.                                     *
*                                                                                  *
* You should have received a copy of the GNU General Public License                *
* along with The AHRS Project.  If not, see <https://www.gnu.org/licenses/>.       *
*                                                                                  *
* In case of use of this project, I ask you to mention me, to whom it may concern. *
***********************************************************************************/

/***************************************************************************************************
*   FILENAME:  matrices.h                                                                          *
*                                                                                                  *
*                                                                                                  *
*   PURPOSE:   Library that defines the object Matrix and all its functions                        *
*                                                                                                  *
*                                                                                                  *
*                                                                                                  *
*   GLOBAL VARIABLES:                                                                              *
*                                                                                                  *
*                                                                                                  *
*   Variable        Type        Description                                                        *
*   --------        ----        -------------------                                                *
*   m               Matrix      Matrix object, contains                                            *
*                                a float**, being the matrix, and 2 int, being rows and columns    *
*                                                                                                  *
*   DEVELOPMENT HISTORY :                                                                          *
*                                                                                                  *
*                                                                                                  *
*   Date          Author            Change Id     Release     Description Of Change                *
*   ----          ------            -------- -    ------      ----------------------               *
*   14-11-2019    N.di Gruttola      1               1         Initial commit                      *
*                   Giardino                                                                       *
*   02-03-2020    N. di Gruttola     2               1.1       Modified libraries, added           *
*                   Giardino                                    my own matrix library              *
*   04-07-2020    N.di Gruttola                      1.2       Added comments, code satisfies      *
*                  Giardino                                     iso9899:1999, as requested per     *
*                                                               MISRA-C:2004                       *
*                                                                                                  *
***************************************************************************************************/

#ifndef matrix_h
#define matrix_h

/* Include Global Parameters */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

/*
* Matrix Object:
*       float** being the pointer to the matrix
*       int c and r are no. of columns and no. of rows
*/

typedef struct Matrix 
{
    float** matrix;        
    int     c;                  
    int     r;                  
}Matrix;

/* Declare Prototypes */

int      ZeroMat        (Matrix *);
Matrix*  Create         (int , int);
int      Destroy        (Matrix *);
int      sum            (Matrix *, Matrix *);
Matrix*  multiply       (Matrix *, Matrix *);
int      subtract       (Matrix *, Matrix *);
int      sc_multiply    (Matrix *, float);
Matrix*  inverse        (Matrix *);
Matrix*  identity       (int);
Matrix*  transpose      (Matrix *);
int      equals         (Matrix *, Matrix *);
float    determinant    (Matrix *);
int      print          (Matrix *);
float*   eigen          (Matrix *);
Matrix*  copy           (Matrix *);
int      rowSwap        (Matrix *, int, int);
int      reduce         (Matrix *, int , int , float);
Matrix*  Chol           (Matrix *);
int      LU             (Matrix *, Matrix *, Matrix *);
Matrix*  sqrtm          (Matrix *);
Matrix*  msum           (Matrix *, Matrix *);
Matrix*  m_sc_multiply  (Matrix *, float );
Matrix*  expm           (Matrix *, float);
Matrix*  msubtract      (Matrix *, Matrix *);
Matrix*  blkdiag        (Matrix *, Matrix *, Matrix *);
Matrix*  diag           (Matrix *);
float    interp1        (float , float *, float *);
float*   CsvToFloat     (const char *);
float    randn          ();
void     seed           (const float);
#endif /* matrix_h */
