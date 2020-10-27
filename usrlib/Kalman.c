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

/****************************************************************************************************
* FILE NAME: Kalman.c                                                                               *
*                                                                                                   *
* PURPOSE: This library gives all the tools to implement a Matrix Object                            *
*                                                                                                   *
* FILE REFERENCES:                                                                                  *
*                                                                                                   *
*   Name    I/O     Description                                                                     *
*   ----    ---     -----------                                                                     *
*   none                                                                                            *
*                                                                                                   *
*                                                                                                   *
* EXTERNAL VARIABLES:                                                                               *
*                                                                                                   *
* Source: <matrix.h>                                                                                *
*                                                                                                   *
* Name          Type    IO Description                                                              *
* ------------- ------- -- -----------------------------                                            *
*   m               Matrix      Matrix object, contains                                             *
*                                a float**, being the matrix, and 2 int, being rows and columns     *
*                                                                                                   *
* Source: <Kalman.h>                                                                                *
*                                                                                                   *
* Name          Type    IO Description                                                              *
* ------------- ------- -- -----------------------------                                            *
*   kalman          Kalman      Kalman object, contains                                             *
*                                all the matrices needed for the Kalman FIlter                      *
*                                                                                                   *
*                                                                                                   *
* STATIC VARIABLES:                                                                                 *
*                                                                                                   *
*   Name     Type       I/O      Description                                                        *
*   ----     ----       ---      -----------                                                        *
*   none                                                                                            *
*                                                                                                   *
* EXTERNAL REFERENCES:                                                                              *
*                                                                                                   *
*  Name                       Description                                                           *
*  -------------              -----------                                                           *
*  none                                                                                             *
*                                                                                                   *
* ABNORMAL TERMINATION CONDITIONS, ERROR AND WARNING MESSAGES:                                      *
*    none, compliant with the standard ISO9899:1999                                                 *
*                                                                                                   *
* ASSUMPTIONS, CONSTRAINTS, RESTRICTIONS: tbd                                                       *
*                                                                                                   *
* NOTES: see documentations                                                                         *
*                                                                                                   *
* REQUIREMENTS/FUNCTIONAL SPECIFICATIONS REFERENCES:                                                *
*                                                                                                   *
* DEVELOPMENT HISTORY:                                                                              *
*                                                                                                   *
*   Date          Author            Change Id     Release     Description Of Change                 *
*   ----          ------            ---------     ------      ----------------------                *
*   14-11-2019    N.di Gruttola      1               1         Initial commit                       *
*                   Giardino                                                                        *
*   02-03-2020    N. di Gruttola     2               1.1       Modified libraries, added            *
*                   Giardino                                    my own matrix library               *
*   04-07-2020    N.di Gruttola                      1.2       Added comments, code satisfies       *
*                  Giardino                                     iso9899:1999, as requested per      *
*                                                               MISRA-C:2004                        *
*                                                                                                   *
*                                                                                                   *
*                                                                                                   *
****************************************************************************************************/

/* Include Global Parameters */

#include "Kalman.h"


/********************************************************************************
*                                                                               *
* FUNCTION NAME: predict                                                        *
*                                                                               *
* PURPOSE: Phase 1 of Kalman Filter, predicts the future                        *
                state                                                           *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* k         Kalman*      IO     Kalman structure                                *
* u         float*       I      IMU acceleration data computed                  *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void predict(kalman *k, float u)
{
    
    /* x_p=A*x(n-1) + u_k*b */
    k->x_p=*multiply(&k->A, &k->x);
    
    sum(&k->x_p, m_sc_multiply(&k->B, u));
   
    /* P_p=A*P_n-1*A^T + Q */
    k->P_p=*multiply(&k->A, &k->P);
    k->P_p=*multiply(&k->P_p, transpose(&k->A));
    sum(&k->P_p, &k->Q);
    
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: innovation                                                     *
*                                                                               *
* PURPOSE: Phase 2 of Kalman Filter, innovates the current                      *
                state                                                           *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* k         Kalman       IO     Kalman structure                                *
* u         Matrix*      I      GPS data computed                               *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void innovation(kalman *k, Matrix *z)
{
    //y=z_n - H*x_p
    k->y=*multiply(&k->H, &k->x_p);
    subtract(z, &k->y);
    k->y=*copy(z);
    
    //S=H*P_p*H_T + R
    k->S=*multiply(&k->H, &k->P_p);
    k->S=*multiply(&k->S, transpose(&k->H));
    sum(&k->S, &k->R);
    
    //K=P_p*H_T*S^-1
    k->K=*multiply(&k->P_p, transpose(&k->H));
    k->K=*multiply(&k->K, inverse(&k->S));
    
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: update                                                         *
*                                                                               *
* PURPOSE: Phase 3 of Kalman Filter, updates the current                        *
                state                                                           *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* k         Kalman       IO     Kalman structure                                *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void update(kalman *k)
{
    Matrix I=*identity(2);
    //x_n=x_p+Ky
    add(&k->x_p, multiply(&k->K, &k->y));
    k->x=*copy(&k->x_p);
    
    //P=(I-K*H)*P_p
    subtract(&I, multiply(&k->K, &k->H));
    k->P=*multiply(&I, &k->P_p);
    
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: Kalman_Filter                                                  *
*                                                                               *
* PURPOSE: Main function of the Kalman Filter, calls the 3 functions            *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* k         Kalman       IO     Kalman structure                                *
* GPS       Matrix*      I      GPS data computed                               *
* a         float        I      IMU acceleration data computed                  *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void Kalman_Filter(kalman *k, float a, Matrix *GPS)
{
    predict(k, a);
    innovation(k, GPS);
    update(k);
}
