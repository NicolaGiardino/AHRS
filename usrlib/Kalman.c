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
* FUNCTION NAME: vPredict                                                       *
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
void vPredict(kalman *k, float u)
{
    Matrix *app;
    Matrix *app2;

    /* x_p=A*x(n-1) + u_k*b */
    app = pxMultiply(k->A, k->x);
    app2 = pxSc_Multiply(k->B, u);
    iSum(k->x, app, app2);
    vDestroy(app);
    vDestroy(app2);

    /* P_p=A*P_n-1*A^T + Q */
    app = pxMultiply(k->A, k->P);
    app2 = transpose(k->A);
    iMultiply(k->P, app, app2);
    vDestroy(app);
    vDestroy(app2);

    app = pxCopy(k->P);
    iSum(k->P, app, k->Q);
    vDestroy(app);
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: vInnovation                                                    *
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
void vInnovation(kalman *k, Matrix *z)
{

    Matrix *app;
    Matrix *app2;

    /* y=z_n - H*x_p */
    iMultiply(k->y , k->H, k->x);
    app = pxCopy(k->y);
    iSubtract(k->y, z, app);
    vDestroy(app);

    /* S=H*P_p*H_T + R */
    iMultiply(k->S, k->H, k->P);

    app = pxCopy(k->S);
    app2 = transpose(k->H);
    iMultiply(k->S, app, app2);
    vDestroy(app);
    vDestroy(app2);

    app=pxCopy(k->S);
    iSum(k->S, app, k->R);
    vDestroy(app);

    /* K=P_p*H_T*S^-1 */
    app = transpose(k->H);
    iMultiply(k->K, k->P, app);
    vDestroy(app);

    app = pxCopy(k->K);
    app2 = pxInverse(k->S);
    iMultiply(k->K, app, app2);
    vDestroy(app);
    vDestroy(app2);
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: vUpdate                                                        *
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
void vUpdate(kalman *k)
{
    Matrix *I = pxIdentity(2);
    Matrix *app;
    Matrix *app2;

    /* x_n=x_p+Ky */
    app = pxCopy(k->x);
    app2 = pxMultiply(k->K, k->y);
    iSum(k->x, app, app2);
    vDestroy(app);
    vDestroy(app2);

    /* P=(I-K*H)*P_p */
    app = pxMultiply(k->K, k->H);
    app2 = pxSubtract(I, app);
    vDestroy(app);
    iMultiply(k->P, app2, k->P);
    vDestroy(app2);
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
void vKalman_Filter(kalman *k, float a, Matrix *GPS)
{
    vPredict(k, a);
    vInnovation(k, GPS);
    vUpdate(k);
}
