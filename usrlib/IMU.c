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
* FILE NAME: IMU.c                                                                                  *
*                                                                                                   *
* PURPOSE: This library gives all the tools to implement the whole AHRS algorithm                   *
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
* kalman      Kalman      Kalman object, contains                                                   *
*                                all the matrices needed for the Kalman FIlter                      *
*                                                                                                   *
*                                                                                                   *
* GLOABL VARIABLES:                                                                                 *
*                                                                                                   *
*   Name       Type       I/O      Description                                                      *
*   ----       ----       ---      -----------                                                      *
*   k          Kalman[3]  IO       Kalman structure                                                 *
*   gravity    float[3]   IO                                                                        *
*   euler      float[3]   I                                                                         *
*   last_lla   float[3]   I        Previous long-lat-alt GPS data                                   *
*   timesexec  int        I                                                                         *
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

#include "IMU.h"

/* Global variables */

kalman k[3];
float  gravity[3];
float  euler[3];
float  last_lla[3] ={0,0,0}; //latitude longitude altitude
float  lla[3];
int    timesexec   =0;


/********************************************************************************
*                                                                               *
* FUNCTION NAME: degtorad                                                       *
*                                                                               *
* PURPOSE: Converts degrees to radians                                          *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* deg       float        I      degrees                                         *
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/
float fDegtorad(float deg)
{
    return deg * (M_PI/180);
}


/********************************************************************************
*                                                                               *
* FUNCTION NAME: LowPassFilter                                                  *
*                                                                               *
* PURPOSE: Low Pass filter                                                      *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* prev      float        I      previous state                                  *
* measured  float        I      new measured state                              *
* alpha     float        I      must be btw 0<alpha<1                           *
*                                                                               *
* RETURN VALUE: float                                                           *
*                                                                               *
********************************************************************************/
float fLowPassFilter(float prev, float measured, float alpha)
{
    return prev + alpha * (measured - prev);
}


/********************************************************************************
*                                                                               *
* FUNCTION NAME: setKalman                                                      *
*                                                                               *
* PURPOSE: set all constant Kalman values (still to add R and Q, R may be a     *
* diagonal matrix with 0.2 as coefficient                                       *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void vSetup_Kalman()
{
    k->P = pxCreate(2, 2);
    k->K = pxCreate(2, 2);
    k->H = pxCreate(2, 2);
    k->R = pxCreate(2, 2);
    k->Q = pxCreate(2, 2);
    k->A = pxCreate(2, 2);
    k->S = pxCreate(2, 2);

    k->x = pxCreate(2, 1);
    k->y = pxCreate(2, 1);
    k->B = pxCreate(2, 1);

    //setting the North Kalman
    //set Dt
    k[0].dt=0.001;
    //set x0                                                                                        
    k[0].x->matrix[0][0]=0;
    k[0].x->matrix[1][0] = 0;
    //set A
    k[0].A->matrix[0][0] = 1;
    k[0].A->matrix[0][1] = k[0]dt;
    k[0].A->matrix[1][0] = 0;
    k[0].A->matrix[1][1] = 1;
    //set B
    k[0].B->matrix[0][0] = k[0].dt * k[0].dt / 2;
    k[0].B->matrix[1][0] = k[0].dt;
    //values either 1 or 0.1, to be decided
    k[0].P->matrix[0][0] = 1;
    k[0].P->matrix[0][1] = 0;
    k[0].P->matrix[1][0] = 0;
    k[0].P->matrix[1][1] = 1;
    //set H
    k[0].H->matrix[0][0] = 1;
    k[0].H->matrix[0][1] = 0;
    k[0].H->matrix[1][0] = 0;
    k[0].H->matrix[1][1] = 1; 
    //set R
    k[0].R->matrix[0][0] = 0.2;
    k[0].R->matrix[0][1] = 0;
    k[0].R->matrix[1][0] = 0;
    k[0].R->matrix[1][1] = 0.2;

    //setting the South Kalman
    k[1].dt=0.001;                                                                                                  
    k[1].x->matrix[0][0]=0;      k[1].x.matrix[0][1]=0;                                                           
    k[1].A->matrix[0][0] = 1;
    k[1].A->matrix[0][1] = k[1].dt;
    k[1].A->matrix[1][0] = 0;
    k[1].A->matrix[1][1] = 1;                                                                                 
    k[1].A->matrix[0][0] = 1;
    k[1].A->matrix[0][1] = k[1].dt;
    k[1].A->matrix[1][0] = 0;
    k[1].A->matrix[1][1] = 1;                                                                                     
    k[1].B->matrix[0][0] = k[1].dt * k[1].dt / 2;
    k[1].B->matrix[0][1] = k[1].dt;                                                                       
    k[1].P->matrix[0][0] = 1;
    k[1].P->matrix[0][1] = 0;
    k[1].P->matrix[1][0] = 0;
    k[1].P->matrix[1][1] = 1;                                                                                  
    k[1].H->matrix[0][0] = 1;
    k[1].H->matrix[0][1] = 0;
    k[1].H->matrix[1][0] = 0;
    k[1].H->matrix[1][1] = 1;                                                                               
    k[1].R->matrix[0][0] = 0.2;
    k[1].R->matrix[0][1] = 0;
    k[1].R->matrix[1][0] = 0;
    k[1].R->matrix[1][1] = 0.2; //set R

    //setting the Down Kalman
    k[2].dt=0.001;                                                                                               
    k[2].x->matrix[0][0] = 0;
    k[2].x->matrix[0][1] = 0;                                                                                  
    k[2].A->matrix[0][0] = 1;
    k[2].A->matrix[0][1] = k[2].dt;
    k[2].A->matrix[1][0] = 0;
    k[2].A->matrix[1][1] = 1;                                                                                   
    k[2].B->matrix[0][0] = k[2].dt * k[2].dt / 2;
    k[2].B->matrix[0][1] = k[2].dt;                                                                   
    k[2].P->matrix[0][0] = 1;
    k[2].P->matrix[0][1] = 0;
    k[2].P->matrix[1][0] = 0;
    k[2].P->matrix[1][1] = 1;                                                                                  
    k[2].H->matrix[0][0] = 1;
    k[2].H->matrix[0][1] = 0;
    k[2].H->matrix[1][0] = 0;
    k[2].H->matrix[1][1] = 1;                                                                                 
    k[2].R->matrix[0][0] = 0.2;
    k[2].R->matrix[0][1] = 0;
    k[2].R->matrix[1][0] = 0;
    k[2].R->matrix[1][1] = 0.2; //set R
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: calculateYPR                                                   *
*                                                                               *
* PURPOSE: Using quaterion to calculate yaw,pitch and roll                      *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* q         float*       O      Quaternions                                     *
* ypr       float*       O      Yaw Pitch and Roll                              *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void vCalculateYPR(float *q, float *ypr)
{
    // calculate gravity vector
    gravity[0] = 2 * (q1*q3 - q0*q2);
    gravity[1] = 2 * (q0*q1 + q2*q3);
    gravity[2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;


    // calculate yaw/pitch/roll angles
    ypr[0] = atan2(2*q1*q2 - 2*q0*3, 2*q0*q0 + 2*q1*q1 - 1);
    ypr[1] = atan(gravity[0] / sqrt(gravity[1]*gravity[1] + gravity[2]*gravity[2]));
    ypr[2] = atan(gravity[1] / sqrt(gravity[0]*gravity[0] + gravity[2]*gravity[2]));


    // calculate Euler angles (they're also yaw, pitch and roll, differences tbd, Wikipedia uses those ones)
    euler[0] = atan2(2*q[1]*q[2] - 2*q[0]*q[3], 2*q[0]*q[0] + 2*q[1]*q[1] - 1);
    euler[1] = -asin(2*q[1]*q[3] + 2*q[0]*q[2]);
    euler[2] = atan2(2*q[2]*q[3] - 2*q[0]*q[1], 2*q[0]*q[0] + 2*q[3]*q[3] - 1);
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: calc_acc_vec                                                   *
*                                                                               *
* PURPOSE: Using quaterion to calculate rotation matrix and                     *
*              acc North, East and Down                                         *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* a         Matrix*      I      Accelerometer data                              *
* offsetx   const float  I      x axis offset                                   *
* offsety   const float  I      y axis offset                                   *
*                                                                               *
* RETURN VALUE: Matrix*                                                         *
*                                                                               *
********************************************************************************/
Matrix *pxCalc_acc_vec(Matrix *a, const float offsetx, const float offsety)
{
    Matrix *acc = pxCreate(3, 1);
    Matrix *rotation = pxCreate(3, 3);

    /*normalize the quaternion
    float n;
    n=invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    q[0]*=n;
    q[1]*=n;
    q[2]*=n;
    q[3]*=n;
    Already normalized*/


    //calculating rotation matrix
    rotation->matrix[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    rotation->matrix[1][0] = 2 * (q1 * q2 - q0 * q3);
    rotation->matrix[2][0] = 2 * (q1 * q3 + q0 * q2);
    rotation->matrix[0][1] = 2 * (q1 * q2 + q0 * q3);
    rotation->matrix[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    rotation->matrix[2][1] = 2 * (q1 * q3 - q0 * q1);
    rotation->matrix[0][2] = 2 * (q1 * q3 - q0 * q2);
    rotation->matrix[1][2] = 2 * (q2 * q3 + q0 * q1);
    rotation->matrix[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    //calculating acceleration vector
    //rotation*g=acc;       inverse rotation;       invrot*acc=a;

    Matrix *app;
    app = pxInverse(rotation);
    iMultiply(acc, app, a);

    vDestroy(app);
    vDestroy(rotation);

    //rotating vector to get accN and accE
    acc->matrix[0]-=offsetx;
    acc->matrix[1]-=offsety;

    return acc;
}

int iCalc_acc_vec(Matrix* acc, Matrix *a, const float offsetx, const float offsety)
{
    if(acc != NULL)
    {
        return -1;
    }

    acc = pxCalc_acc_vec(acc, a, offsetx, offsety);

    return 0;
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: compute_GPS                                                    *
*                                                                               *
* PURPOSE: computing latitude, longitude and altitude to get                    *
*           position and velocity                                               *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* lla       float[3]     I      Lat-lon-alt                                     *
* x         float[3]     O      position                                        *
* v         float[3]     O      velocity                                        *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void vCompute_GPS(float lla[3], float x[3], float v[3])
{
    float d_poles = 20004500;
    float r_earth = 6378388;

    //conversion to radians
    lla[0] += 90;
    lla[1] += 180;
    float rad;
    rad = fDegtorad(lla[0]);
    lla[0] = rad;
    rad = fDegtorad(lla[1]);
    lla[1] = rad;

    //convertion to meters
    float meters_lat;
    meters_lat = lla[0] / M_PI * (d_poles + M_PI * lla[2]);
    float C;
    C = 2 * M_PI * (r_earth + lla[2]) * cosf(lla[0]);
    float meters_lon;
    meters_lon = lla[1] / (2 * M_PI) * C;
    lla[0] = meters_lat;
    lla[1] = meters_lon;

    /*========================================================================
    //this code will not be used, this is to get position from lat/lon/alt
    int R = 6371; // Radius of the earth in km
    float dLat = degtorad(lat-last_ll[0]);
    float dLon = degtorad(lon-last_ll[1]);
    float a = sin(dLat/2) * sin(dLat/2) + cos(degtorad(last_ll[0])) * cos(degtorad(lat)) * sin(dLon/2) * sin(dLon/2);
    float c = 2 * atan2(sqrt(a), sqrt(1-a));
    x = k.xo[0] + R * c*1000; // Distance in m
    v = k.x0[1] + (x - k.x0[0])/k.dt;
    ========================================================================*/
    
    if (last_lla[0]==0 && last_lla[1]==0 && last_lla[2]==0) 
    {
        //only the first time
        last_lla[0] = lla[0];
        last_lla[1] = lla[1];
        last_lla[2] = lla[2];
    }

    for (int i = 0; i < 3; i++)
    {
        x[i] = k[i].x.matrix[0][0] + (lla[i] - last_lla[i]);
        v[i] = k[i].x.matrix[1][0] + (x[i] - k[i].x.matrix[0][0]) / k[i].dt;
    }

    last_lla[0] = lla[0];
    last_lla[1] = lla[1];
    last_lla[2] = lla[2];
}

/********************************************************************************
*                                                                               *
* FUNCTION NAME: vCompute_GPS                                                   *
*                                                                               *
* PURPOSE: Using acceleration vector and latlongalt to calculate the velocity   *
*               passing through the Kalman Filter                               *
*                                                                               *
* ARGUMENT LIST:                                                                *
*                                                                               *
* Argument  Type         IO     Description                                     *
* --------- --------     --     ---------------------------------               *
* lla       float*       O      velocity                                        *
* x         float3]      I       lat/long/alt                                   *
*                                                                               *
* RETURN VALUE: void                                                            *
*                                                                               *
********************************************************************************/
void vCalculate_velocity(float* velocity, int a)
{ 
    float x[3];
    float v[3];

    Matrix *acc = pxCreate(3, 1);
    Matrix *rotation = pxCreate(3, 3);

    /*normalize the quaternion
       float n;
       n=invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
       q[0]*=n;
       q[1]*=n;
       q[2]*=n;
       q[3]*=n;
       Already normalized*/


    //calculating rotation matrix
    rotation->matrix[0][0] = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    rotation->matrix[1][0] = 2 * (q1 * q2 - q0 * q3);
    rotation->matrix[2][0] = 2 * (q1 * q3 + q0 * q2);
    rotation->matrix[0][1] = 2 * (q1 * q2 + q0 * q3);
    rotation->matrix[1][1] = q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3;
    rotation->matrix[2][1] = 2 * (q1 * q3 - q0 * q1);
    rotation->matrix[0][2] = 2 * (q1 * q3 - q0 * q2);
    rotation->matrix[1][2] = 2 * (q2 * q3 + q0 * q1);
    rotation->matrix[2][2] = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;

    //calculating acceleration vector
    //rotation*g=acc;       inverse rotation;       invrot*acc=a;
    Matrix* app;
    app = pxInverse(rotation);
    iMultiply(acc, app, a);
    vDestroy(app);

    for (int i = 0; i < 3; i++)
    {
        x[i] = k[i].x->matrix[0][0];
        v[i] = k[i].x->matrix[0][1];
    }
    Matrix *GPS = pxCreate(1, 1);

    //this piece of code works while the GPS retrieves data (the IMU works at 1800Hz, while the GPS works at 1 to 5Hz)
    if(!a)
    {//can also be else if(lla[0]==lastlla[0] &&...&&...)
        for (int i=0; i<3; i++) 
        {
            x[i] += acc->matrix[0][i] * sampleFreq * sampleFreq;
            v[i] += acc->matrix[0][i] * sampleFreq;
        }
    }
    //this is the code that computes GPS data into position and velocity
    else
    {
        vCompute_GPS(lla, x, v);
    }
    for (int i = 0; i < 3; i++)
    {
        GPS.matrix[0][0] = lla[i];
        vKalman_Filter(&k[i], acc->matrix[0][i], GPS);
        velocity[i] = k[i].x->matrix[1][0];
    }

    vDestroy(GPS);
    vDestroy(acc);
    vDestroy(rotation);
}

void vDelete_Kalman()
{

    vDestroy(k->x);
    vDestroy(k->P);
    vDestroy(k->y);
    vDestroy(k->B);
    vDestroy(k->K);
    vDestroy(k->H);
    vDestroy(k->R);
    vDestroy(k->Q);
    vDestroy(k->A);
    vDestroy(k->S);

}