/******************************************************************/
//This file is part of The AHRS Project.
//
//Copyright © 2019 By Nicola di Gruttola Giardino. All rights reserved.
//@mail: nicoladgg@protonmail.com
//
//AHRS is free software: you can redistribute it and/or modify
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


#include "IMU.h"

/*===========================================================================*/
/* Global variables                                                          */
/*===========================================================================*/

kalman k[3];
float gravity[3];
float euler[3];
float last_lla[3]={0,0,0}; //latitude longitude altitude
float lla[3];
int timesexec=0;

float degtorad(float deg) {
    return deg * (M_PI/180);
}

/*===========================================================================*/
/* Low-Pass filter, alpha must be 0<alpha<1                                   */
/*===========================================================================*/
float LowPassFilter(float prev, float measured, float alpha) {
    return prev + alpha * (measured - prev);
}


/*===========================================================================*/
/* set all constant Kalman values (still to add R and Q, R may be a          */
/* diagonal matrix with 0.2 as coefficient                                   */
/*===========================================================================*/
void setKalman(){
    k->P=*Create(2,2);
    k->P_p=*Create(2,2);
    k->K=*Create(2,2);
    k->H=*Create(2,2);
    k->R=*Create(2,2);
    k->Q=*Create(2,2);
    k->A=*Create(2,2);
    k->S=*Create(2,2);
    
    k->x=*Create(2,1);
    k->x_p=*Create(2,1);
    k->y=*Create(2,1);
    k->B=*Create(2,1);
    
    //setting the North Kalman
    k[0].dt=0.001;                                                                                                  //set Dt
    k[0].x.matrix[0][0]=0;      k[0].x.matrix[1][0]=0;                                                            //set x0
    k[0].A.matrix[0][0]=1;   k[0].A.matrix[0][1]=k[0].dt;     k[0].A.matrix[1][0]=0;   k[0].A.matrix[1][1]=1;   //set A
    k[0].B.matrix[0][0]=k[0].dt*k[0].dt/2;     k[0].B.matrix[1][0]=k[0].dt;                                       //set B
    k[0].P.matrix[0][0]=1;    k[0].P.matrix[0][1]=0;    k[0].P.matrix[1][0]=0;    k[0].P.matrix[1][1]=1;   //values either 1 or 0.1, to be decided
    k[0].H.matrix[0][0]=1;    k[0].H.matrix[0][1]=0;    k[0].H.matrix[1][0]=0;    k[0].H.matrix[1][1]=1;        //set H
    k[0].R.matrix[0][0]=0.2;    k[0].R.matrix[0][1]=0;    k[0].R.matrix[1][0]=0;    k[0].R.matrix[1][1]=0.2;    //set R
    
    
    
    //setting the South Kalman
    k[1].dt=0.001;                                                                                                  //set Dt
    k[1].x.matrix[0][0]=0;      k[1].x.matrix[0][1]=0;                                                            //set x0
    k[1].A.matrix[0][0]=1;   k[1].A.matrix[0][1]=k[1].dt;     k[1].A.matrix[1][0]=0;   k[1].A.matrix[1][1]=1;   //set A
    k[1].B.matrix[0][0]=k[1].dt*k[1].dt/2;     k[1].B.matrix[0][1]=k[1].dt;                                       //set B
    k[1].P.matrix[0][0]=1;    k[1].P.matrix[0][1]=0;    k[1].P.matrix[1][0]=0;    k[1].P.matrix[1][1]=1;   //values either 1 or 0.1, to be decided
    k[1].H.matrix[0][0]=1;    k[1].H.matrix[0][1]=0;    k[1].H.matrix[1][0]=0;    k[1].H.matrix[1][1]=1;        //set H
    k[1].R.matrix[0][0]=0.2;    k[1].R.matrix[0][1]=0;    k[1].R.matrix[1][0]=0;    k[1].R.matrix[1][1]=0.2;    //set R
    
    //setting the Down Kalman
    k[2].dt=0.001;                                                                                                  //set Dt
    k[2].x.matrix[0][0]=0;      k[2].x.matrix[0][1]=0;                                                            //set x0
    k[2].A.matrix[0][0]=1;   k[2].A.matrix[0][1]=k[2].dt;     k[2].A.matrix[1][0]=0;   k[2].A.matrix[1][1]=1;   //set A
    k[2].B.matrix[0][0]=k[2].dt*k[2].dt/2;     k[2].B.matrix[0][1]=k[2].dt;                                       //set B
    k[2].P.matrix[0][0]=1;    k[2].P.matrix[0][1]=0;    k[2].P.matrix[1][0]=0;    k[2].P.matrix[1][1]=1;   //values either 1 or 0.1, to be decided
    k[2].H.matrix[0][0]=1;    k[2].H.matrix[0][1]=0;    k[2].H.matrix[1][0]=0;    k[2].H.matrix[1][1]=1;        //set H
    k[2].R.matrix[0][0]=0.2;    k[2].R.matrix[0][1]=0;    k[2].R.matrix[1][0]=0;    k[2].R.matrix[1][1]=0.2;    //set R
}


/*===========================================================================*/
/* Using quaterion to calculate yaw,pitch and roll                           */
/*===========================================================================*/
void calculateYPR(float *q, float *ypr){
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


/*============================================================================*/
/* Using quaterion to calculate rotation matrix and acc North, East and Down  */
/*============================================================================*/
Matrix *calc_acc_vec(Matrix *a, const float offsetx, const float offsety){
    Matrix *acc=Create(3, 1);
    Matrix rotation=*Create(3, 3);

    /*normalize the quaternion
    float n;
    n=invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
    q[0]*=n;
    q[1]*=n;
    q[2]*=n;
    q[3]*=n;
    Already normalized*/


    //calculating rotation matrix
    rotation.matrix[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    rotation.matrix[1][0] = 2*(q1*q2 - q0*q3);
    rotation.matrix[2][0] = 2*(q1*q3 + q0*q2);
    rotation.matrix[0][1] = 2*(q1*q2 + q0*q3);
    rotation.matrix[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    rotation.matrix[2][1] = 2*(q1*q3 - q0*q1);
    rotation.matrix[0][2] = 2*(q1*q3 - q0*q2);
    rotation.matrix[1][2] = 2*(q2*q3 + q0*q1);
    rotation.matrix[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;


    //calculating acceleration vector
    //rotation*g=acc;       inverse rotation;       invrot*acc=a;

    acc=multiply(inverse(&rotation), a);


    //rotating vector to get accN and accE
    acc->matrix[0]-=offsetx;
    acc->matrix[1]-=offsety;

    return acc;
}


/*===========================================================================*/
/* computing latitude, longitude and altitude to get                         */
/* position and velocity                                                     */
/*===========================================================================*/
void compute_GPS(float lla[3], float x[3], float v[3]){
    float d_poles=20004500;
    float r_earth=6378388;
    
    //conversion to radians
    lla[0]+=90;
    lla[1]+=180;
    float rad = degtorad(lla[0]);
    lla[0]=rad;
    rad = degtorad(lla[1]);
    lla[1]=rad;
    
    //convertion to meters
    float meters_lat = lla[0]/M_PI*(d_poles + M_PI*lla[2]);
    float C = 2*M_PI*(r_earth + lla[2])*cosf(lla[0]);
    float meters_lon = lla[1]/(2*M_PI) * C;
    lla[0]=meters_lat;
    lla[1]=meters_lon;
    
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
    
    if (last_lla[0]==0 && last_lla[1]==0 && last_lla[2]==0) { //only the first time
        last_lla[0]=lla[0];
        last_lla[1]=lla[1];
        last_lla[2]=lla[2];
    }
    
    
    for (int i=0; i<3; i++) {
        x[i] = k[i].x.matrix[0][0] + (lla[i] - last_lla[i]);
        v[i] = k[i].x.matrix[1][0] + (x[i] - k[i].x.matrix[0][0])/k[i].dt;
    }
    
    last_lla[0]=lla[0];
    last_lla[1]=lla[1];
    last_lla[2]=lla[2];
}


/*============================================================================*/
/* Using acceleration vector and latlongalt to calculate the velocity         */
/* passing through the Kalman Filter                                          */
/*============================================================================*/
void calculate_velocity(float* velocity, int a){ //passing latlongalt
    float x[3];
    float v[3];

    Matrix acc=*Create(3, 1);
    Matrix rotation=*Create(3, 3);

       /*normalize the quaternion
       float n;
       n=invSqrt(q[0]*q[0]+q[1]*q[1]+q[2]*q[2]+q[3]*q[3]);
       q[0]*=n;
       q[1]*=n;
       q[2]*=n;
       q[3]*=n;
       Already normalized*/


    //calculating rotation matrix
    rotation.matrix[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
    rotation.matrix[1][0] = 2*(q1*q2 - q0*q3);
    rotation.matrix[2][0] = 2*(q1*q3 + q0*q2);
    rotation.matrix[0][1] = 2*(q1*q2 + q0*q3);
    rotation.matrix[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
    rotation.matrix[2][1] = 2*(q1*q3 - q0*q1);
    rotation.matrix[0][2] = 2*(q1*q3 - q0*q2);
    rotation.matrix[1][2] = 2*(q2*q3 + q0*q1);
    rotation.matrix[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;


    //calculating acceleration vector
    //rotation*g=acc;       inverse rotation;       invrot*acc=a;

    acc=multiply(inverse(&rotation), a);


    for (int i=0; i<3; i++) {
        x[i]=k[i].x_p.matrix[0][0];
        v[i]=k[i].x_p.matrix[0][1];
    }
    Matrix GPS=*Create(1, 1);

    //this piece of code works while the GPS retrieves data (the IMU works at 1800Hz, while the GPS works at 1 to 5Hz)
    if(!a){//can also be else if(lla[0]==lastlla[0] &&...&&...)
        for (int i=0; i<3; i++) {
            x[i]+=acc->matrix[0][i]*sampleFreq*sampleFreq;
            v[i]+=acc->matrix[0][i]*sampleFreq;
        }
    }
    //this is the code that computes GPS data into position and velocity
    else{
        compute_GPS(lla, x, v);
    }
    for (int i=0; i<3; i++) {
        GPS.matrix[0][0]=lla[i];
        Kalman_Filter(&k[i], acc->matrix[0][i], &GPS);
        velocity[i]=k[i].x.matrix[1][0];
    }

    Destroy(&GPS);
    Destroy(&acc);
    Destroy(&rotation);
}
