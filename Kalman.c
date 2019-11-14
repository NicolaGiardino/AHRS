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

#include "Kalman.h"
/*
k->P=*constructor(2,2);
k->P_p=*constructor(2,2);
k->K=*constructor(2,2);
k->H=*constructor(2,2);
k->R=*constructor(2,2);
k->Q=*constructor(2,2);
k->A=*constructor(2,2);
k->S=*constructor(2,2);

k->x=*constructor(1,2);
k->x_p=*constructor(1,2);
k->y=*constructor(1,2);
k->B=*constructor(1,2);
 */


/*====================================*/
/* Predict state                      */
/*====================================*/
void predict(kalman *k, float u){
    
    //x_p=A*x(n-1) + u_k*b
    k->x_p=*multiply(&k->A, &k->x);
    scalar_multiply(&k->B, u);
    add(&k->x_p, &k->B);
    
    //P_p=A*P_n-1*A^T + Q
    k->P_p=*multiply(&k->A, &k->P);
    k->P_p=*multiply(&k->P_p, transpose(&k->A));
    add(&k->P_p, &k->Q);
    
}

/*====================================*/
/* Innovation state                   */
/*====================================*/
void innovation(kalman *k, Matrix *z){//y gps data, z[1][2]
    //y=z_n - H*x_p
    k->y=*multiply(&k->H, &k->x_p);
    subtract(z, &k->y);
    k->y=*clone(z);
    
    //S=H*P_p*H_T + R
    k->S=*multiply(&k->H, &k->P_p);
    k->S=*multiply(&k->S, transpose(&k->H));
    add(&k->S, &k->R);
    
    //K=P_p*H_T*S^-1
    k->K=*multiply(&k->P_p, transpose(&k->H));
    k->K=*multiply(&k->K, inversion(&k->S));
    
}

/*====================================*/
/* Update state                       */
/*====================================*/
void update(kalman *k){
    Matrix I=*identity(2);
    //x_n=x_p+Ky
    add(&k->x_p, multiply(&k->K, &k->y));
    k->x=*clone(&k->x_p);
    
    //P=(I-K*H)*P_p
    subtract(&I, multiply(&k->K, &k->H));
    k->P=*multiply(&I, &k->P_p);
    
}

/*====================================*/
/* Main Kalman Function               */
/*====================================*/
void Kalman_Filter(kalman *k, float a, Matrix *GPS){
    predict(k, a);
    innovation(k, GPS);
    update(k);
}
