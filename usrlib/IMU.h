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
*   FILENAME:  IMU.h                                                                               *
*                                                                                                  *
*                                                                                                  *
*   PURPOSE:   Library that defines all the fuction to run the Attitude and Heading				   *
					Reference System Algorithm													   *
*                                                                                                  *
*                                                                                                  *
*                                                                                                  *
*   GLOBAL VARIABLES:                                                                              *
*                                                                                                  *
*                                                                                                  *
*   Variable        Type        Description                                                        *
*   --------        ----        -------------------                                                *
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

#ifndef IMU_h
#define IMU_h

/* Include Global Parameters */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "MadgwickAHRS.h"
#include "Kalman.h"
#include "GPS_Library.h"


/* Declare Prototypes */

float  degtorad       (float );							
float  LowPassFilter  (float , float , float );
void   calculateYPR   (float *, float *);

/* the following are to use in this order */

Matrix*  calc_acc_vec        (Matrix *, const float , const float );
void     setKalman			 ();
void     compute_GPS		 (float [3], float [3], float [3]);
void     calculate_velocity  (float *, Matrix *);



#endif /* IMU_h */
