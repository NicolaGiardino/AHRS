/* THIS IS NOT MY OWN LIBRARY, I JUST MODIFIED SOMETHING AND ADDED COMMENTS.
       THE ORIGINAL LIBRARY IS LINED IN THE .c FILE                       */

/***************************************************************************************************
*   FILENAME:  GPS_Lib.h                                                                           *
*                                                                                                  *
*                                                                                                  *
*   PURPOSE:   Library that defines all the function needed to compute GPS data                    *
*                                                                                                  *
*                                                                                                  *
*                                                                                                  *
*   GLOBAL VARIABLES:                                                                              *
*                                                                                                  *
*                                                                                                  *
*                                                                                                  *
*   DEVELOPMENT HISTORY :                                                                          *
*                                                                                                  *
*                                                                                                  *
*   Date          Modified by       Change Id     Release     Description Of Change                *
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

/* Include Global Parameters */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#define _GPRMC_  1
#define _GPGGA_  2
#define _OTHER_  3


int     GPRMC_ok     = 0;
int     GPGGA_ok     = 0;
uint8_t char_number  = 0;
uint8_t SentenceType = 0;
uint8_t Term;
char    sentence[6];
char    rawTime[11];
char    rawDate[7];
char    rawSpeed[6];
char    rawCourse[6];
char    rawSatellites[3];
char    rawLatitude[13];
char    rawLongitude[13];
char    rawAltitude[7];
char    buffer[12];

/* Declare Prototypes */

void    stringcpy        (char *str1, char *str2, int dir);
int     GPSRead          (uint8_t c);
uint8_t GPSSecond        (void);
uint8_t GPSMinute        (void);
uint8_t GPSHour          (void);
uint8_t GPSDay           (void);
uint8_t GPSMonth         (void);
uint8_t GPSyear          (void);
float   parse_rawDegree  (char *term_);
float   Latitude         (void);
float   Longitude        (void);
float   Altitude         (void);
uint8_t Satellites       (void);
float   Speed            (void);
float   Course           (void);
