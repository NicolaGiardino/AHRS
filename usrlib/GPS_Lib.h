#include <stdint.h>
#include <stdlib.h>
#include <string.h>


#define _GPRMC_  1
#define _GPGGA_  2
#define _OTHER_  3


int GPRMC_ok = 0, GPGGA_ok = 0;
uint8_t char_number = 0, SentenceType = 0, Term;
char sentence[6], rawTime[11], rawDate[7], rawSpeed[6], rawCourse[6], rawSatellites[3],
     rawLatitude[13], rawLongitude[13], rawAltitude[7], buffer[12];

void stringcpy(char *str1, char *str2, int dir);
int GPSRead(uint8_t c);
uint8_t GPSSecond(void);
uint8_t GPSMinute(void);
uint8_t GPSHour(void);
uint8_t GPSDay(void);
uint8_t GPSMonth(void);
uint8_t GPSyear(void);
float parse_rawDegree(char *term_);
float Latitude(void);
float Longitude(void);
float Altitude(void);
uint8_t Satellites(void);
float Speed(void);
float Course(void);
