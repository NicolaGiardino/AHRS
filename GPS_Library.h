

#define _GPRMC_  1
#define _GPGGA_  2
#define _OTHER_  3

#include <stdint.h>
#include <stdlib.h>
#include <string.h>


int GPRMC_ok = 0, GPGGA_ok = 0;
uint8_t char_number = 0, SentenceType = 0, Term;
char sentence[6], rawTime[11], rawDate[7], rawSpeed[6], rawCourse[6], rawSatellites[3],
     rawLatitude[13], rawLongitude[13], rawAltitude[7], buffer[12];

void stringcpy(char *str1, char *str2, int dir) {
  uint8_t chr = 0;
  do {
    str2[chr + dir] = str1[chr];
  } while(str1[chr++] != '\0');
}


int GPSRead(uint8_t c) {//there goes the read from UART for GPS, ChibiOS function to be set

  switch(c) {
    case '\r':  // sentence end
      if(SentenceType == _GPRMC_)
        GPRMC_ok = 1;
      if(SentenceType == _GPGGA_)
        GPGGA_ok = 1;
      if(GPRMC_ok && GPGGA_ok) {
        GPRMC_ok = GPGGA_ok = 0;
        return 1;
      }
      break;

    case '$': // sentence start
      Term = char_number = 0;
      break;

    case ',':  // term end (new term start)
      buffer[char_number] = '\0';
      if(Term == 0) {
        stringcpy(buffer, sentence,0);
        if(strcmp(sentence, "GPRMC") == 0)
          SentenceType = _GPRMC_;
        else if(strcmp(sentence, "GPGGA") == 0)
               SentenceType = _GPGGA_;
             else
               SentenceType = _OTHER_;
      }

      // Time
      if(Term == 1 && SentenceType == _GPRMC_) {
        stringcpy(buffer, rawTime,0);
      }

      // Latitude
      if((Term == 3) && (SentenceType == _GPRMC_)) {
        stringcpy(buffer, rawLatitude, 1);
      }
      // Latitude N/S
      if((Term == 4) && (SentenceType == _GPRMC_)) {
        if(buffer[0] == 'N')
          rawLatitude[0] = '0';
        else
          rawLatitude[0] = '-';
      }

      // Longitude
      if((Term == 5) && (SentenceType == _GPRMC_)) {
        stringcpy(buffer, rawLongitude, 1);
      }
      // Longitude E/W
      if((Term == 6) && (SentenceType == _GPRMC_)) {
        if(buffer[0] == 'E')
          rawLongitude[0] = '0';
        else
          rawLongitude[0] = '-';
      }

      // Speed
      if((Term == 7) && (SentenceType == _GPRMC_)) {
        stringcpy(buffer, rawSpeed,0);
      }

      // Course
      if((Term == 8) && (SentenceType == _GPRMC_)) {
        stringcpy(buffer, rawCourse,0);
      }

      // Date
      if(Term == 9 && SentenceType == _GPRMC_) {
        stringcpy(buffer, rawDate,0);
      }

      // Satellites
      if((Term == 7) && (SentenceType == _GPGGA_)) {
        stringcpy(buffer, rawSatellites,0);
      }

      // Altitude
      if((Term == 9) && (SentenceType == _GPGGA_)) {
        stringcpy(buffer, rawAltitude,0);
      }
      Term++;
      char_number = 0;
      break;

    default:
      buffer[char_number++] = c;
      break;
  }

  return 0;
}

uint8_t GPSSecond() {
  return ((rawTime[4] - '0') * 10 + (rawTime[5] - '0'));
}
uint8_t GPSMinute() {
  return ((rawTime[2] - '0') * 10 + (rawTime[3] - '0'));
}
uint8_t GPSHour() {
  return ((rawTime[0] - '0') * 10 + (rawTime[1] - '0'));
}

uint8_t GPSDay() {
  return ((rawDate[0] - '0') * 10 + (rawDate[1] - '0'));
}
uint8_t GPSMonth() {
  return ((rawDate[2] - '0') * 10 + (rawDate[3] - '0'));
}
uint8_t GPSyear() {
  return ((rawDate[4] - '0') * 10 + (rawDate[5] - '0'));
}

float parse_rawDegree(char *term_) {
  float term_value = atof(term_)/100;
  int16_t term_dec = term_value;
  term_value -= term_dec;
  term_value  = term_value * 5/3 + term_dec;
  return term_value;
}

float Latitude() {
  return parse_rawDegree(rawLatitude);
}

float Longitude() {
  return parse_rawDegree(rawLongitude);
}

float Altitude() {
  return atof(rawAltitude);
}

uint8_t Satellites() {
  return atoi(rawSatellites);
}

float Speed() {
  return (atof(rawSpeed) * 1.852);
}

float Course() {
  return atof(rawCourse);
}

