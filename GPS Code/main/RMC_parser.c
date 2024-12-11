/*
    Author: Dan Abidov
    Class: Microcomputer Systems
    Professor: Girma Tewolde
    Project: Final Project
    
    Purpose: Receive an RMC NMEA sentence, Parse in order to receive data, and return data as a formatted struct

    Example RMC sentence:
        $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10

    Message ID:             $GPRMC                  RMC protocol header
    UTC Time:               161229.487              hhmmss.sss
    Status:                 A                       A = data is valid, V = invalid
    Latitude:               3723.2475               ddmm.mmmm
    N/S Indicator:          N                       N or S
    Longitude:              12158.3416              dddmm.mmmm
    E/W Indicator:          W                       E or W
    Speed Over Ground:      0.13        Knots       
    Course Over Ground:     309.62      deg         True
    Date:                   120598                  ddmmyy
    Magnetic Variation:     
    Variation Dir:          E
    Mode (Only NMEA v2.3):  A
    Checksum:               *10
    <cr><lf>

    Source :https://www.sparkfun.com/datasheets/GPS/NMEA%20Reference%20Manual-Rev2.1-Dec07.pdf
*/
#include <string.h>
#include <stdio.h>
#include "RMC_parser.h"

// Function to tokenize an NMEA sentence and parse the RMC data
RMC_Struct parseRMC(char *nmeaSentence) {
    RMC_Struct RMCdata = {0};  // Initialize struct with zero values
    char *tokenArray[13] = {NULL};
    int i = 0;

    // Tokenize the NMEA sentence
    for (char *token = strtok((char *)nmeaSentence, ","); token != NULL; token = strtok(NULL, ",")) {
        if (i < 13) {
            tokenArray[i++] = token;
        }
    }

    // Parse each field
    sscanf(tokenArray[1], "%2hhd%2hhd%2hhd", &RMCdata.hours, &RMCdata.minutes, &RMCdata.seconds);
    RMCdata.validity = (tokenArray[2][0] == 'A') ? 1 : 0;
    
    sscanf(tokenArray[3], "%2hhd%f", &RMCdata.latDeg, &RMCdata.latMin);
    RMCdata.latDir = (tokenArray[4][0] == 'N') ? 1 : 0;

    sscanf(tokenArray[5], "%3hhd%f", &RMCdata.longDeg, &RMCdata.longMin);
    RMCdata.longDir = (tokenArray[6][0] == 'E') ? 1 : 0;

    sscanf(tokenArray[7], "%f", &RMCdata.groundSpeed);
    sscanf(tokenArray[8], "%f", &RMCdata.groundDirection);

    sscanf(tokenArray[9], "%2hhd%2hhd%2d", &RMCdata.day, &RMCdata.month, &RMCdata.year);

    if (tokenArray[10] && strlen(tokenArray[10]) > 0) {
        sscanf(tokenArray[10], "%f", &RMCdata.magVariation);
    }
    if (tokenArray[11] && strlen(tokenArray[11]) > 0) {
        RMCdata.varDir = tokenArray[11][0];
    }

    return RMCdata;
}

