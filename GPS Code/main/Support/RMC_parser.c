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
#include <time.h>
#include "string.h"
#include <stdio.h>


typedef struct
{
    char hours;
    char minutes;
    char seconds;
    char validity;
    char latDeg;
    float latMin;
    char latDir;
    char longDeg;
    float longMin;
    char longDir;
    float groundSpeed;
    float groundDirection;
    int year;
    char month;
    char day;
    float magVariation;
    char varDir;
}RMC_Struct;

typedef struct
{
    char* UTC_Time[10];
    char* validity;
    char* latitude[6];
    char* latDir;
    char* longitude[7];
    char* longDir;
    char* groundSpeed[5];
    char* groundDirection[6];
    char* date[6];
    char* magVariation[10];
    char* varDir;
}RMC_Handle;

RMC_Struct data;
RMC_Handle sortedString;

void* RMC_Parse_Time()
{
    char timeString = sortedString.UTC_Time;
    char hours;
    char minutes;
    char seconds;
    sscanf(timeString, "%2d%2d%2d", &hours, &minutes, &seconds);
    data.hours = hours;
    data.minutes = minutes;
    data.seconds = seconds;
}

void* RMC_Parse_Validity()
{
    if(sortedString.validity = 'A')
    {
        data.validity = 1;
    }
    else
    {
        data.validity = 0;
    }
}

void* RMC_Parse_Lat()
{
    char deg, minutes;
    char latString = sortedString.latitude;
    scanf(latString,'%2d%6f', &deg, &minutes);

    data.latDeg = deg;
    data.latMin = minutes;
}

void* RMC_Parse_LatDir()
{
    if(sortedString.latDir = 'N')
    {
        data.latDir = 1;
    }
    else if (sortedString.latDir = 'S')
    {
        data.latDir = 0;
    }
}

void* RMC_Parse_Long()
{
    char deg, minutes;
    char longString = sortedString.longitude;
    scanf(longString,'%3d%5f', &deg, &minutes);

    data.longDeg = deg;
    data.longMin = minutes;
}

void* RMC_Parse_LongDir()
{
    if(sortedString.longDir = 'E')
    {
        data.longDir = 1;
    }
    else if (sortedString.longDir = 'W')
    {
        data.longDir = 0;
    }
}

void* RMC_Parse_GroundSpeed()
{
    float speed;
    scanf(sortedString.groundSpeed,'%f', &speed);
    data.groundSpeed = speed;
}

void RMC_Parse_GroundDir()
{
    float deg;
    scanf(sortedString.groundDirection,'%f', &deg);
    data.groundDirection = deg;
}

void* RMC_Parse_Date()
{
    int year;
    char month;
    char day;

    char dateString = sortedString.date;
    scanf('%2d%2d%2d', &day, &month, &year);

    data.year = year;
    data.month = month;
    data.day = day;
}

void* RMC_Parse_MagneticVar()
{
    float mag;
    scanf(sortedString.magVariation,'%f', &mag);
    data.magVariation = mag;
}

void RMC_Parse_VarDir()
{
    char dir = sortedString.varDir;
    data.varDir = dir;
}

void tokenToStruct(char* tokenArray, RMC_Handle* sortedString)
{
    strcpy(tokenArray[1], (sortedString->UTC_Time));
    strcpy(tokenArray[2], (sortedString->validity));
    strcpy(tokenArray[3], (sortedString->latitude));
    strcpy(tokenArray[4], (sortedString->latDir));
    strcpy(tokenArray[5], (sortedString->longitude));
    strcpy(tokenArray[6], (sortedString->longDir));
    strcpy(tokenArray[7], (sortedString->groundSpeed));
    strcpy(tokenArray[8], (sortedString->groundDirection));
    strcpy(tokenArray[9], (sortedString->date));
    strcpy(tokenArray[10], (sortedString->magVariation));
    strcpy(tokenArray[11], (sortedString->varDir));
}

char* RMC_Token(char nmeaSentence[])
{
    //char nmeaSentence[120] = "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10";
    char *tokenArray[13] = {NULL};
    int i = 0;

    for (char *nmeaTokens = strtok(nmeaSentence, ","); nmeaTokens != NULL; nmeaTokens = strtok(NULL, ",")) {
        tokenArray[i] = nmeaTokens; // Store the token itself
        printf("The resultant token is: %s\n", tokenArray[i]); // Print the token
        i++;
    }
    tokenToStruct((char*) tokenArray, &sortedString);
}

RMC_Struct parseRMC(char *nmeaSentence[])
{

    RMC_Token((char *)nmeaSentence);

    RMC_Parse_Time;
    RMC_Parse_Validity;
    RMC_Parse_Lat;
    RMC_Parse_LatDir;
    RMC_Parse_Long;
    RMC_Parse_LongDir;
    RMC_Parse_GroundSpeed;
    RMC_Parse_GroundDir;
    RMC_Parse_Date;
    //RMC_Parse_MagneticVar;
    //RMC_Parse_VarDir;

    return data;
}
