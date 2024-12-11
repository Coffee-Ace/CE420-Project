/*
    Author: Dan Abidov
    Project: CE 420 Final Project
    
    Purpose: This header file contains a struct 
    defining the data contained within an RMC string
    as well as a function which converts a an RMC sentence into
    the RMC struct
*/
#include <stdint.h>

/**
 * @brief This struct is typdef for the information which can be extracted 
 * from an RMC sentence. 
 * 
 * 
 * @var char hours              from 0-23
 * @var char minutes            from 0-59
 * @var char seconds            from 0-59
 * @var char validity           1 is valid, 0 is invalid
 * @var char latDeg             Latitude Degrees
 * @var float latMin            Latitude Minutes
 * @var char latDir             Latitude Direction. 1 is North, 0 is South
 * @var char longDeg            Longitude Degrees. 
 * @var float longMin           Longitude Minutes
 * @var char longDir            Longitutde Direction. 1 is West, 0 is East
 * @var float groundSpeed       Speed over ground in knots
 * @var float groundDirection   Direction over ground in degrees
 * @var int year                Year
 * @var char month              Month
 * @var char day                Day
 * @var float magVariation      Magnetic Variation
 * @var char varDir             Variation Direction
 * 
 * @author Dan Abidov, Senior 1 at Kettering University
 */
typedef struct {
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
} RMC_Struct;


/**
 *
 * @brief This function parses an RMC sentence 
 * 
 * Example RMC sentence:
 *     $GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10
 * 
 * @param nmeaSentence A pointer to the address of the RMC sentence
 * 
 * @returns RMC_Struct containing any data parsed from the RMC sentence
 * 
 * @author Dan Abidov, Senior 1 at Kettering University
 */
RMC_Struct parseRMC(char *nmeaSentence[]);
