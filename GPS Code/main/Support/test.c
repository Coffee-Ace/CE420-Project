#include "string.h"
#include "stddef.h"
#include "stdio.h"

void main(void) {
    char nmeaSentence[120] = "$GPRMC,161229.487,A,3723.2475,N,12158.3416,W,0.13,309.62,120598, ,*10";
    char *tokenArray[13] = {NULL};
    int i = 0;

    for (char *nmeaTokens = strtok(nmeaSentence, ","); nmeaTokens != NULL; nmeaTokens = strtok(NULL, ",")) {
        tokenArray[i] = nmeaTokens; // Store the token itself
        printf("The resultant token is: %s\n", tokenArray[i]); // Print the token
        i++;
    }
}