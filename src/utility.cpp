#include <Arduino.h>
#include "utility.h"

int clamp(int n, int minn, int maxn) {				// https://stackoverflow.com/questions/5996881/how-to-limit-a-number-to-be-within-a-specified-range-python
	return max(min(maxn, n), minn);
}

bool split(char* input, char** next, char delim) {	// https://github.com/bdring/PendantsForFluidNC/blob/main/lib/GrblParserC/src/GrblParserC.c
    char* pos = strchr(input, delim);
    if (pos) {
        *pos  = '\0';
        *next = pos + 1;
        return true;
    }
    *next = input + strlen(input);  
    return false;
}

void parse_integers(char* s, int* nums, uint8_t maxnums) {	// modified it so it actually returns bytes lmao
    char*  next;
    size_t i = 0;
    do {
        if (i >= maxnums) {
            return;
        }
        split(s, &next, ',');
        nums[i++] = atoi(s);

        s = next;
    } while (*s);
}