/*
* File:   rnc.c
* Author: Erik Friesen
* MIT license
* Permission is hereby granted, free of charge, to any person obtaining a
* copy of this software and associated documentation files (the "Software"),
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included
* in all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*
*/

#include <xc.h>
#include <string.h>
#include "rnd.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "logs.h"

SemaphoreHandle_t ulRandMutex = NULL;
static int RandomInited = 0;

void InitializeTRNG(void) {
    ulRandMutex = xSemaphoreCreateMutex();
    configASSERT(ulRandMutex != NULL);
    RNGCONbits.TRNGMODE = 1;
    RNGCONbits.TRNGEN = 1;
}

unsigned long ulRand(void) {
    static int err = 0;
    unsigned long rv = 0;
    if (!RandomInited) {
	InitializeTRNG();
	RandomInited = 1;
    }
    (void) xSemaphoreTake(ulRandMutex, portMAX_DELAY);
    while (RNGCNT < 4);
    rv = RNGSEED1;
    (void) xSemaphoreGive(ulRandMutex);
    if (!rv && err < 3) {
	err++;
	SysLog(LOG_NORMAL, "TRNG: Failed, 0\r\n");
    }
    return rv;
}

int trng_read(unsigned char * mem, int len) {
    int rl = len;
    if (!RandomInited) {
	InitializeTRNG();
	RandomInited = 1;
    }
    (void) xSemaphoreTake(ulRandMutex, portMAX_DELAY);
    while (len) {
	while (RNGCNT < 64);
	if (len > 8) {
	    memcpy(mem, (unsigned char*) &RNGSEED1, 8);
	    mem += 8;
	    len -= 8;
	} else {
	    memcpy(mem, (unsigned char*) &RNGSEED1, len);
	    len = 0;
	}
    }
    (void) xSemaphoreGive(ulRandMutex);
    return rl;
}