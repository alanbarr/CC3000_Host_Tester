/*
 * The MIT License (MIT)
 * 
 * Copyright (c) 2012 Alan Barr
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

/* This header file is used to try and provide basic time configuration
 * for the launchpad. Define TIME430_CLOCK_FREQ in your project before including
 * this header file will use this preprocessor to give you the appropriate
 * DCO setup as well correct millisecond and microsecond delays. It may be 
 * more convenient to define  TIME430_CLOCK_FREQ in the makefile if there are 
 * multiple source files.
 *
 * The correct defines for TIME430_CLOCK_FREQ are:
 * 1,8,12 and 16. These correspond to the desired cpu speed in Mhz.
 */

#ifndef _TIME430_H_
#define _TIME430_H_

#ifdef TIME430_CLOCK_FREQ

    #if TIME430_CLOCK_FREQ == 1     \
    ||  TIME430_CLOCK_FREQ == 8     \
    ||  TIME430_CLOCK_FREQ == 12    \
    ||  TIME430_CLOCK_FREQ == 16 

        /*Setup Macros*/
        /*Intermediate step to ensure symbol is replaced*/
        #define CONVERT_FREQ_TO_STRING(x)   CREATE_DCO_STRING(x)
        /*Concatenates number into calibration define*/
        #define CREATE_DCO_STRING(x)        BCSCTL1 = CALBC1_ ## x ## MHZ;\
                                            DCOCTL = CALDCO_ ## x ## MHZ; 
        #define TIME430_US_CYCLES           (TIME430_CLOCK_FREQ)
        #define TIME430_MS_CYCLES           (TIME430_CLOCK_FREQ * 1000UL)
        #define TIME430_CLOCK_HZ            (TIME430_CLOCK_FREQ * 1000000UL)

        /* Use the below to configure the DCO*/
        #define TIME430_CALIBRATE_CLOCK() \
                                    CONVERT_FREQ_TO_STRING(TIME430_CLOCK_FREQ)

        /* The below macros will give you the correct delay depending on the 
         * configured clock. The largest value which __delay_cycles can take
         * (mspgcc ver. 20110612) is 106609*/
        #define TIME430_DELAY_US(x)     __delay_cycles((x) * TIME430_US_CYCLES)
        #define TIME430_DELAY_MS(x)     __delay_cycles((x) * TIME430_MS_CYCLES)

    #else
        #error TIME430_CLOCK_FREQ was not 1,2,8 or 16.\
                Please ensure it is defined correctly.
    #endif 

#else
    #error "TIME430_CLOCK_FREQ has not been defined."
#endif  /*#ifdef TIME430_CLOCK_FREQ*/


#endif /*_TIME430_H_*/
