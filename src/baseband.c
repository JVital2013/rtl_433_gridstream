/**
 * Baseband
 * 
 * Various functions for baseband sample processing
 *
 * Copyright (C) 2012 by Benjamin Larsson <benjamin@southpole.se>
 * Copyright (C) 2015 Tommy Vestermark
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "baseband.h"
#include <stdio.h>
#include <string.h>
#include <math.h>


static uint16_t scaled_squares[256];

/* precalculate lookup table for envelope detection */
static void calc_squares() {
    int i;
    for (i = 0; i < 256; i++)
        scaled_squares[i] = (127 - i) * (127 - i);
}

/** This will give a noisy envelope of OOK/ASK signals
 *  Subtract the bias (-128) and get an envelope estimation
 *  The output will be written in the input buffer
 *  @returns   pointer to the input buffer
 */
void envelope_detect(const uint8_t *iq_buf, uint16_t *y_buf, uint32_t len, int decimate) {
    unsigned int i;
    unsigned op = 0;
    unsigned int stride = 1 << decimate;

    for (i = 0; i < len; i += stride) {
        y_buf[op++] = scaled_squares[iq_buf[2 * i ]] + scaled_squares[iq_buf[2 * i + 1]];
    }
}


/** Something that might look like a IIR lowpass filter
 *
 *  [b,a] = butter(1, Wc) # low pass filter with cutoff pi*Wc radians
 *  Q1.15*Q15.0 = Q16.15
 *  Q16.15>>1 = Q15.14
 *  Q15.14 + Q15.14 + Q15.14 could possibly overflow to 17.14
 *  but the b coeffs are small so it wont happen
 *  Q15.14>>14 = Q15.0 \o/
 */
#define F_SCALE 15
#define S_CONST (1<<F_SCALE)
#define FIX(x) ((int)(x*S_CONST))

///  [b,a] = butter(1, 0.01) -> 3x tau (95%) ~100 samples
//static int a[FILTER_ORDER + 1] = {FIX(1.00000), FIX(0.96907)};
//static int b[FILTER_ORDER + 1] = {FIX(0.015466), FIX(0.015466)};
///  [b,a] = butter(1, 0.05) -> 3x tau (95%) ~20 samples
static int a[FILTER_ORDER + 1] = {FIX(1.00000), FIX(0.85408)};
static int b[FILTER_ORDER + 1] = {FIX(0.07296), FIX(0.07296)};


void baseband_low_pass_filter(const uint16_t *x_buf, int16_t *y_buf, uint32_t len, FilterState *state) {
    unsigned int i;

    /* Calculate first sample */
    y_buf[0] = ((a[1] * state->y[0] >> 1) + (b[0] * x_buf[0] >> 1) + (b[1] * state->x[0] >> 1)) >> (F_SCALE - 1);
    for (i = 1; i < len; i++) {
        y_buf[i] = ((a[1] * y_buf[i - 1] >> 1) + (b[0] * x_buf[i] >> 1) + (b[1] * x_buf[i - 1] >> 1)) >> (F_SCALE - 1);
    }

    /* Save last samples */
    memcpy(state->x, &x_buf[len - 1 - FILTER_ORDER], FILTER_ORDER * sizeof (int16_t));
    memcpy(state->y, &y_buf[len - 1 - FILTER_ORDER], FILTER_ORDER * sizeof (int16_t));
    //fprintf(stderr, "%d\n", y_buf[0]);
}

///  [b,a] = butter(1, 0.1) -> 3x tau (95%) ~10 samples
static int alp[2] = {FIX(1.00000), FIX(0.72654)};
static int blp[2] = {FIX(0.13673), FIX(0.13673)};

void baseband_demod_FM(const uint8_t *x_buf, int16_t *y_buf, unsigned num_samples, DemodFM_State *state) {
	int16_t ar, ai;		// New IQ sample: x[n]
	int16_t br, bi;		// Old IQ sample: x[n-1]
	int32_t pr, pi;		// Phase difference vector
	int16_t angle;		// Phase difference angle
	int16_t xdc, ydc, xdc_old, ydc_old;	// DC blocker variables
	int16_t xlp, ylp, xlp_old, ylp_old;	// Low Pass filter variables

	// Pre-feed old sample
	ar = state->br; ai = state->bi;

	for (unsigned n = 0; n < num_samples; n++) {
		// delay old sample 
		br = ar;
		bi = ai;
		// get new sample
		ar = x_buf[2*n]-128;
		ai = x_buf[2*n+1]-128;
		// Calculate phase difference vector: x[n] * conj(x[n-1])
		//pr = ar*br+ai*bi;	// May exactly overflow an int16_t (-128*-128 + -128*-128)
		pi = ai*br-ar*bi; 
		//angle = (int16_t)((atan2f(pi, pr) / M_PI) * INT16_MAX);	// Inefficient floating point for now...
		// DC blocker filter
		xdc = pi;	// We cheat for now and only use imaginary part (works well for small angles)
		ydc = xdc - xdc_old + ydc_old - ydc_old/256;
		ydc_old = ydc; xdc_old = xdc;
		// Low pass filter
		xlp = ydc;
		ylp = ((alp[1] * ylp_old >> 1) + (blp[0] * xlp >> 1) + (blp[1] * xlp_old >> 1)) >> (F_SCALE - 1);
		ylp_old = ylp; xlp_old = xlp;
		
		y_buf[n] = ylp;
	}

	// Store newest sample for next run
	state->br = ar; state->bi = ai;
}


void baseband_init(void) {
	calc_squares();
}


static FILE *dumpfile = NULL;

void baseband_dumpfile(const uint8_t *buf, uint32_t len) {
	if (dumpfile == NULL) {
		dumpfile = fopen("dumpfile.dat", "wb");
	}
	
	if (dumpfile == NULL) {
		fprintf(stderr, "Error: could not open dumpfile.dat\n");
	} else {
		fwrite(buf, 1, len, dumpfile);
		fflush(dumpfile);		// Flush as file is not closed cleanly...
	}
}
