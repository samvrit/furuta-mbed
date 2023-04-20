/*
 * matrix_definitions.h
 *
 *  Created on: 20-Apr-2023
 *      Author: Samvrit
 */

#ifndef CONFIG_SPECIFIC_SINGLE_PENDULUM_MATRIX_DEFINITIONS_H_
#define CONFIG_SPECIFIC_SINGLE_PENDULUM_MATRIX_DEFINITIONS_H_

static const float A[N_STATES][N_STATES] = {   {0.000000,  0.000000,   1.000000,   0.000000},
                                               {0.000000,  0.000000,   0.000000,   1.000000},
                                               {0.000000,  37.3894,    0.000000,   0.000000},
                                               {0.000000,  82.4377,    0.000000,   0.000000}};

static const float B[N_STATES][N_STATES] = {   {0.000000,  0.000000,   0.000000,   0.000000},
                                               {0.000000,  0.000000,   0.000000,   0.000000},
                                               {148.1531,  0.000000,   0.000000,   0.000000},
                                               {176.7290,  0.000000,   0.000000,   0.000000}};

static const float C[N_STATES][N_STATES] = {   {1.000000,  0.000000,   0.000000,   0.000000},
                                               {0.000000,  1.000000,   0.000000,   0.000000},
                                               {0.000000,  0.000000,   1.000000,   0.000000},
                                               {0.000000,  0.000000,   0.000000,   1.000000}};

static const float K[N_STATES][N_STATES] = {   {-0.3162,    9.5801,   -0.5308,    2.4884},
                                               {0.000000,  0.000000,   0.000000,   0.000000},
                                               {0.000000,  0.000000,   0.000000,   0.000000},
                                               {0.000000,  0.000000,   0.000000,   0.000000}};

static const float Q = 75e-6f;
static const float R = 1.21e-6f;

#endif /* CONFIG_SPECIFIC_SINGLE_PENDULUM_MATRIX_DEFINITIONS_H_ */
