/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#ifndef MISCELLANEOUS_H_
#define MISCELLANEOUS_H_

#include "CMatrixLib.h"

// dynamically allocate the matrix using FreeRTOS pvPortMalloc
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w);

matrix_float * matrix_float_alloc_hollow(const int16_t h, const int16_t w, float * data_pointer);

// dynamically allocate the vector using FreeRTOS pvPortMalloc
vector_float * vector_float_alloc(const int16_t length, int8_t orientation);

vector_float * vector_float_alloc_hollow(const int16_t length, int8_t orientation, float * data_pointer);

#endif // MISCELLANEOUS_H_
