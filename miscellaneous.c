/*
 * miscellaneous.h
 *
 *  Author: Tomas Baca
 */

#include "miscellaneous.h"

/**
 * dynamically allocate the matrix using FreeRTOS pvPortMalloc
 */
matrix_float * matrix_float_alloc(const int16_t h, const int16_t w) {
	matrix_float * m;

	m->height = h;
	m->width = w;

	return m;
}

/**
 * dynamically allocate the matrix using FreeRTOS pvPortMalloc without a data
 */
matrix_float * matrix_float_alloc_hollow(const int16_t h, const int16_t w, float * data_pointer) {
	matrix_float * m;

	m->height = h;
	m->width = w;
	m->data = data_pointer;

	return m;
}

// vector allocation
vector_float * vector_float_alloc(const int16_t length, int8_t orientation) {
	vector_float * v;

	v->length = length;
	v->orientation = orientation;

	return v;
}

// vector allocation without a data
vector_float * vector_float_alloc_hollow(const int16_t length, int8_t orientation, float * data_pointer) {

	vector_float * v;

	v->length = length;
	v->orientation = orientation;
	v->data = data_pointer;

	return v;
}
