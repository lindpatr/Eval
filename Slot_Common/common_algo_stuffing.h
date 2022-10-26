/*
 * common_algo_stuffing.h
 *
 *  Created on: 20 oct. 2022
 *      Author: BEL-LinPat
 */


/* Copyright 2011, Jacques Fortier. All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted, with or without modification.
 */
#ifndef COMMON_ALGO_STUFFING_H_
#define COMMON_ALGO_STUFFING_H_

#include <stdint.h>
#include <stddef.h>

size_t cobs_encode(const uint8_t * restrict input, size_t length, uint8_t * restrict output);
size_t cobs_decode(const uint8_t * restrict input, size_t length, uint8_t * restrict output);


size_t base252_encode(const uint8_t * restrict input, size_t length, uint8_t * restrict output);
#endif /*COMMON_ALGO_STUFFING_H_*/
