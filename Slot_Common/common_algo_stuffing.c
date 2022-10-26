/* Copyright 2011, Jacques Fortier. All rights reserved.
 *
 * Redistribution and use in source and binary forms are permitted, with or without modification.
 */
//#include <stdint.h>
//#include <stddef.h>
#include "common_algo_stuffing.h"
/* Stuffs "length" bytes of data at the location pointed to by
 * "input", writing the output to the location pointed to by
 * "output". Returns the number of bytes written to "output".
 *
 * Remove the "restrict" qualifiers if compiling with a
 * pre-C99 C dialect.
 */
size_t cobs_encode(const uint8_t * restrict input, size_t length, uint8_t * restrict output)
{
    size_t read_index = 0;
    size_t write_index = 1;
    size_t code_index = 0;
    uint8_t code = 1;

    while(read_index < length)
    {
        if(input[read_index] == 0)
        {
            output[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        }
        else
        {
            output[write_index++] = input[read_index++];
            code++;
            if(code == 0xFF)
            {
                output[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }

    output[code_index] = code;

    return write_index;
}

/* Unstuffs "length" bytes of data at the location pointed to by
 * "input", writing the output * to the location pointed to by
 * "output". Returns the number of bytes written to "output" if
 * "input" was successfully unstuffed, and 0 if there was an
 * error unstuffing "input".
 *
 * Remove the "restrict" qualifiers if compiling with a
 * pre-C99 C dialect.
 */
size_t cobs_decode(const uint8_t * restrict input, size_t length, uint8_t * restrict output)
{
    size_t read_index = 0;
    size_t write_index = 0;
    uint8_t code;
    uint8_t i;

    while(read_index < length)
    {
        code = input[read_index];

        if(read_index + code > length && code != 1)
        {
            return 0;
        }

        read_index++;

        for(i = 1; i < code; i++)
        {
            output[write_index++] = input[read_index++];
        }
        if(code != 0xFF && read_index != length)
        {
            output[write_index++] = '\0';
        }
    }

    return write_index;
}

/*
 * encode data in base 252 and framed with
 * startofframe, endofframe
 */
const uint8_t startofframe = 0x01; // [SOH]
const uint8_t endofframe = 0x00; // [NUL]
const uint8_t escape = 0x02; // [STX]

size_t base252_encode(const uint8_t * restrict input, size_t length, uint8_t * restrict output)
{
    int32_t outCnt = 1;
    output[0] = startofframe;

    for (uint32_t cnt = 0; cnt < length; cnt++)
    {
        if (input[cnt] < 3)
        {
            output[outCnt] = escape;
            outCnt++;
        }

        output[outCnt] = input[cnt];
        outCnt++;
    }

    output[outCnt] = endofframe;
    outCnt++;

    return outCnt;
}
