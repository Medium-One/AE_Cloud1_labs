/*
 * helpers.c
 *
 *  Created on: Nov 27, 2017
 *      Author: demo
 */


#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "fx_stdio.h"
#include "helpers.h"

/*
 * copies a line from open file f into the buffer pointed to by out, up to
 * out_size bytes. if eof_ok is non-zero, we don't treat encountering EOF as an
 * error.
 */
int get_line(FILE * f, char * out, size_t out_size, uint8_t eof_ok) {
    char buf[200];

    if (fgets(buf, sizeof(buf), f) == NULL) {
        // EOF encountered, or error
        if (!eof_ok)
            return -1;
        if (!feof(f))  // not EOF, so an error
            return -2;
    }
    buf[strcspn(buf, "\r\n")] = 0;
    strncpy(out, buf, out_size);
    return 0;
}
