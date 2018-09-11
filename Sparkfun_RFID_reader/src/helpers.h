/*
 * helpers.h
 *
 *  Created on: Nov 27, 2017
 *      Author: demo
 */

#ifndef HELPERS_H_
#define HELPERS_H_


#include "m1.h"
#include "app.h"


typedef struct {
    user_credentials_t device;
    user_credentials_t registration;
#ifdef USE_M1DIAG
    user_credentials_t diagnostics;
#endif
    project_credentials_t project;
} app_config_t;


int get_line(FILE * f, char * out, size_t out_size, uint8_t eof_ok);

#endif /* HELPERS_H_ */
