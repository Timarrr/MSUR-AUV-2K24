/*
 * logger.c
 *
 *  Created on: Nov 27, 2023
 *      Author: mango
 */
#include "logger.h"
#include "config.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

void LOG_CDC_FS(Severity severity, const char *fmt, ...){
	va_list args;
    va_start(args, fmt);
//	if(severity < CONF_GLOBAL_SEVERITY)
//	    	return;
	printf("[%f]:", ((float)HAL_GetTick())/1000);
    switch(severity){
	case TRACE:
		printf(" trace: ");
		break;
	case DEBUG:
		printf("\033[37m debug: ");
		break;
	case INFO:
		printf("\033[36m info: ");
		break;
	case NOTICE:
		printf("\033[36m notice: ");
		break;
	case WARN:
		printf("\033[33m warn: ");
		break;
	case ERR:
		printf("\033[31;1m err: ");
		break;
	}

    vprintf(fmt, args);
    printf("\033[0m\r\n");
    va_end(args);
}


