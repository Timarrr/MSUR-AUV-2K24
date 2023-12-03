/*
 * logger.h
 *
 *  Created on: Nov 27, 2023
 *      Author: mango
 */

#ifndef INC_LOGGER_H_
#define INC_LOGGER_H_

#include "stm32h7xx_hal.h"

#undef DEBUG

enum _Severity {
	TRACE,
	DEBUG,
	INFO,
	NOTICE,
	WARN,
	ERR,
};
typedef enum _Severity Severity;

void LOG_CDC_FS(Severity severity, const char *fmt, ...);

#endif /* INC_LOGGER_H_ */
