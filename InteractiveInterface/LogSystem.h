#pragma once

typedef enum
{
    LOG_DEBUG = 0,
    LOG_INFO = 1,
    LOG_WARNING = 2,
    LOG_ERROR = 3,
    LOG_FATAL = 4
} log_type_t;

bool lprintf(uint8_t log_type, const char *fmt, ...);
