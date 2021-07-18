#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>



#include "LogSystem.h"

#define LOGSYSTEM_BUFSIZE 128

#define PRINT_IN_COLOR

static char logsystem_buf[LOGSYSTEM_BUFSIZE];

bool lprintf(uint8_t log_type,const char* fmt, ...)
{
    //echo -e "\e[1;31m This is red text! \e[0m"
    //处理颜色
    switch(log_type)
    {
        case LOG_DEBUG:
        {
            
        }
        case LOG_INFO:
        {
            
        }
        case LOG_WARNING:
        {
            
        }
        case LOG_ERROR:
        {
            
        }
        case LOG_FATAL:
        {
            
        }
        default:
        {
            return false;
        }
    }

    va_list args;
    int n;

    va_start(args, fmt);
    n = vsnprintf(logsystem_buf, LOGSYSTEM_BUFSIZE, fmt, args);
    va_end(args);

    Uart2_Send( logsystem_buf , n );

    // int i = 0;
    // for(i = 0; i < n; i++)
    // {
    // //    HAL_UART_Transmit(&huart2, (uint8_t *)&myprintf_buf[i], 1, 0xFFFF); //根据不同的平台，修改串口输出的函数
    // }

    return;
}

