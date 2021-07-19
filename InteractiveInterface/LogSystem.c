#include <stdio.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>


#include "Basic.h"
#include "LogSystem.h"
#include "drv_Uart2.h"

#define LOGSYSTEM_BUFSIZE 128

#define PRINT_IN_COLOR

static char logsystem_buf[LOGSYSTEM_BUFSIZE];

bool lprintf( uint8_t log_type , const char* fmt , ...)
{
    TIME time = get_TIME_now();
    sprintf(logsystem_buf,"[%lf]",time.t/1E8);
    Uart2_Send( logsystem_buf , strlen(logsystem_buf) );
    //echo -e "\e[1;31m This is red text! \e[0m"
    //处理颜色
    switch(log_type)
    {
        case LOG_DEBUG:
        {
            sprintf(logsystem_buf,"\e[1;37m [DEBUG] ");
            break;
        }
        case LOG_INFO:
        {
            sprintf(logsystem_buf,"\e[1;32m [ INFO] ");
            break;
        }
        case LOG_WARNING:
        {
            sprintf(logsystem_buf,"\e[1;33m [ WARN] ");
            break;
        }
        case LOG_ERROR:
        {
            sprintf(logsystem_buf,"\e[1;31m [ERROR] ");
            break;
        }
        case LOG_FATAL:
        {
            sprintf(logsystem_buf,"\e[1;35m [FATAL] ");
            break;
        }
        default:
        {
            sprintf(logsystem_buf,"switch error\r\n");
            Uart2_Send( logsystem_buf , strlen(logsystem_buf) );
            return false;
        }
        // Uart2_Send( logsystem_buf , strlen(logsystem_buf) );
        // Uart2_Send( logsystem_buf , 16 );
    }
    Uart2_Send( logsystem_buf , strlen(logsystem_buf) );


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



    sprintf(logsystem_buf," \e[0m \r\n");
    Uart2_Send( logsystem_buf , strlen(logsystem_buf) );
    return true;
}

