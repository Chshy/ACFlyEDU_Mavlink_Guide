#include "AC_Math.h"
#include "Quaternion.h"
#include "MeasurementSystem.h"

#include "STS.h"
#include "drv_Uart2.h"

static bool SDI_RCTrigger( unsigned int Task_ID );
static void SDI_Server( unsigned int Task_ID );

void init_drv_SDI2()
{
	STS_Add_Task( STS_Task_Trigger_Mode_Custom , 0 , SDI_RCTrigger , SDI_Server );
}

static bool SDI_RCTrigger( unsigned int Task_ID )
{
	if( UART2_DataAvailable() )
		return true;
	return false;
}
int32_t SDI2_Point[32];
TIME SDI2_Time = {0};
static void SDI_Server( unsigned int Task_ID )
{
	//简单二次开发协议解析
	
	/*状态机变量*/
		static uint8_t rc_step1 = 0;	//0：接收包头'A' 'C'
																	//1：接收1字节消息类别
																	//2：接收1字节消息长度
																	//3：接收数据包内容
																	//4：接收2字节校验
		static uint8_t rc_step2 = 0;
	
		#define MAX_SDI_PACKET_SIZE 4*6
		static uint8_t msg_type;
		static uint8_t msg_length;
		ALIGN4 static uint8_t msg_pack[MAX_SDI_PACKET_SIZE];
		static uint8_t sumA;
		static uint8_t sumB;
		
		#define reset_SDI_RC ( rc_step1 = rc_step2 = 0 )
	/*状态机变量*/
	
	uint8_t rc_buf[20];
	uint8_t length = read_UART2( rc_buf , 20 );
	for( uint8_t i = 0 ; i < length ; ++i )
	{
		uint8_t r_data = rc_buf[i];
		
		switch( rc_step1 )
		{
			case 0 :
				//接收包头'A''C'
				if( rc_step2 == 0 )
				{
					if( r_data == 'A' )
						rc_step2 = 1;
				}
				else
				{
					if( r_data == 'C' )
					{
						rc_step1 = 1;
						rc_step2 = 0;
						sumA = sumB = 0;
					}
					else
						rc_step2 = 0;
				}
				break;
				
			case 1:
				//接收消息类别
				msg_type = r_data;
				sumB += r_data;
				sumA += sumB;
				rc_step1 = 2;
				rc_step2 = 0;
				break;
			
			case 2:
				//接收消息长度
				if( r_data > MAX_SDI_PACKET_SIZE )
				{
					reset_SDI_RC;
					break;
				}
				msg_length = r_data;
				sumB += r_data;
				sumA += sumB;
				if( msg_length == 0 )
					rc_step1 = 4;
				else
					rc_step1 = 3;
				rc_step2 = 0;
				break;
				
			case 3:
				//接收数据包
				msg_pack[ rc_step2 ] = r_data;
				sumB += r_data;
				sumA += sumB;
				++rc_step2;
				if( rc_step2 >= msg_length )
				{
					rc_step1 = 4;
					rc_step2 = 0;
				}
				break;
				
			case 4:
				//接收校验位
				if( rc_step2 == 0 )
				{
					if( sumA != r_data )
					{
						reset_SDI_RC;
						break;
					}
					rc_step2 = 1;
				}
				else
				{
					if( sumB == r_data )
					{
						SDI2_Point[0] = msg_type;
						if( msg_type==210 && msg_length==12 )
						{
							SDI2_Point[1] = *(int32_t*)&msg_pack[0];
							SDI2_Point[2] = *(int32_t*)&msg_pack[4];
							SDI2_Point[3] = *(int32_t*)&msg_pack[8];							
						}
						SDI2_Time = get_TIME_now();
					}
					reset_SDI_RC;
				}			
					
				break;
		}
	}
}
