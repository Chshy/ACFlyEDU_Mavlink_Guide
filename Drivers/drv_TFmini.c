#include "Basic.h"
#include "drv_Uart2.h"

#include "Quaternion.h"
#include "MeasurementSystem.h"
#include "drv_SDI.h"
#include "Commulink.h"
#include "STS.h"
#include "Sensors_Backend.h"
#include "RingBuf.h"

#include "TM4C123GH6PM.h"
#include "uart.h"
#include "sysctl.h"
#include "gpio.h"
#include "pin_map.h"
#include "interrupt.h"
#include "hw_ints.h"
#include "hw_gpio.h"
#include "Timer.h"
#include "udma.h"

#include "InteractiveInterface.h"
#include "stdio.h"
char strtf[18];

typedef struct
{
	uint8_t Rsv1; //预留
	uint8_t ID;
	uint32_t System_time;
	//int 24:
	uint8_t dis_x1k[3]; //Dist距离 1000倍
	uint8_t dis_status;
	uint16_t Strength; //信号强度,暂时设定为>0有效
	uint8_t Rsv2;	   //预留

	float Dist; //解析完成后的距离值

} __PACKED _TfMini;
static const unsigned char packet_ID[2] = {0x57, 0x00};

/*接收缓冲区*/
#define RX_BUFFER_SIZE 24
static uint8_t R7_rx_buffer[RX_BUFFER_SIZE];
static uint8_t R7_Rx_RingBuf;
static float HIGH_EIGHT;
static float LOW_EIGHT;
static float ks109_HIGH;
static void UART7_Handler(void);
static void UART7_receive(void);
static void ks109_send(unsigned int Task_ID);

void init_drv_TFmini(void)
{

	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART7);
	GPIOPinConfigure(GPIO_PE0_U7RX);
	GPIOPinConfigure(GPIO_PE1_U7TX);

	GPIOPinTypeUART(GPIOE_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTConfigSetExpClk(UART7_BASE, SysCtlClockGet(), 921600,
						(UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
	UARTFIFOEnable(UART7_BASE);

	//配置串口接收中断
	UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT);
	UARTIntRegister(UART7_BASE, UART7_Handler);

	UARTFIFOEnable(UART7_BASE);
	UARTIntEnable(UART7_BASE, UART_INT_RX | UART_INT_RT); //使能UART0发送接收中断
	UARTIntRegister(UART7_BASE, UART7_Handler);			  //UART中断地址注册
	IntPrioritySet(INT_UART7, INT_PRIO_7);
	//STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f/10 , 0 , send );
	PositionSensorRegister(default_laser_sensor_index,
						   Position_Sensor_Type_RangePositioning,
						   Position_Sensor_DataType_s_z,
						   Position_Sensor_frame_ENU,
						   0.05f,
						   false);
}

uint16_t hign_z;
//static uint8_t hbyte[9];
_TfMini SensorD;

static void UART7_Handler()
{

	static int i = 0;
	UARTIntClear(UART7_BASE, UART_INT_OE | UART_INT_RT);
	UARTRxErrorClear(UART7_BASE);

	while ((UART7->FR & (1 << 4)) == false)
	{
		//接收
		uint8_t rdata = UART7->DR & 0xff;

		static unsigned char rc_counter = 0;
		static unsigned char receive = 0;

		static uint16_t check_sum;
		static signed char sum = 0;
		/************************/
		if (rc_counter == 0)
			sum = 0;
		if (rc_counter < 2)
		{
			//接收包头
			if (rdata != packet_ID[rc_counter])
				rc_counter = 0;
			else
			{
				sum += rdata;
				++rc_counter;
			}
		}
		else if (rc_counter < 15) //接收[Rsv1-Rsv2]数据
		{						  //接收数据
			switch (rc_counter)
			{
			case 2:
				SensorD.Rsv1 = rdata;
				break;
			case 3:
				SensorD.ID = rdata;
				break;
			case 4:
			case 5:
			case 6:
			case 7:
				((unsigned char *)&SensorD.System_time)[rc_counter - 4] = rdata;
				break;
			case 8:
			case 9:
			case 10:
				((unsigned char *)SensorD.dis_x1k)[rc_counter - 8] = rdata;
				break;
			case 11:
				SensorD.dis_status = rdata;
				break;
			case 12:
			case 13:
				((unsigned char *)&SensorD.Strength)[rc_counter - 12] = rdata;
				break;
			case 14:
				SensorD.Rsv1 = rdata;
				break;
			default:
				rc_counter = 0;
				break;
			}
			sum += (unsigned char)rdata;
			++rc_counter;
		}
		else
		{ //校验
			if (sum == rdata)
			{ //校验成功

				//开始计算距离值
				int32_t temp = (int32_t)(SensorD.dis_x1k[0] << 8 | SensorD.dis_x1k[1] << 16 | SensorD.dis_x1k[2] << 24) / 256;
				SensorD.Dist = temp / 1000.0f * 100; //注意单位
				//结束计算距离值

				if (SensorD.Strength > 0 && SensorD.Dist > 2 && SensorD.Dist < 500) //2cm~5m
				{
					vector3_float position;
					position.z = SensorD.Dist;
					//角度补偿
					float lean_cosin = get_lean_angle_cosin();
					position.z *= lean_cosin;
					hign_z = position.z;

					sprintf(strtf, "tfz=%lf", position.z);
					OLED_Draw_Str8x6(strtf, 2, 5);
					OLED_Update();

					PositionSensorUpdatePosition(default_ultrasonic_sensor_index, position, true, -1);
				}
				else
					PositionSensorSetInavailable(default_ultrasonic_sensor_index);
			}
			rc_counter = 0;
		}

		/************************/
	}
}

uint16_t get_position_z()
{
	return hign_z;
}
