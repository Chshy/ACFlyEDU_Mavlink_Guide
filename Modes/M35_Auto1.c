#include "Modes.h"
#include "Basic.h"
#include "stdlib.h"
#include <stdio.h>
#include "M35_Auto1.h"

#include "AC_Math.h"
#include "Receiver.h"
#include "InteractiveInterface.h"
#include "ControlSystem.h"
#include "MeasurementSystem.h"
#include "drv_Uart2.h"

//20210714Y
#include "NavCmdProcess.h"
//20210714Y

//20210712Y
#include "CommuLink.h"
#include "mavlink.h"
static uint16_t mav_mode;
//20210712Y
NavCmdInf navInf;

static void M35_Auto1_MainFunc();
static void M35_Auto1_enter();
static void M35_Auto1_exit();
const Mode M35_Auto1 = 
{
	50 , //mode frequency
	M35_Auto1_enter , //enter
	M35_Auto1_exit ,	//exit
	M35_Auto1_MainFunc ,	//mode main func
};

typedef struct
{
	//退出模式计数器
	uint16_t exit_mode_counter;
	
	//自动飞行状态机
	uint8_t auto_step1;	//0-记录按钮位置
											//1-等待按钮按下起飞 
											//2-等待起飞完成 
											//3-等待2秒
											//4-降落
											//5-等待降落完成
	uint16_t auto_counter;
	float last_button_value;
	
	float last_height;
}MODE_INF;
static MODE_INF* Mode_Inf;

static void M35_Auto1_enter()
{
	Led_setStatus( LED_status_running1 );
	
	//初始化模式变量
	Mode_Inf = malloc( sizeof( MODE_INF ) );
	Mode_Inf->exit_mode_counter = 0;
	Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
	Altitude_Control_Enable();

	//20210714Y
	//任务状态机
	init_NavCmdInf(&navInf);
	//20210714Y
	
}

static void M35_Auto1_exit()
{
	Altitude_Control_Disable();
	Attitude_Control_Disable();
	
	free( Mode_Inf );
}

static void M35_Auto1_MainFunc()
{
	

	const Receiver* rc = get_current_Receiver();
		
	if( rc->available == false )
	{
		//接收机不可用
		//降落

		//20210712Y
		//取消GUIDED模式
		get_mav_mode(&mav_mode);
		set_mav_mode(mav_mode & ~MAV_MODE_FLAG_GUIDED_ENABLED);
		//20210712Y

		Position_Control_set_XYLock();
		Position_Control_set_TargetVelocityZ( -50 );
		return;
	}

	//读遥控器1-4通道
	float throttle_stick = rc->data[0];
	float yaw_stick = rc->data[1];
	float pitch_stick = rc->data[2];
	float roll_stick = rc->data[3];	
	
	/*判断退出模式*/
	//外八
		if( throttle_stick < 5 && yaw_stick < 5 && pitch_stick < 5 && roll_stick > 95 )
		{
			//持续一定时间
			if( ++Mode_Inf->exit_mode_counter >= 50 )
			{
				//20210712Y
				get_mav_mode(&mav_mode);
				set_mav_mode(mav_mode & ~MAV_MODE_FLAG_GUIDED_ENABLED);
				//20210712Y
				change_Mode( 1 );
				return;
			}
		}
		else
			Mode_Inf->exit_mode_counter = 0;
	/*判断退出模式*/
		
	//判断摇杆是否在中间
	bool sticks_in_neutral = 
		in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( pitch_stick , 5 , 50 ) && \
		in_symmetry_range_offset_float( roll_stick , 5 , 50 );
	
	/*
	extern float SDI_Point[6];
	extern TIME SDI_Time;
	extern int32_t SDI2_Point[20];
	*/

	
	/*DE 打印四元数Yaw
		char str[32];
		sprintf(str,"%lf",Quaternion_getYaw( get_Airframe_attitude() ));
		OLED_Draw_Str8x6(str,4,50);
		OLED_Update();
	//DE*/
	

	//摇杆在中间 并且 Measurement 准备完成
	// if( sticks_in_neutral && get_Position_Measurement_System_Status() == Measurement_System_Status_Ready )
	if( sticks_in_neutral ) //DEBUG
	{


		//20210712Y
		//开启Guided模式
		get_mav_mode(&mav_mode);
		set_mav_mode(mav_mode | MAV_MODE_FLAG_GUIDED_ENABLED);
		//20210712Y



//20210714Y

		int msg_res = 0;
		uint8_t msg_res_MAVRES;

		/*
		if(navInf.counter1 == 0 && navInf.counter2 == 0)//如果任务状态机为初始状态(没有任务在执行)
		{
			//获取消息
			bool msg_available;
			ModeMsg msg;
			msg_available = ModeReceiveMsg( &msg );
			
			
			
			//MAV_CMD消息处理

			
			if( msg_available )
			{	
				if(check_NavCmd(msg.cmd,50,msg.frame,msg.params))
				{
					msg_res = Process_NavCmd(msg.cmd,50,msg.frame,msg.params,navInf);
					switch(msg_res)
					{
						case -100://FAIL
						{
							msg_res_MAVRES = MAV_RESULT_FAILED;
							break;
						}
						case -3://IN PROGRESS
						case -2://IN PROGRESS
						{
							msg_res_MAVRES = MAV_RESULT_IN_PROGRESS;
							break;
						}
						case -1://SUCESS
						{
							msg_res_MAVRES = MAV_RESULT_ACCEPTED;
							break;
						}
						default:
						{
							msg_res_MAVRES = MAV_RESULT_UNSUPPORTED;
							break;
						}
					}
					Send_NavCmd_ACK(&msg,msg_res_MAVRES);
				}				
			}
		}
		else//有任务在执行中
		{
			msg_res = Process_NavCmd(msg.cmd,50,msg.frame,msg.params,navInf);
			switch(msg_res)
			{
				case -100://FAIL
				{
					msg_res_MAVRES = MAV_RESULT_FAILED;
					break;
				}
				case -3://IN PROGRESS
				case -2://IN PROGRESS
				{
					msg_res_MAVRES = MAV_RESULT_IN_PROGRESS;
					break;
				}
				case -1://SUCESS
				{
					msg_res_MAVRES = MAV_RESULT_ACCEPTED;
					break;
				}
				default:
				{
					msg_res_MAVRES = MAV_RESULT_UNSUPPORTED;
					break;
				}
			}
			Send_NavCmd_ACK(&msg,msg_res_MAVRES);
		}
		*/















//20210714Y


/*
		//20210713Y
		//获取消息
		bool msg_available;
		ModeMsg msg;
		msg_available = ModeReceiveMsg( &msg );
		bool msg_handled = false;
		
		//MAV_CMD消息处理
		//if( msg_available && msg.cmd==MAV_CMD_COMPONENT_ARM_DISARM )
		if( msg_available )
		{	
			
			/-*
			switch( msg.cmd )
			case MAV_CMD_COMPONENT_ARM_DISARM:
				Led_setStatus( LED_status_running2 );
				char str[32];
				sprintf(str,"%d",msg.cmd);
				OLED_Draw_Str8x6(str,0,50);
				OLED_Update();
			break;
			*-/

			// default:
			// break;
			
			
			if(check_NavCmd(msg.cmd,50,msg.frame,msg.params))
			{
				Process_NavCmd(msg.cmd,50,msg.frame,msg.params,)
			}

			msg_handled = 1;
		}
		//20210713Y
*/



		//摇杆在中间
		//执行自动飞行		
		//只有在位置有效时才执行自动飞行
		
		//打开水平位置控制
		#define write_new_byte(buffer,i,byte) buffer[i++]=byte;CK_A+=byte; CK_B+=CK_A
		Position_Control_Enable();
		//extern TIME SDI2_Time;


		/*
		switch( Mode_Inf->auto_step1 )
		{
			case 0:
				Mode_Inf->last_button_value = rc->data[5];
				++Mode_Inf->auto_step1;
				Mode_Inf->auto_counter = 0;
				break;
			
			case 1:
				//等待按钮按下起飞
				if( get_is_inFlight() == false && fabsf( rc->data[5] - Mode_Inf->last_button_value ) > 15 )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				else
					goto ManualControl;
				break;
				
			case 2:	
			{				
				uint8_t buf[30];
				unsigned char CK_A = 0 , CK_B = 0;
				buf[0] = 'A';	buf[1] = 'C';
				int buffer_index = 2;
				write_new_byte( buf, buffer_index, 201 );
				write_new_byte( buf, buffer_index, 0 );
				buf[ buffer_index++ ] = CK_A;
				buf[ buffer_index++ ] = CK_B;
				Uart2_Send( buf, buffer_index );
				if( get_pass_time(SDI2_Time)<0.5 && SDI2_Point[0]==210 )
				{
					Position_Control_Takeoff_HeightRelative( 300.0f );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
			}
			case 3:
				//等待起飞完成飞直线
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					Position_Control_set_TargetPositionZRelative( 1700 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 4:
				//等待起飞完成飞直线
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					uint8_t buf[30];
					unsigned char CK_A = 0 , CK_B = 0;
					buf[0] = 'A';	buf[1] = 'C';
					int buffer_index = 2;
					write_new_byte( buf, buffer_index, 202 );
					write_new_byte( buf, buffer_index, 0 );
					buf[ buffer_index++ ] = CK_A;
					buf[ buffer_index++ ] = CK_B;
					Uart2_Send( buf, buffer_index );
					
					Position_Control_set_TargetPositionXY_LatLon( SDI2_Point[2]*1e-7, SDI2_Point[1]*1e-7 );
					//Position_Control_set_TargetPositionXY_LatLon( 23.154063 , 113.028636 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 5:
				//等待直线飞行完成下降
				if( get_Position_ControlMode() == Position_ControlMode_Position )
				{
					uint8_t buf[30];
					unsigned char CK_A = 0 , CK_B = 0;
					buf[0] = 'A';	buf[1] = 'C';
					int buffer_index = 2;
					write_new_byte( buf, buffer_index, 203 );
					write_new_byte( buf, buffer_index, 0 );
					buf[ buffer_index++ ] = CK_A;
					buf[ buffer_index++ ] = CK_B;
					Uart2_Send( buf, buffer_index );
					
					Position_Control_set_TargetPositionZRelative( -1500 );
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
					Mode_Inf->last_height = 500;
					OLED_Draw_TickCross8x6( false , 0 , 0 );
				}
				break;
				
			case 6:
				//等待降高度完成
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					uint8_t buf[30];
					unsigned char CK_A = 0 , CK_B = 0;
					buf[0] = 'A';	buf[1] = 'C';
					int buffer_index = 2;
					write_new_byte( buf, buffer_index, 204 );
					write_new_byte( buf, buffer_index, 0 );
					buf[ buffer_index++ ] = CK_A;
					buf[ buffer_index++ ] = CK_B;
					Uart2_Send( buf, buffer_index );
					
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 7:
				if( Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 1.0f )
				{
					Position_Control_set_TargetPositionXYRelativeBodyHeading( SDI_Point[0] , SDI_Point[1] );
					Position_Control_set_TargetPositionZRelative( -(SDI_Point[2] - 300) );
					
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
			
			case 8:
				if( get_Position_ControlMode() == Position_ControlMode_Position && get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 9:
				if( Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 1.0f )
				{
					Attitude_Control_set_Target_YawRelative( SDI_Point[3] + PI_f/2 );
					
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
			
			case 10:
				if( ++Mode_Inf->auto_counter == 200 )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 11:			
				OLED_Draw_Str8x6( "     " , 2 , 0 );
				OLED_Draw_Str8x6( "     " , 3 , 0 );
				if( Time_isValid(SDI_Time) && get_pass_time(SDI_Time) < 1.0f )
				{
					OLED_Draw_TickCross8x6( true , 0 , 0 );
					char str[5];
					sprintf( str , "%5.1f", SDI_Point[0] );
					OLED_Draw_Str8x6( str , 2 , 0 );
					sprintf( str , "%5.1f" , SDI_Point[1] );
					OLED_Draw_Str8x6( str , 3 , 0 );
					Position_Control_set_TargetVelocityBodyHeadingXY_AngleLimit( \
						constrain_float( SDI_Point[0] * 1.2f , 50 ) ,	\
						constrain_float( SDI_Point[1] * 1.2f , 50 ) ,	\
						0.15f ,	\
						0.15f	\
					);
					Position_Control_set_TargetVelocityZ(	\
						constrain_range_float( -SDI_Point[2] * 0.1f , -20 , -50 )	\
					);
					//Attitude_Control_set_Target_YawRate( constrain_float( SDI_Point[3] * 0.2f , 0.6f ) );
					Mode_Inf->last_height = SDI_Point[2];
				}
				else
				{
					//目标不可用，刹车锁位置
					Position_Control_set_XYLock();
					if( Mode_Inf->last_height > 100 )
						Position_Control_set_ZLock();
					else
						Position_Control_set_TargetVelocityZ( -30 );
				//	Attitude_Control_set_YawLock();
				}
				OLED_Update();
				
				//等待按钮按下
				if( get_is_inFlight() == false )
				{	
					Position_Control_set_XYLock();
					Mode_Inf->auto_step1 = 12;
					Mode_Inf->auto_counter = 0;
				}
				break;
				
			case 12:
			{
				uint8_t buf[30];
				unsigned char CK_A = 0 , CK_B = 0;
				buf[0] = 'A';	buf[1] = 'C';
				int buffer_index = 2;
				write_new_byte( buf, buffer_index, 205 );
				write_new_byte( buf, buffer_index, 0 );
				buf[ buffer_index++ ] = CK_A;
				buf[ buffer_index++ ] = CK_B;
				Uart2_Send( buf, buffer_index );
				if( ++Mode_Inf->auto_counter >= 1000 )
				{
					Mode_Inf->auto_step1 = 0;
					Mode_Inf->auto_counter = 0;
				}
				else if( get_pass_time(SDI2_Time)<0.5 && SDI2_Point[0]==211 )
				{
					Mode_Inf->auto_step1 = 13;
					Mode_Inf->auto_counter = 0;
				}
				break;
			}		
			
			case 13:
			{
				uint8_t buf[30];
				unsigned char CK_A = 0 , CK_B = 0;
				buf[0] = 'A';	buf[1] = 'C';
				int buffer_index = 2;
				write_new_byte( buf, buffer_index, 206 );
				write_new_byte( buf, buffer_index, 0 );
				buf[ buffer_index++ ] = CK_A;
				buf[ buffer_index++ ] = CK_B;
				Uart2_Send( buf, buffer_index );
				if( get_pass_time(SDI2_Time)<0.5 && SDI2_Point[0]==210 )
				{
					++Mode_Inf->auto_step1;
					Mode_Inf->auto_counter = 0;
				}
				break;
			}

			case 14:
			{
				Position_Control_Takeoff_HeightRelative( 300.0f );
				++Mode_Inf->auto_step1;
				Mode_Inf->auto_counter = 0;
			}
			
			case 15:
			{
				//等待起飞完成飞直线
				if( get_Altitude_ControlMode() == Position_ControlMode_Position )
				{
					Mode_Inf->auto_step1 = 9;
					Mode_Inf->auto_counter = 0;
				}
				break;
			}
		}
		*/
			
	}
	else
	{
		ManualControl:
		//摇杆不在中间
		//手动控制

		//20210712Y
		//关闭Guided模式
		get_mav_mode(&mav_mode);
		set_mav_mode(mav_mode & ~MAV_MODE_FLAG_GUIDED_ENABLED);
		//20210712Y
		
		//20210714Y
		init_NavCmdInf(&navInf);
		//20210714Y


		Mode_Inf->auto_step1 = Mode_Inf->auto_counter = 0;
		
		char str[16];
		double Lat , Lon;
		get_LatLon_From_Point( get_Position().x , get_Position().y , &Lat , &Lon );
		// sprintf( str , "%d", (int)(Lat*1e7) );
		// OLED_Draw_Str8x6( str , 0 , 0 );
		// sprintf( str , "%d", (int)(Lon*1e7) );
		// OLED_Draw_Str8x6( str , 1 , 0 );
		// OLED_Update();
		
		//关闭水平位置控制
		Position_Control_Disable();
		
		//高度控制输入
		if( in_symmetry_range_offset_float( throttle_stick , 5 , 50 ) )
			Position_Control_set_ZLock();
		else
			Position_Control_set_TargetVelocityZ( ( throttle_stick - 50.0f ) * 6 );

		//偏航控制输入
		if( in_symmetry_range_offset_float( yaw_stick , 5 , 50 ) )
			Attitude_Control_set_YawLock();
		else
			Attitude_Control_set_Target_YawRate( ( 50.0f - yaw_stick )*0.05f );
		
		//Roll Pitch控制输入
		Attitude_Control_set_Target_RollPitch( \
			( roll_stick 	- 50.0f )*0.015f, \
			( pitch_stick - 50.0f )*0.015f );
	}
}