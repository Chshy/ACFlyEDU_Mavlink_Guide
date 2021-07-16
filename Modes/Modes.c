#include "Modes.h"

#include "STS.h"

//0-9为非飞行和校准模式
#include "M00_init.h"
#include "M01_Ground.h"

//10-19为校准模式
#include "M10_RCCalib.h"
#include "M12_AccCalib.h"
#include "M13_MagCalib.h"
#include "M15_HorizontalCalib.h"

//30-39为飞行模式
#include "M30_Att.h"
#include "M32_PosCtrl.h"
#include "M35_Auto1.h"
#include "M36_MAVLINK.h"

//20210713Y
#include "mavlink.h"

//简易环形队列
#define MSG_Q_SIZE 20//队列最大长度
static ModeMsg message_queue[MSG_Q_SIZE];
static uint8_t msg_q_head = 0;
static uint8_t msg_q_tail = 0;
static uint8_t msg_q_len = 0;//当前队列长度(元素数量)

bool SendMsgToMode( ModeMsg msg )//队尾入队
{
	if(msg_q_len >= MSG_Q_SIZE)//队列放满了
	{
		return false;
	}

	message_queue[msg_q_tail++] = msg;//放进去,tail++
	msg_q_tail %= MSG_Q_SIZE;//环形队列
	msg_q_len++;//计数+1

	return true;
}

bool ModeReceiveMsg( ModeMsg* msg )//队头出队
{
	if(msg_q_len <= 0)//队列是空的
	{
		return false;
	}

	*msg = message_queue[msg_q_head++];//取出来,head++
	msg_q_head %= MSG_Q_SIZE;//环形队列
	msg_q_len--;//计数-1
	return true;
}
//20210713Y



static unsigned int Mode_Task_ID;
static uint8_t current_mode = 0;
static Mode Modes[50] = {0};

static void Modes_Server( unsigned int Task_ID )
{
	Modes[ current_mode ].mode_main_func();
}

bool change_Mode( unsigned char mode )
{
	if( Modes[ mode ].mode_main_func == 0 )
		return false;
	Modes[ current_mode ].mode_exit();
	current_mode = mode;
	STS_Change_Task_Mode( Mode_Task_ID , STS_Task_Trigger_Mode_RoughTime , 1.0f / Modes[current_mode].mode_frequency , 0 );
	Modes[ current_mode ].mode_enter();
	return true;
}
unsigned char get_current_Mode()
{
	return current_mode;
}

void init_Modes()
{
	/*初始化模式*/
		//0-9为非飞行和校准模式
		Modes[0] = M00_init;
		Modes[1] = M01_Ground;
	
		//10-19为校准模式
		Modes[10] = M10_RCCalib;
		Modes[12] = M12_AccCalib;
		Modes[13] = M13_MagCalib;
		Modes[15] = M15_HorizontalCalib;

		//30-39为飞行模式
		Modes[30] = M30_Att;
		Modes[32] = M32_PosCtrl;
		Modes[35] = M35_Auto1;
		Modes[36] = M36_MAVLINK;
		
	/*初始化模式*/
	
	current_mode = 0;
	Mode_Task_ID = STS_Add_Task( STS_Task_Trigger_Mode_RoughTime , 1.0f / Modes[current_mode].mode_frequency , 0 , Modes_Server );
	Modes[ current_mode ].mode_enter();
}