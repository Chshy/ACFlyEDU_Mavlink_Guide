#include "MavlinkSendFuncs.h"
#include "mavlink.h"

#include "AC_Math.h"
#include "Configurations.h"
#include "Modes.h"
#include "MeasurementSystem.h"


//zzw

#include "Sensors.h"
#include "ControlSystem.h"

//zzw
static bool Msg01_SYS_STATUS( uint8_t port, mavlink_message_t* msg_sd )
{
    uint32_t onboard_control_sensors_present=0,onboard_control_sensors_health=0;
    onboard_control_sensors_present|=MAV_SYS_STATUS_SENSOR_3D_GYRO;
    onboard_control_sensors_present|=MAV_SYS_STATUS_SENSOR_3D_ACCEL;
    onboard_control_sensors_present|=MAV_SYS_STATUS_SENSOR_3D_MAG;
    onboard_control_sensors_health=onboard_control_sensors_present;
    if(GetPositionSensor(internal_baro_sensor_index)->present)onboard_control_sensors_present|=MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if(GetPositionSensor(default_optical_flow_index)->present)onboard_control_sensors_present|=	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    if(GetPositionSensor(default_gps_sensor_index)->present)onboard_control_sensors_present|=MAV_SYS_STATUS_SENSOR_GPS;
    if(GetPositionSensor(internal_baro_sensor_index)->available)onboard_control_sensors_health|=MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE;
    if(GetPositionSensor(default_optical_flow_index)->available)onboard_control_sensors_health|=	MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW;
    if(GetPositionSensor(default_gps_sensor_index)->available)onboard_control_sensors_health|=MAV_SYS_STATUS_SENSOR_GPS;
    mavlink_msg_sys_status_pack_chan(
        1,	//system id
        MAV_COMP_ID_AUTOPILOT1,	//component id
        port, 	//chan
        msg_sd,
        onboard_control_sensors_present,//onboard_control_sensors_present
        onboard_control_sensors_present,//onboard_control_sensors_enabled
        onboard_control_sensors_health,//onboard_control_sensors_health
        0,//load
        getBatteryVoltage()*1000,//voltage_battery mV
        -1,//current_battery
        -1,//battery_remaining
        0,//drop_rate_comm
        0,//errors_comm
        0,
        0,
        0,
        0);
    return true;
}

//zzw
static bool Msg26_SCALED_IMU( uint8_t port, mavlink_message_t* msg_sd )
{
    const IMU_Sensor* acc0 = GetAccelerometer( 0 );
    const IMU_Sensor* gyro0 = GetGyroscope( 0 );
    const IMU_Sensor* mag0 = GetMagnetometer( 0 );
    vector3_float acc_data_raw = acc0->data;
    vector3_float gyro_data_raw = gyro0->data;
    vector3_float mag_data_raw = mag0->data;
    acc_data_raw.x=acc_data_raw.x/0.980665;
    acc_data_raw.y=acc_data_raw.y/0.980665;
    acc_data_raw.z=acc_data_raw.z/0.980665;
    gyro_data_raw.x=gyro_data_raw.x*1000;
    gyro_data_raw.y=gyro_data_raw.y*1000;
    gyro_data_raw.z=gyro_data_raw.z*1000;
    mag_data_raw.x=mag_data_raw.x*1000;
    mag_data_raw.y=mag_data_raw.y*1000;
    mag_data_raw.z=mag_data_raw.z*1000;

    mavlink_msg_scaled_imu_pack_chan(
        1,	//system id
        MAV_COMP_ID_AUTOPILOT1,	//component id
        port, //chan
        msg_sd,	//msg
        get_System_Run_Time() * 1000, 	//boot ms
        acc_data_raw.x,	//xacc
        acc_data_raw.y,	//yacc
        acc_data_raw.z,	//zacc
        gyro_data_raw.x, //xgyro
        gyro_data_raw.y,	//ygyro
        gyro_data_raw.z,	//zgyro
        mag_data_raw.x,	//xmag
        mag_data_raw.y,	//ymag
        mag_data_raw.z);	//zmag
    return true;
}



















static bool Msg30_ATTITUDE( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_Measurement_System_Status() != Measurement_System_Status_Ready )
		return false;
	
	Quaternion airframe_quat = get_Airframe_attitude();
	vector3_float angular_rate = get_AngularRateCtrl();
	
	mavlink_msg_attitude_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		get_System_Run_Time() * 1000 , 	//boot ms
		Quaternion_getRoll( airframe_quat ) ,
		Quaternion_getPitch( airframe_quat ) ,
		Quaternion_getYaw( airframe_quat ) ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

static bool Msg31_ATTITUDE_QUATERNION( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Attitude_Measurement_System_Status() != Measurement_System_Status_Ready )
		return false;
	
	Quaternion airframe_quat = get_Airframe_attitude();
	vector3_float angular_rate = get_AngularRateCtrl();
	
	mavlink_msg_attitude_quaternion_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		get_System_Run_Time() * 1000 , 	//boot ms
		airframe_quat.qw ,
		airframe_quat.qx ,
		airframe_quat.qy ,
		airframe_quat.qz ,
		angular_rate.x , 
		angular_rate.y ,
		angular_rate.z );
	
	return true;
}

//旧32
/*
static bool Msg32_LOCAL_POSITION_NED( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_Measurement_System_Status() != Measurement_System_Status_Ready )
		return false;
	
	vector3_float Position = get_Position();
	vector3_float VelocityENU = get_VelocityENU();
	
	mavlink_msg_local_position_ned_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		get_System_Run_Time() * 1000 , 	//boot ms
		Position.y * 0.01f ,
		Position.x * 0.01f ,
		Position.z * -0.01f ,
		VelocityENU.y * 0.01f , 
		VelocityENU.x * 0.01f ,
		VelocityENU.z * -0.01f);
	return true;
}
*/

static bool Msg32_LOCAL_POSITION_NED( uint8_t port, mavlink_message_t* msg_sd )
{
    if( get_Altitude_Measurement_System_Status() != Measurement_System_Status_Ready )
        return false;

    
    // vector3_float Position;
    // vector3_float VelocityENU;
    // if(Position_Sensors[0].publishing == true)
    // {
    //     //如果T265连接则使用视觉数据
    //     get_Estimated_Sensor_Position_z( &Position.z , 0 );
	//     get_Estimated_Sensor_Position_xy( &Position.x , &Position.y , 0 );
    // }
    // else
    // {
    //     //如果T265未连接则使用整体解算数据
    //     Position = get_Position();
    // }
    // VelocityENU = get_VelocityENU();


    vector3_float Position = get_Position();
    vector3_float VelocityENU = get_VelocityENU();




    mavlink_msg_local_position_ned_pack_chan(
        1,	//system id
        MAV_COMP_ID_AUTOPILOT1,	//component id
        port, 	//chan
        msg_sd,
        get_System_Run_Time() * 1000, 	//boot ms
        Position.y * 0.01f,
        Position.x * 0.01f,
        Position.z * -0.01f,
        VelocityENU.y * 0.01f,
        VelocityENU.x * 0.01f,
        VelocityENU.z * -0.01f);
    
    
    // /*
	static char str[32];
	// sprintf(str,"Reg:%d",DEBUG_REGIST_SUCESS);
	// OLED_Draw_Str8x6(str,0,0);
    sprintf(str,"NED");
	OLED_Draw_Str8x6(str,0,64);
	sprintf(str,"N:%4.3lf",Position.y);
	OLED_Draw_Str8x6(str,1,64);
	sprintf(str,"E:%4.3lf",Position.x);
	OLED_Draw_Str8x6(str,2,64);
	sprintf(str,"D:%4.3lf",-Position.z);
	OLED_Draw_Str8x6(str,3,64);
    sprintf(str,"vN:%4.3lf",VelocityENU.y);
	OLED_Draw_Str8x6(str,4,64);
	sprintf(str,"vE:%4.3lf",VelocityENU.x);
	OLED_Draw_Str8x6(str,5,64);
	sprintf(str,"vD:%4.3lf",-VelocityENU.z);
	OLED_Draw_Str8x6(str,6,64);

//Ultrasonic
    if( GetPositionSensor(default_ultrasonic_sensor_index)->present == true )
    {
        float USData;
        get_Estimated_Sensor_Position_z(&USData,default_ultrasonic_sensor_index);

        static char str[32];
        sprintf(str,"US");
        OLED_Draw_Str8x6(str,5,0);
        sprintf(str,"%lf",USData);
        OLED_Draw_Str8x6(str,6,0);
        OLED_Update();
		
    }


    //sprintf(str,"atm:    ");
	//OLED_Draw_Str8x6(str,7,64);


    //  float atm;
    // get_Estimated_Sensor_Position_z(&atm,internal_baro_sensor_index);

    // // if(get_Estimated_Sensor_Position_z(&atm,internal_baro_sensor_index))
    // // {
    //     sprintf(str,"%lf",atm);
	//      OLED_Draw_Str8x6(str,7,50);
    // // }
    // else
    // {
    //     sprintf(str,"FAIL");
    //     OLED_Draw_Str8x6(str,7,88);
    // }




	OLED_Update();
    // */
    return true;
}







//20210712Y
/*
static bool Msg33_GLOBAL_POSITION_INT ( uint8_t port , mavlink_message_t* msg_sd )
{
	if( get_Altitude_Measurement_System_Status() != Measurement_System_Status_Ready )
		return false;
	
	double lat = 0, lon = 0, alt = 0;
	
	PosSensorHealthInf2 global_posInf;
	if( get_OptimalGlobal_XY(&global_posInf) )
	{		
		map_projection_reproject( &global_posInf.mp, 
				global_posInf.PositionENU.x+global_posInf.HOffset.x, 
				global_posInf.PositionENU.y+global_posInf.HOffset.y,
				&lat, &lon );
	}
	
	PosSensorHealthInf1 z_posInf;	
	if( get_OptimalGlobal_Z(&z_posInf) )
		alt = z_posInf.PositionENU.z + z_posInf.HOffset;
	
	static bool last_inFlight = false;
	static double homeZ = 0;
	bool inFlight;
	get_is_inFlight(&inFlight);
	if( inFlight != last_inFlight )
	{
		getHomeLocalZ( &homeZ, 0.01 );
		last_inFlight = inFlight;
	}
	vector3<double> position;
	get_Position_Ctrl(&position);
	double heightAboveGround = position.z - homeZ;
	
	vector3<double> vel;
	get_VelocityENU_Ctrl(&vel);	
		
	mavlink_msg_global_position_int_pack_chan( 
		1 ,	//system id
		MAV_COMP_ID_AUTOPILOT1 ,	//component id
		port , 	//chan
		msg_sd ,
		get_System_Run_Time() * 1000 , 	//boot ms
		lat*1e7  ,	//lat
		lon*1e7  ,	//lon
		alt*10 ,	//alt
		heightAboveGround*10 ,	//Altitude above ground
		vel.y ,	//vel north
		vel.x ,	//vel east
		-vel.z ,	//vel down
		0xffff	//Vehicle heading
	);		
	return true;
}
*/
//20210712Y



static bool Msg34_RC_CHANNELS_SCALED( uint8_t port, mavlink_message_t* msg_sd )
{
    const Receiver* Current_receiver=get_current_Receiver();
    mavlink_msg_rc_channels_scaled_pack_chan(
        1,	//system id
        MAV_COMP_ID_AUTOPILOT1,	//component id
        port, 	//chan
        msg_sd,
        get_System_Run_Time() * 1000, 	//boot ms
        0,//port
        Current_receiver->data[0],
        Current_receiver->data[1],
        Current_receiver->data[2],
        Current_receiver->data[3],
        Current_receiver->data[4],
        Current_receiver->data[5],
        -1,
        -1,
        255
    );
    return true;
}




//zzw
static bool Msg245_EXTENDED_SYS_STATE( uint8_t port, mavlink_message_t* msg_sd )
{
    uint8_t landed_state=0;
    if(get_is_inFlight())landed_state=MAV_LANDED_STATE_IN_AIR;
    else landed_state=	MAV_LANDED_STATE_ON_GROUND;
    mavlink_msg_extended_sys_state_pack_chan(
        1,	//system id
        MAV_COMP_ID_AUTOPILOT1,	//component id
        port, 	//chan
        msg_sd,
        MAV_VTOL_STATE_UNDEFINED,
        landed_state
    );
    return true;
}






bool (*const Mavlink_Send_Funcs[])( uint8_t port , mavlink_message_t* msg_sd ) = 
{
	/*000-*/	0	,
	/*001-*/	Msg01_SYS_STATUS	,//zzw
	/*002-*/	0	,
	/*003-*/	0	,
	/*004-*/	0	,
	/*005-*/	0	,
	/*006-*/	0	,
	/*007-*/	0	,
	/*008-*/	0	,
	/*009-*/	0	,
	/*010-*/	0	,
	/*011-*/	0	,
	/*012-*/	0	,
	/*013-*/	0	,
	/*014-*/	0	,
	/*015-*/	0	,
	/*016-*/	0	,
	/*017-*/	0	,
	/*018-*/	0	,
	/*019-*/	0	,
	/*020-*/	0	,
	/*021-*/	0	,
	/*022-*/	0	,
	/*023-*/	0	,
	/*024-*/	0	,
	/*025-*/	0	,
	/*026-*/	Msg26_SCALED_IMU	,//zzw
	/*027-*/	0	,
	/*028-*/	0	,
	/*029-*/	0	,
	/*030-*/	Msg30_ATTITUDE	,
	/*031-*/	Msg31_ATTITUDE_QUATERNION	,
	/*032-*/	Msg32_LOCAL_POSITION_NED	,
	/*033-*/	0	,//Msg33_GLOBAL_POSITION_INT	,//20210712Y
	/*034-*/	Msg34_RC_CHANNELS_SCALED	,//zzw
	/*035-*/	0	,
	/*036-*/	0	,
	/*037-*/	0	,
	/*038-*/	0	,
	/*039-*/	0	,
	/*040-*/	0	,
	/*041-*/	0	,
	/*042-*/	0	,
	/*043-*/	0	,
	/*044-*/	0	,
	/*045-*/	0	,
	/*046-*/	0	,
	/*047-*/	0	,
	/*048-*/	0	,
	/*049-*/	0	,
	/*050-*/	0	,
	/*051-*/	0	,
	/*052-*/	0	,
	/*053-*/	0	,
	/*054-*/	0	,
	/*055-*/	0	,
	/*056-*/	0	,
	/*057-*/	0	,
	/*058-*/	0	,
	/*059-*/	0	,
	/*060-*/	0	,
	/*061-*/	0	,
	/*062-*/	0	,
	/*063-*/	0	,
	/*064-*/	0	,
	/*065-*/	0	,
	/*066-*/	0	,
	/*067-*/	0	,
	/*068-*/	0	,
	/*069-*/	0	,
	/*070-*/	0	,
	/*071-*/	0	,
	/*072-*/	0	,
	/*073-*/	0	,
	/*074-*/	0	,
	/*075-*/	0	,
	/*076-*/	0	,
	/*077-*/	0	,
	/*078-*/	0	,
	/*079-*/	0	,
	/*080-*/	0	,
	/*081-*/	0	,
	/*082-*/	0	,
	/*083-*/	0	,
	/*084-*/	0	,
	/*085-*/	0	,
	/*086-*/	0	,
	/*087-*/	0	,
	/*088-*/	0	,
	/*089-*/	0	,
	/*090-*/	0	,
	/*091-*/	0	,
	/*092-*/	0	,
	/*093-*/	0	,
	/*094-*/	0	,
	/*095-*/	0	,
	/*096-*/	0	,
	/*097-*/	0	,
	/*098-*/	0	,
	/*099-*/	0	,
	/*100-*/	0	,
	/*101-*/	0	,
	/*102-*/	0	,
	/*103-*/	0	,
	/*104-*/	0	,
	/*105-*/	0	,
	/*106-*/	0	,
	/*107-*/	0	,
	/*108-*/	0	,
	/*109-*/	0	,
	/*110-*/	0	,
	/*111-*/	0	,
	/*112-*/	0	,
	/*113-*/	0	,
	/*114-*/	0	,
	/*115-*/	0	,
	/*116-*/	0	,
	/*117-*/	0	,
	/*118-*/	0	,
	/*119-*/	0	,
	/*120-*/	0	,
	/*121-*/	0	,
	/*122-*/	0	,
	/*123-*/	0	,
	/*124-*/	0	,
	/*125-*/	0	,
	/*126-*/	0	,
	/*127-*/	0	,
	/*128-*/	0	,
	/*129-*/	0	,
	/*130-*/	0	,
	/*131-*/	0	,
	/*132-*/	0	,
	/*133-*/	0	,
	/*134-*/	0	,
	/*135-*/	0	,
	/*136-*/	0	,
	/*137-*/	0	,
	/*138-*/	0	,
	/*139-*/	0	,
	/*140-*/	0	,
	/*141-*/	0	,
	/*142-*/	0	,
	/*143-*/	0	,
	/*144-*/	0	,
	/*145-*/	0	,
	/*146-*/	0	,
	/*147-*/	0	,
	/*148-*/	0	,
	/*149-*/	0	,
	/*150-*/	0	,
	/*151-*/	0	,
	/*152-*/	0	,
	/*153-*/	0	,
	/*154-*/	0	,
	/*155-*/	0	,
	/*156-*/	0	,
	/*157-*/	0	,
	/*158-*/	0	,
	/*159-*/	0	,
	/*160-*/	0	,
	/*161-*/	0	,
	/*162-*/	0	,
	/*163-*/	0	,
	/*164-*/	0	,
	/*165-*/	0	,
	/*166-*/	0	,
	/*167-*/	0	,
	/*168-*/	0	,
	/*169-*/	0	,
	/*170-*/	0	,
	/*171-*/	0	,
	/*172-*/	0	,
	/*173-*/	0	,
	/*174-*/	0	,
	/*175-*/	0	,
	/*176-*/	0	,
	/*177-*/	0	,
	/*178-*/	0	,
	/*179-*/	0	,
	/*180-*/	0	,
	/*181-*/	0	,
	/*182-*/	0	,
	/*183-*/	0	,
	/*184-*/	0	,
	/*185-*/	0	,
	/*186-*/	0	,
	/*187-*/	0	,
	/*188-*/	0	,
	/*189-*/	0	,
	/*190-*/	0	,
	/*191-*/	0	,
	/*192-*/	0	,
	/*193-*/	0	,
	/*194-*/	0	,
	/*195-*/	0	,
	/*196-*/	0	,
	/*197-*/	0	,
	/*198-*/	0	,
	/*199-*/	0	,
	/*200-*/	0	,
	/*201-*/	0	,
	/*202-*/	0	,
	/*203-*/	0	,
	/*204-*/	0	,
	/*205-*/	0	,
	/*206-*/	0	,
	/*207-*/	0	,
	/*208-*/	0	,
	/*209-*/	0	,
	/*210-*/	0	,
	/*211-*/	0	,
	/*212-*/	0	,
	/*213-*/	0	,
	/*214-*/	0	,
	/*215-*/	0	,
	/*216-*/	0	,
	/*217-*/	0	,
	/*218-*/	0	,
	/*219-*/	0	,
	/*220-*/	0	,
	/*221-*/	0	,
	/*222-*/	0	,
	/*223-*/	0	,
	/*224-*/	0	,
	/*225-*/	0	,
	/*226-*/	0	,
	/*227-*/	0	,
	/*228-*/	0	,
	/*229-*/	0	,
	/*230-*/	0	,
	/*231-*/	0	,
	/*232-*/	0	,
	/*233-*/	0	,
	/*234-*/	0	,
	/*235-*/	0	,
	/*236-*/	0	,
	/*237-*/	0	,
	/*238-*/	0	,
	/*239-*/	0	,
	/*240-*/	0	,
	/*241-*/	0	,
	/*242-*/	0	,
	/*243-*/	0	,
	/*244-*/	0	,
	/*245-*/	Msg245_EXTENDED_SYS_STATE	,//zzw
	/*246-*/	0	,
	/*247-*/	0	,
	/*248-*/	0	,
	/*249-*/	0	,
	/*250-*/	0	,
	/*251-*/	0	,
	/*252-*/	0	,
	/*253-*/	0	,
	/*254-*/	0	,
	/*255-*/	0	,
	/*256-*/	0	,
	/*257-*/	0	,
	/*258-*/	0	,
	/*259-*/	0	,
	/*260-*/	0	,
	/*261-*/	0	,
	/*262-*/	0	,
	/*263-*/	0	,
	/*264-*/	0	,
	/*265-*/	0	,
	/*266-*/	0	,
	/*267-*/	0	,
	/*268-*/	0	,
	/*269-*/	0	,
	/*270-*/	0	,
	/*271-*/	0	,
	/*272-*/	0	,
	/*273-*/	0	,
	/*274-*/	0	,
	/*275-*/	0	,
	/*276-*/	0	,
	/*277-*/	0	,
	/*278-*/	0	,
	/*279-*/	0	,
	/*280-*/	0	,
	/*281-*/	0	,
	/*282-*/	0	,
	/*283-*/	0	,
	/*284-*/	0	,
	/*285-*/	0	,
	/*286-*/	0	,
	/*287-*/	0	,
	/*288-*/	0	,
	/*289-*/	0	,
};
const uint16_t Mavlink_Send_Funcs_Count = sizeof( Mavlink_Send_Funcs ) / sizeof( void* );