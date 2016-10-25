/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			RobotDog.c
** 最后修订日期:  	2009-10-20
** 最后版本:			1.0
** 描述:				机器狗服务程序
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-10-20
** 版本:				1.0
** 描述:				机器狗服务程序
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
*************************************************************************************************************************/
#include "Apps/RobotDog.h"





/*************************************************************************************************************************
**	初始化舵机位置基点表
*************************************************************************************************************************/
const uint16	SERVO_BASIC_POSITION[32]	= {513, 452, 503, 512, 525, 498, 518, 532,
											   488, 522, 522, 521, 522, 489, 515, 516,
											   525, 532, 516};

/*************************************************************************************************************************
**	初始化舵机位置矢量方向
*************************************************************************************************************************/
const uint8		SERVO_OFFSET_DIRECTION[32] 	= {TRUE, TRUE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE,
											   FALSE, FALSE, TRUE, TRUE, TRUE, TRUE, FALSE, FALSE,
											   FALSE, FALSE, TRUE};


/*************************************************************************************************************************
** 函数名称:			ConversionRelPosition
**
** 函数描述:			把舵机相对位置转换为实际位置
**
**
** 输入变量:			uint8 id, int16 rel_position, uint16* position;
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 ConversionRelPosition(uint8 id, int16 rel_position, uint16* position)
{
	uint8 temp_state = 0;
	uint8 temp_value = 0;

	int32 temp_position = 0;

	temp_state = mapping_control.pInverse1DimArray(ttlbus_control.p_device_mapping, ttlbus_control.device_amount, id, &temp_value);

	if (TRUE == temp_state)
	{
		if (FALSE == SERVO_OFFSET_DIRECTION[temp_value])
		{
			temp_position = (int32)(SERVO_BASIC_POSITION[temp_value] - rel_position);
		}
		else
		{
			temp_position = (int32)(SERVO_BASIC_POSITION[temp_value] + rel_position);
		}

		if (temp_position > 1023)
		{
			*position = 1023;
		}
		else if (temp_position < 0)
		{
			*position = 0;
		}
		else
		{
			*position = (uint16)temp_position;
		}

		return TRUE;
	}

	return FALSE;
}


/*************************************************************************************************************************
** 函数名称:			ControlSynchronous4Dof
**
** 函数描述:			4自由度同步位置控制
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ControlSynchronous4Dof(const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time)
{
	uint8 count = 0;
	uint8 temp_id = 0;

	uint16 temp_position = 0;
	uint16 temp_velocity[4] = {0, 0, 0, 0};
	uint16 temp_time[4] = {0, 0, 0, 0};
	uint16 temp_max = 0;

	temp_id = first_id;

	for (count = 0; count < 4; count++)
	{
		ConversionRelPosition((temp_id + count), *(vector + count), &temp_position);

		servo_control.pCalcVelocity(first_id + count, temp_position, ref_time, &temp_velocity[count], &temp_time[count]);
	}

	for (count = 0; count < 4; count++)
	{
		if (temp_max < temp_time[count])
		{
			temp_max = temp_time[count];
		}
	}

	*calc_time = temp_max;

	for (count = 0; count < 4; count++)
	{
		ConversionRelPosition((temp_id + count), *(vector + count), &temp_position);

		servo_control.pCalcVelocity(first_id + count, temp_position, temp_max, &temp_velocity[count], &temp_time[count]);

		if (0 != temp_velocity[count])
		{
			servo_control.pSetSyncPosition(first_id + count, temp_position, temp_velocity[count]);
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			ControlSynchronous1Dof
**
** 函数描述:			1自由度同步位置控制
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ControlSynchronous1Dof(const int16 vector, uint8 id, uint16 ref_time, uint16* calc_time)
{
	uint16 temp_position = 0;
	uint16 temp_velocity = 0;
	uint16 temp_time = 0;

	ConversionRelPosition(id, vector, &temp_position);

	servo_control.pCalcVelocity(id, temp_position, ref_time, &temp_velocity, &temp_time);

	if (0 != temp_velocity)
	{
		servo_control.pSetSyncPosition(id, temp_position, temp_velocity);
	}

	*calc_time = temp_time;
}


/*************************************************************************************************************************
** 函数名称:			SlowlyWalk
**
** 函数描述:			4自由度同步位置控制
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-20
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void SlowlyWalk(void)
{
	const int16 l_foreleg_vector_a[4] = {0, -40, 170, 150};
	const int16 l_foreleg_vector_b[4] = {90, -50, 230, 145};
	const int16 l_foreleg_vector_c[4] = {160, 50, 220, 80};
	const int16 l_foreleg_vector_d[4] = {190, 30, 200, 10};
	const int16 l_foreleg_vector_e[4] = {290, 30, 400, 120};
	const int16 l_foreleg_vector_f[4] = {45, 20, 340, 200};

	const int16 r_foreleg_vector_a[4] = {0, -40, 180, 150};
	const int16 r_foreleg_vector_b[4] = {90, -50, 230, 145};
	const int16 r_foreleg_vector_c[4] = {160, 50, 220, 65};
	const int16 r_foreleg_vector_d[4] = {190, 30, 200, 10};
	const int16 r_foreleg_vector_e[4] = {290, 30, 400, 120};
	const int16 r_foreleg_vector_f[4] = {45, 20, 370, 200};

	const int16 l_hindleg_vector_a[4] = {0, 40, 147, 155};
	const int16 l_hindleg_vector_b[4] = {90, -30, 165, 105};
	const int16 l_hindleg_vector_c[4] = {150, -35, 190, 30};
	const int16 l_hindleg_vector_d[4] = {190, 50, 180, 15};
	const int16 l_hindleg_vector_e[4] = {270, 0, 400, 120};
	const int16 l_hindleg_vector_f[4] = {115, 0, 340, 240};

	const int16 r_hindleg_vector_a[4] = {0, 40, 137, 165};
	const int16 r_hindleg_vector_b[4] = {90, -30, 220, 150};
	const int16 r_hindleg_vector_c[4] = {150, -35, 200, 70};
	const int16 r_hindleg_vector_d[4] = {170, 50, 200, 15};
	const int16 r_hindleg_vector_e[4] = {270, 0, 400, 120};
	const int16 r_hindleg_vector_f[4] = {115, 0, 340, 240};



	uint16 temp_time = 0;
	uint8 temp_value = 0;

	const uint8 slowly_walk_mapping[4] = {1, 3, 0, 2};

	signal_control.pGetByte(ADDR_MOVELEG, &temp_value);

	switch (slowly_walk_mapping[temp_value])
	{
		case 0:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_d[0], 11, 500, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_b[0], 21, 500, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_e[0], 31, 200, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_c[0], 41, 500, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(200);

			ControlSynchronous4Dof(&l_hindleg_vector_f[0], 31, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_hindleg_vector_a[0], 31, 200, &temp_time);
			servo_control.pActionSync();

			_delay_ms(200);

			signal_control.pSetByte(ADDR_MOVELEG, 0);

			break;
		}

		case 1:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_e[0], 11, 200, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_c[0], 21, 500, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_b[0], 31, 500, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_d[0], 41, 500, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(200);

			ControlSynchronous4Dof(&l_foreleg_vector_f[0], 11, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_foreleg_vector_a[0], 11, 200, &temp_time);
			servo_control.pActionSync();

			_delay_ms(200);

			signal_control.pSetByte(ADDR_MOVELEG, 3);

			break;
		}

		case 2:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_b[0], 11, 500, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_d[0], 21, 500, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_c[0], 31, 500, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_e[0], 41, 200, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(200);

			ControlSynchronous4Dof(&r_hindleg_vector_f[0], 41, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_hindleg_vector_a[0], 41, 200, &temp_time);
			servo_control.pActionSync();

			_delay_ms(200);

			signal_control.pSetByte(ADDR_MOVELEG, 1);

			break;
		}

		case 3:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_c[0], 11, 500, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_e[0], 21, 200, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_d[0], 31, 500, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_b[0], 41, 500, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(200);

			ControlSynchronous4Dof(&r_foreleg_vector_f[0], 21, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_foreleg_vector_a[0], 21, 200, &temp_time);
			servo_control.pActionSync();

			_delay_ms(200);

			signal_control.pSetByte(ADDR_MOVELEG, 2);

			break;
		}

		default:
		{
			break;
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			TurnLeft
**
** 函数描述:			转弯控制
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void TurnLeft(void)
{
	const int16 r_foreleg_vector_a[4] = {115, 60, 180, 60};
	const int16 r_foreleg_vector_b[4] = {90, 15, 140, 70};
	const int16 r_foreleg_vector_c[4] = {90, -32, 155, 60};
	const int16 r_foreleg_vector_d[4] = {110, -80, 175, 35};
	const int16 r_foreleg_vector_e[4] = {210, -40, 360, 100};
	const int16 r_foreleg_vector_f[4] = {250, 30, 380, 190};

	const int16 l_hindleg_vector_a[4] = {120, 75, 190, 70};
	const int16 l_hindleg_vector_b[4] = {100, 35, 190, 75 };
	const int16 l_hindleg_vector_c[4] = {125, -40, 190, 65};
	const int16 l_hindleg_vector_d[4] = {140, -90, 170, 40};
	const int16 l_hindleg_vector_e[4] = {215, -65, 280, 90};
	const int16 l_hindleg_vector_f[4] = {280, 40, 305, 140};

	const int16 l_foreleg_vector_a[4] = {115, 70, 190, 70};
	const int16 l_foreleg_vector_b[4] = {100, 15, 180, 120};
	const int16 l_foreleg_vector_c[4] = {90, -45, 110, 60};
	const int16 l_foreleg_vector_d[4] = {110, -60, 160, 85};
	const int16 l_foreleg_vector_e[4] = {270, 20, 390, 200};
	const int16 l_foreleg_vector_f[4] = {190, 55, 300, 100};

	const int16 r_hindleg_vector_a[4] = {110, 65, 175, 40};
	const int16 r_hindleg_vector_b[4] = {105, 25, 165, 60};
	const int16 r_hindleg_vector_c[4] = {130, -45, 120, 20};
	const int16 r_hindleg_vector_d[4] = {140, -60, 160, 50};
	const int16 r_hindleg_vector_e[4] = {275, -30, 380, 200};
	const int16 r_hindleg_vector_f[4] = {200, 60, 270, 100};


	uint16 temp_time = 0;
	uint8 temp_value = 0;

	const uint8 turn_left_mapping[4] = {3, 2, 0, 1};

	signal_control.pGetByte(ADDR_MOVELEG, &temp_value);

	switch (turn_left_mapping[temp_value])
	{
		case 0:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_b[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_b[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_f[0], 31, 250, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_d[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&l_hindleg_vector_e[0], 31, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_hindleg_vector_d[0], 31, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 3);

			break;
		}

		case 1:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_c[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_a[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_c[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_e[0], 41, 250, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&r_hindleg_vector_f[0], 41, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_hindleg_vector_a[0], 41, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 1);

			break;
		}

		case 2:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_d[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_f[0], 21, 250, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_b[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_b[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&r_foreleg_vector_e[0], 21, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_foreleg_vector_d[0], 21, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 0);

			break;
		}

		case 3:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_e[0], 11, 250, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_c[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_a[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_c[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&l_foreleg_vector_f[0], 11, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_foreleg_vector_a[0], 11, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 2);

			break;
		}

		default:
		{
			break;
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			TurnRight
**
** 函数描述:			转弯控制
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void TurnRight(void)
{
	const int16 l_foreleg_vector_a[4] = {85, 60, 200, 80};
	const int16 l_foreleg_vector_b[4] = {90, 15, 180, 90};
	const int16 l_foreleg_vector_c[4] = {100, -10, 155, 70};
	const int16 l_foreleg_vector_d[4] = {120, -50, 190, 75};
	const int16 l_foreleg_vector_e[4] = {180, 20, 380, 130};
	const int16 l_foreleg_vector_f[4] = {260, 30, 300, 210};

	const int16 r_hindleg_vector_a[4] = {145, 60, 160, 55};
	const int16 r_hindleg_vector_b[4] = {120, -10, 190, 40};
	const int16 r_hindleg_vector_c[4] = {110, -20, 100, 30};
	const int16 r_hindleg_vector_d[4] = {150, -40, 190, 50};
	const int16 r_hindleg_vector_e[4] = {205, -30, 380, 130};
	const int16 r_hindleg_vector_f[4] = {260, 30, 345, 200};

	const int16 r_foreleg_vector_a[4] = {125, 65, 230, 95};
	const int16 r_foreleg_vector_b[4] = {90, 15, 180, 100};
	const int16 r_foreleg_vector_c[4] = {80, -45, 120, 70};
	const int16 r_foreleg_vector_d[4] = {100, -90, 180, 90};
	const int16 r_foreleg_vector_e[4] = {240, 20, 390, 200};
	const int16 r_foreleg_vector_f[4] = {170, 50, 300, 130};

	const int16 l_hindleg_vector_a[4] = {160, 85, 160, 20};
	const int16 l_hindleg_vector_b[4] = {120, 30, 190, 60};
	const int16 l_hindleg_vector_c[4] = {95, -50, 100, 00};
	const int16 l_hindleg_vector_d[4] = {145, -90, 160, 40};
	const int16 l_hindleg_vector_e[4] = {275, -30, 360, 200};
	const int16 l_hindleg_vector_f[4] = {200, 60, 340, 130};


	uint16 temp_time = 0;
	uint8 temp_value = 0;

	const uint8 turn_right_mapping[4] = {0, 1, 3, 2};

	signal_control.pGetByte(ADDR_MOVELEG, &temp_value);

	switch (turn_right_mapping[temp_value])
	{
		case 0:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_f[0], 11, 250, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_d[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_b[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_b[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&l_foreleg_vector_e[0], 11, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_foreleg_vector_d[0], 11, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 1);

			break;
		}

		case 1:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_c[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_e[0], 21, 250, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_c[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_a[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&r_foreleg_vector_f[0], 21, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_foreleg_vector_a[0], 21, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 3);

			break;
		}

		case 2:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_b[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_b[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_d[0], 31, 600, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_f[0], 41, 250, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&r_hindleg_vector_e[0], 41, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&r_hindleg_vector_d[0], 41, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 2);

			break;
		}

		case 3:
		{
			ControlSynchronous4Dof(&l_foreleg_vector_a[0], 11, 600, &temp_time);
			ControlSynchronous4Dof(&r_foreleg_vector_c[0], 21, 600, &temp_time);
			ControlSynchronous4Dof(&l_hindleg_vector_e[0], 31, 250, &temp_time);
			ControlSynchronous4Dof(&r_hindleg_vector_c[0], 41, 600, &temp_time);
			servo_control.pActionSync();
			led_control.pChangeLedBit(0);

			_delay_ms(250);

			ControlSynchronous4Dof(&l_hindleg_vector_f[0], 31, 100, &temp_time);
			servo_control.pActionSync();

			_delay_ms(100);

			ControlSynchronous4Dof(&l_hindleg_vector_a[0], 31, 250, &temp_time);
			servo_control.pActionSync();

			_delay_ms(250);

			signal_control.pSetByte(ADDR_MOVELEG, 0);

			break;
		}

		default:
		{
			break;
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			SitDown
**
** 函数描述:			坐下
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void SitDown(void)
{
	const int16 a[4]= {140-70, 0, 280, 140+70};
	const int16 b[4]= {130-70, 0, 240, 140+70};
	const int16 a1[4]= {100, 0, 0, 100};
	const int16 b1[4]= {130-50, 200, 240, 140+70};

	uint16 temp_time = 0;

	ControlSynchronous4Dof(&a[0], 11, 400, &temp_time);
	ControlSynchronous4Dof(&a[0], 21, 400, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 400, &temp_time);
	ControlSynchronous4Dof(&b[0], 41, 400, &temp_time);
	servo_control.pActionSync();

	_delay_ms(400);

	ControlSynchronous4Dof(&a1[0], 11, 800, &temp_time);
	ControlSynchronous4Dof(&a1[0], 21, 800, &temp_time);
	ControlSynchronous4Dof(&b1[0], 31, 1200, &temp_time);
	ControlSynchronous4Dof(&b1[0], 41, 1200, &temp_time);
	servo_control.pActionSync();

	_delay_ms(1200);

	ControlSynchronous1Dof(-120, 2, 700, &temp_time);
	servo_control.pActionSync();

	_delay_ms(700);

	ControlSynchronous1Dof(70, 2, 700, &temp_time);
	servo_control.pActionSync();

	_delay_ms(700);
}


/*************************************************************************************************************************
** 函数名称:			Stand
**
** 函数描述:			站立
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void Stand(void)
{
	const int16 a[4]= {140, 0, 280, 140};
	const int16 b[4]= {130, 0, 240, 140};

	uint16 temp_time = 0;

	ControlSynchronous4Dof(&a[0], 11, 1800, &temp_time);
	ControlSynchronous4Dof(&a[0], 21, 1800, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 1800, &temp_time);
	ControlSynchronous4Dof(&b[0], 41, 1800, &temp_time);

	servo_control.pActionSync();

	_delay_ms(200);

	ControlSynchronous1Dof(0, 1, 500, &temp_time);
	ControlSynchronous1Dof(0, 2, 500, &temp_time);
	ControlSynchronous1Dof(0, 3, 500, &temp_time);

	servo_control.pActionSync();

	_delay_ms(1800);
}


/*************************************************************************************************************************
** 函数名称:			ShookHands
**
** 函数描述:			握手
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/

void ShookHands(void)
{
	const int16 a3[4]= {-250, -50, 0, 250};
	const int16 a4[4]= {-350, -50, 0, 350};
	const int16 a[4] = {100, -50, 0, 100};
	const int16 a1[4]= {100-80, 0, -80, 100};
	const int16 a2[4]= {-300, -50, 0, 100};
	const int16 a5[4]= {40, 0, 0, 100};
	const int16 b[4] = {130-50, 200-40, 240, 140+70};
	const int16 b1[4] = {130-50, 200+40, 240, 140+70};

	uint8 i;
	uint16 temp_time = 0;

	ControlSynchronous1Dof(-100, 1, 600, &temp_time);
	ControlSynchronous1Dof(-70, 2, 600, &temp_time);
	servo_control.pActionSync();

	_delay_ms(600);

	ControlSynchronous4Dof(&a[0], 11, 300, &temp_time);
	ControlSynchronous4Dof(&a1[0], 21, 300, &temp_time);
	servo_control.pActionSync();

	_delay_ms(300);

	ControlSynchronous4Dof(&a2[0], 21, 600, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 600, &temp_time);
	ControlSynchronous4Dof(&b1[0], 41, 600, &temp_time);
	servo_control.pActionSync();

	_delay_ms(600);

	for(i=0;i<3;i++)
	{
		ControlSynchronous4Dof(&a3[0], 21, 300, &temp_time);
		servo_control.pActionSync();
		_delay_ms(300);

		ControlSynchronous4Dof(&a4[0], 21, 300, &temp_time);
		servo_control.pActionSync();
		_delay_ms(300);
	}

	ControlSynchronous4Dof(&a5[0], 21, 1200, &temp_time);
	servo_control.pActionSync();

	_delay_ms(1200);
}


/*************************************************************************************************************************
** 函数名称:			Urination
**
** 函数描述:			撒尿动作
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-24
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void Urination(void)
{
	const int16 a[4] = {170, 0, 340, 170};
	const int16 a1[4]= {0, 0, 0, 160};
	const int16 b[4] = {130, 80, 240, 140};
	const int16 b1[4]= {60, 0, 120, 240};
	const int16 b2[4]= {300, 300, -80, 90};
	const int16 b3[4]= {300, 270, -40, 90};
	const int16 b4[4]= {300, 310, -110, 90};
	const int16 b5[4]= {60, 80, 120, 60};

	uint16 temp_time = 0;

	ControlSynchronous1Dof(90, 1, 500, &temp_time);
	ControlSynchronous1Dof(150, 2, 700, &temp_time);
	servo_control.pActionSync();

	_delay_ms(800);

	ControlSynchronous4Dof(&a[0], 11, 1200, &temp_time);
	ControlSynchronous4Dof(&a1[0], 21, 1200, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 1200, &temp_time);
	ControlSynchronous4Dof(&b1[0], 41, 800, &temp_time);
	servo_control.pActionSync();

	_delay_ms(800);

	ControlSynchronous4Dof(&b5[0], 31, 850, &temp_time);
	ControlSynchronous4Dof(&b2[0], 41, 850, &temp_time);
	servo_control.pActionSync();

	_delay_ms(2900);

	ControlSynchronous1Dof(-90, 1, 500, &temp_time);
	ControlSynchronous1Dof(-150, 2, 700, &temp_time);
	servo_control.pActionSync();

	_delay_ms(600);

	ControlSynchronous4Dof(&b3[0], 41, 100, &temp_time);
	servo_control.pActionSync();

	_delay_ms(100);

	ControlSynchronous4Dof(&b4[0], 41, 100, &temp_time);
	servo_control.pActionSync();

	_delay_ms(300);

	ControlSynchronous4Dof(&b3[0], 41, 100, &temp_time);
	servo_control.pActionSync();

	_delay_ms(100);

	ControlSynchronous4Dof(&b4[0], 41, 100, &temp_time);
	servo_control.pActionSync();

	_delay_ms(300);
}


/*************************************************************************************************************************
** 函数名称:			GetDown
**
** 函数描述:			趴下
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void GetDown(void)
{
	const int16 a[4]= {-100, 0, 200, 240};
	const int16 b[4]= {-100, 70, 180, 240};

	uint16 temp_time = 0;

	ControlSynchronous4Dof(&a[0], 11, 1000, &temp_time);
	ControlSynchronous4Dof(&a[0], 21, 1000, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 1000, &temp_time);
	ControlSynchronous4Dof(&b[0], 41, 1000, &temp_time);
	servo_control.pActionSync();

	_delay_ms(1000);
}


/*************************************************************************************************************************
** 函数名称:			Stretched
**
** 函数描述:			伸懒腰
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void Stretched(void)
{
	uint16 temp_time = 0;
	uint8 i;

	const int16 a[4]= {-380, 0, 0, 140};
	const int16 a1[4]= {140, 0, 280, -80};
	const int16 b[4]= {30, 0, 240, 140};
	const int16 a2[4]= {180, 0, 0, 140};
	const int16 b2[4]= {380, 0, 0, 200};
	const int16 a3[4]= {140, 0, 280, 140};
	const int16 b3[4]= {130, 0, 240, 140};

	ControlSynchronous4Dof(&a1[0], 11, 300, &temp_time);
	servo_control.pActionSync();

	_delay_ms(300);

	ControlSynchronous4Dof(&a[0], 11, 1200, &temp_time);
	ControlSynchronous4Dof(&a[0], 21, 1200, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 1200, &temp_time);
	ControlSynchronous4Dof(&b[0], 41, 1200, &temp_time);
	servo_control.pActionSync();

	_delay_ms(2200);

	ControlSynchronous4Dof(&a2[0], 11, 1200, &temp_time);
	ControlSynchronous4Dof(&a2[0], 21, 1200, &temp_time);
	ControlSynchronous4Dof(&b2[0], 31, 1200, &temp_time);
	ControlSynchronous4Dof(&b2[0], 41, 1200, &temp_time);
	servo_control.pActionSync();

	_delay_ms(2200);

	ControlSynchronous4Dof(&a3[0], 11, 1800, &temp_time);
	ControlSynchronous4Dof(&a3[0], 21, 1800, &temp_time);
	ControlSynchronous4Dof(&b3[0], 31, 1800, &temp_time);
	ControlSynchronous4Dof(&b3[0], 41, 1800, &temp_time);
	servo_control.pActionSync();

	_delay_ms(1200);

	for(i=0;i<3;i++)
	{
		ControlSynchronous1Dof(-10, 34, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);

		ControlSynchronous1Dof(100, 34, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);

		ControlSynchronous1Dof(-10, 44, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);

		ControlSynchronous1Dof(100, 44, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);
	}
}


/*************************************************************************************************************************
** 函数名称:			Scratched
**
** 函数描述:			抓痒
**
**
** 输入变量:			const int16* vector, uint8 first_id, uint16 ref_time, uint16* calc_time;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			徐鑫鑫
** 创建日期:			2009-10-22
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void Scratched(void)
{
	const int16 a[4]= {-100, -70, 200, 240};
	const int16 a1[4]= {-100, 70, 200, 240};
	const int16 b[4]= {-100, -70, 180, 240};
	const int16 b1[4]= {-280, 30, 180, 240};

	uint8 i;

	uint16 temp_time = 0;

	ControlSynchronous4Dof(&a[0], 11, 400, &temp_time);
	ControlSynchronous4Dof(&a1[0], 21, 500, &temp_time);
	ControlSynchronous4Dof(&b[0], 31, 500, &temp_time);
	ControlSynchronous4Dof(&b1[0], 41, 800, &temp_time);
	servo_control.pActionSync();

	_delay_ms(200);

	ControlSynchronous1Dof(-250, 1, 500, &temp_time);
	ControlSynchronous1Dof(-100, 2, 700, &temp_time);
	servo_control.pActionSync();

	_delay_ms(1300);

	for(i=0;i<2;i++)
	{
		ControlSynchronous1Dof(230, 43, 200, &temp_time);
		ControlSynchronous1Dof(190, 44, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(200);

		ControlSynchronous1Dof(180, 43, 50, &temp_time);
		ControlSynchronous1Dof(240, 44, 50, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);
	}

	_delay_ms(500);

	for(i=0;i<6;i++)
	{
		ControlSynchronous1Dof(230, 43, 100, &temp_time);
		ControlSynchronous1Dof(190, 44, 100, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);

		ControlSynchronous1Dof(180, 43, 50, &temp_time);
		ControlSynchronous1Dof(240, 44, 50, &temp_time);
		servo_control.pActionSync();

		_delay_ms(50);
	}

	_delay_ms(700);

	for(i=0;i<2;i++)
	{
		ControlSynchronous1Dof(230, 43, 200, &temp_time);
		ControlSynchronous1Dof(190, 44, 200, &temp_time);
		servo_control.pActionSync();

		_delay_ms(200);

		ControlSynchronous1Dof(180, 43, 50, &temp_time);
		ControlSynchronous1Dof(240, 44, 50, &temp_time);
		servo_control.pActionSync();

		_delay_ms(100);
	}
}


/*************************************************************************************************************************
** 函数名称:			SlowlyWalk
**
** 函数描述:			4自由度同步位置控制
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-24
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void RobotDog_Behavior(void)
{
	uint8 temp_value = 0;

	signal_control.pGetByte(ADDR_BEHAVIOR, &temp_value);

	switch (temp_value)
	{
		case BEHAVIOR_HOLD:					// 保持
		{
			_delay_ms(100);
			break;
		}
		case BEHAVIOR_IDEL:					// 休息
		{
			_delay_ms(100);
			GetDown();
			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_STANDING:				// 站立
		{
			_delay_ms(100);
			Stand();
			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_STRETCH:				// 伸懒腰
		{
			_delay_ms(100);
			Stretched();
			Stand();
			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_WALKING:				// 步行
		{
			SlowlyWalk();
			break;
		}
		case BEHAVIOR_TURNLEFT:				// 左转
		{
			_delay_ms(40);
			TurnLeft();
			break;
		}
		case BEHAVIOR_TURNRIGHT:			// 右转
		{
			_delay_ms(40);
			TurnRight();
			break;
		}
		case BEHAVIOR_MARKING:				// 做标记
		{
			_delay_ms(100);
			Urination();
			Stand();
			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_TK:					// 挠痒痒
		{
			_delay_ms(100);
			Scratched();

			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_SIT:					// 坐下
		{
			_delay_ms(100);

			SitDown();

			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}
		case BEHAVIOR_HANDSHAKE:			// 握手
		{
			_delay_ms(100);

			ShookHands();

			signal_control.pSetByte(ADDR_BEHAVIOR, BEHAVIOR_HOLD);
			break;
		}

		default:
		{
			break;
		}
	}
}


/*************************************************************************************************************************
** 函数名称:			Test
**
** 函数描述:			测试
**
**
** 输入变量:			void;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-10-21
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
void Test(void)
{
	const int16 vector[4] = {0, 0, 0, 0};

	uint16 temp_time = 0;

	ControlSynchronous4Dof(&vector[0], 11, 1000, &temp_time);
	ControlSynchronous4Dof(&vector[0], 21, 1000, &temp_time);
	ControlSynchronous4Dof(&vector[0], 31, 1000, &temp_time);
	ControlSynchronous4Dof(&vector[0], 41, 1000, &temp_time);
	servo_control.pActionSync();
}


/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
