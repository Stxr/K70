/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Mapping.c
** 最后修订日期:		2009-10-13
** 最后版本:			1.0
** 描述:				映射管理方法
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-10-13
** 版本:				1.0
** 描述:				映射管理函数，一维数组的映射反解处理
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
#include "Lib/Mapping.h"


/*************************************************************************************************************************
** 函数名称:			Inverse1DimArray
**
** 函数描述:			一维数组形式的映射反解，要求映射表是定长一维uint8数组，数组号对应唯一的映射值。
**
**
** 输入变量:			uint8* p_mapping(映射数组首地址), uint8 length,uint8 val(映射表中的值), uint8* site(在表中的位置，也就是反解值);
** 返回值:			uint8;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 Inverse1DimArray(uint8* p_mapping, uint8 length, uint8 val, uint8* site)
{
	uint8 count = 0;

	for (count = 0; count < length; count++)
	{
		if (val == *(p_mapping + count))
		{
			*site = count;
			return TRUE;
		}
	}

	return FALSE;
}


/*************************************************************************************************************************
														结构体声明
*************************************************************************************************************************/
MAPPING_CONTROL_STRUCT mapping_control =
{
	.pInverse1DimArray = Inverse1DimArray,
};


/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
