/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Queue.h
** 最后修订日期:		2009-03-07
** 最后版本:			1.0
** 描述:				队列操作函数库，包含队列的结构定义、初始化、入队、出队等
**
**------------------------------------------------------------------------------------------------------------------------
** 创建人:			律晔
** 创建日期:			2009-03-07
** 版本:				1.0
** 描述:				队列操作函数库，包含队列的结构定义、初始化、入队、出队等
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			律晔
** 修订日期:			2009-08-19
** 版本:				1.1
** 描述:				添加检查队列产度、状态功能
**
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
** 版本:
** 描述:
**
*************************************************************************************************************************/
#ifndef QUEUE_H_
#define QUEUE_H_

#include "ConfigTypes.h"


#define		QUEUE_GLOBALS

#ifndef   QUEUE_GLOBALS
     #define QUEUE_EXT     
#else 
     #define QUEUE_EXT  extern
#endif 


/*************************************************************************************************************************
 * 												结构体定义
*************************************************************************************************************************/

typedef struct SBufferQueue						// 定义缓冲队列结构
{
	uint8 state;								// 队列状态
	uint8 length;								// 队列长度

	uint8* init_address;						// 空间首地址

	uint8* front;								// 队头指针
	uint8* rear;								// 队尾指针
}BUFFER_QUEUE;


typedef void tInitBuffer(BUFFER_QUEUE *p_buffer, uint8 length);
typedef uint8 tTestEmptyBuffer(BUFFER_QUEUE *p_buffer);
typedef void tAddBuffer(BUFFER_QUEUE *p_buffer, uint8 item);
typedef uint8 tOutBuffer(BUFFER_QUEUE *p_buffer, uint8 *item);
typedef uint8 tCheckBufferLength(BUFFER_QUEUE *p_buffer);
typedef uint8 tCheckBufferState(BUFFER_QUEUE *p_buffer);
typedef void tClearBuffer(BUFFER_QUEUE *p_buffer);

typedef struct SBufferQueueControl
{
	tInitBuffer*		pInit;				// 初始化缓冲队列
	tTestEmptyBuffer*	pTestEmpty;			// 测试缓冲队列是否为空
	tAddBuffer*			pAdd;				// 向缓冲队列添加元素
	tOutBuffer*			pOut;				// 从缓冲队列取出元素
	tCheckBufferLength*	pCheckLength;		// 检查缓冲队列当前长度
	tCheckBufferState*	pCheckState;		// 检查缓冲队列状态
	tClearBuffer*		pClear;				// 清空缓冲队列
}BUFFER_QUEUE_CONTROL_STRUCT;


QUEUE_EXT BUFFER_QUEUE_CONTROL_STRUCT buffer_queue_control;	// 对外声明缓冲队列的控制结构体


/*************************************************************************************************************************
                                                         函数声明
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
**                                                      文件结束
*************************************************************************************************************************/
