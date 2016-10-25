/*******************************************************Copyright*********************************************************
**                                            北京博创兴盛机器人技术有限公司
**                                                       研发部
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------文件信息---------------------------------------------------------
** 文件名称:			Queue.c
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
#include "Lib/Queue.h"


/*************************************************************************************************************************
** 函数名称:			InitBufferQueue
**
** 函数描述:			缓冲队列初始化；并按照初始化长度分配内存空间；
**                      
**                      
**					                 
** 输入变量:			struct BUFFER_QUEUE *p_buffer; uint8 length;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-03-07
**------------------------------------------------------------------------------------------------------------------------
** 修订人:			律晔
** 修订日期:			2009-08-10
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void InitBufferQueue(BUFFER_QUEUE *p_buffer, uint8 length)
{
	uint8* p_free_space = NULL;

	SetReg(p_buffer->state, 0);										// 初始化状态寄存器

	if(0x00 == length)
	{
		SetRegMask(p_buffer->state, ERR_MASK, ERR_VALUE);			// 置错误类型标志位
		return;
	}

	p_buffer->length = length;										// 初始化缓冲区长度

	p_free_space = malloc(length);									// 分配长度为length的RAM空间

	if (NULL == p_free_space)
	{
		SetRegMask(p_buffer->state, ERR_MASK, ERR_NOTAVAIL);		// 置错误类型标志位
		return;
	}

	p_buffer->init_address = p_free_space;

	p_buffer->rear = p_free_space;
	p_buffer->front = p_free_space;

	SetRegBit(p_buffer->state, INIT_COMPLETE);						// 置位初始化完成标志
}


/*************************************************************************************************************************
** 函数名称:			TestEmptyBufferQueue
**
** 函数描述:			测试缓冲队列是否为空，为空则返回TRUE，不为空则返回FALSE。
**                      
**                      
**					                 
** 输入变量:			struct BUFFER_QUEUE *p_buffer;
** 返回值:			void;
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
static uint8 TestEmptyBufferQueue(BUFFER_QUEUE *p_buffer)
{
	return (p_buffer->rear == p_buffer->front);
}


/*************************************************************************************************************************
** 函数名称:			AddBufferQueue
**
** 函数描述:			向缓冲队列添加元素，队尾指针rear加1。
**
**					                 
** 输入变量:			struct BUFFER_QUEUE *p_buffer; uint8 &item;
** 返回值:			void;
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
static void AddBufferQueue(BUFFER_QUEUE *p_buffer, uint8 item)
{
	*p_buffer->rear = item;												// 数据入队

	p_buffer->rear++;													// 队尾地址＋1

	if (p_buffer->rear >= (p_buffer->init_address + p_buffer->length))	// 队尾地址超过分配内存边界
	{
		p_buffer->rear = p_buffer->init_address;						// 队尾地址回缓冲区首地址
	}
	
	if (p_buffer->rear == p_buffer->front)					// 队列长度大于缓冲区长度
	{
		SetRegMask(p_buffer->state, ERR_MASK, ERR_OVERRUN);				// 置溢出错误标志
	}
}

/*************************************************************************************************************************
** 函数名称:			OutBufferQueue
**
** 函数描述:			从队列取出元素，队头指针front加1。当队列为空时返回FALSE，操作成功返回TRUE。
**                      
**                      
**					                 
** 输入变量:			struct BUFFER_QUEUE *p_buffer; uint8 &item;
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
static uint8 OutBufferQueue(BUFFER_QUEUE *p_buffer, uint8 *p_item)
{
	if(TestEmptyBufferQueue(p_buffer))
	{
		return FALSE;
	}
	else
	{
		*p_item = *p_buffer->front;											// 数据出队

		p_buffer->front++;
		
		if(p_buffer->front >= (p_buffer->init_address + p_buffer->length))
		{
			p_buffer->front = p_buffer->init_address;
		}

		return TRUE;
	}
}


/*************************************************************************************************************************
** 函数名称:			CheckBufferQueueLength
**
** 函数描述:			检查缓冲队列长度;
**
** 输入变量:			struct BUFFER_QUEUE *p_buffer;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CheckBufferQueueLength(BUFFER_QUEUE* p_buffer)
{
	uint8 temp_value = 0;

	if (p_buffer->rear >= p_buffer->front)
	{
		temp_value = p_buffer->rear - p_buffer->front;
	}
	else
	{
		temp_value = p_buffer->front - p_buffer->rear;
		temp_value = p_buffer->length - temp_value;
	}

	return temp_value;
}


/*************************************************************************************************************************
** 函数名称:			CheckBufferQueueState
**
** 函数描述:			检查缓冲队列状态，分配内存空间失败、数据溢出等;
**
**
**
** 输入变量:			struct BUFFER_QUEUE *p_buffer;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static uint8 CheckBufferQueueState(BUFFER_QUEUE *p_buffer)
{
	return p_buffer->state;
}


/*************************************************************************************************************************
** 函数名称:			ClearBufferQueue
**
** 函数描述:			清空队列;
**
**
** 输入变量:			struct BUFFER_QUEUE *p_buffer;
** 返回值:			void;
**
** 使用宏或常量:		None;
** 使用全局变量:		None;
**
** 调用函数:			None;
**
** 创建人:			律晔
** 创建日期:			2009-08-19
**------------------------------------------------------------------------------------------------------------------------
** 修订人:
** 修订日期:
**------------------------------------------------------------------------------------------------------------------------
*************************************************************************************************************************/
static void ClearBufferQueue(BUFFER_QUEUE *p_buffer)
{
	p_buffer->rear = p_buffer->front;								// 移动头尾指针

	SetReg(p_buffer->state, 0);										// 初始化状态寄存器
	SetRegBit(p_buffer->state, INIT_COMPLETE);						// 置位初始化完成标志
}


/*************************************************************************************************************************
														结构体声明
*************************************************************************************************************************/

BUFFER_QUEUE_CONTROL_STRUCT buffer_queue_control =
{
	.pInit = InitBufferQueue,
	.pTestEmpty = TestEmptyBufferQueue,
	.pAdd = AddBufferQueue,
	.pOut = OutBufferQueue,
	.pCheckLength = CheckBufferQueueLength,
	.pCheckState = CheckBufferQueueState,
	.pClear = ClearBufferQueue,
};


/*************************************************************************************************************************
**														文件结束
*************************************************************************************************************************/
