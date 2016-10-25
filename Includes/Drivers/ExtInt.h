/*******************************************************Copyright*********************************************************
**                                            ����������ʢ�����˼������޹�˾
**                                                       �з���
**                                               http://robot.up-tech.com
**
**-------------------------------------------------------�ļ���Ϣ---------------------------------------------------------
** �ļ�����:			ExtInt.h
** ����޶�����:		2009-08-17
** ���汾:			1.0
** ����:				 �ⲿ�жϵĽӿں����������ⲿ�жϵĳ�ʼ��������ģʽ��
**
**------------------------------------------------------------------------------------------------------------------------
** ������:			����
** ��������:			2008-08-17
** �汾:				1.0
** ����:				�ⲿ�жϵĽӿں����������ⲿ�жϵĳ�ʼ��������ģʽ��
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
**------------------------------------------------------------------------------------------------------------------------
** �޶���:
** �޶�����:
** �汾:
** ����:
**
*************************************************************************************************************************/
#ifndef EXTINT_H_
#define EXTINT_H_

#include "ConfigTypes.h"


#define		EXTINT_GLOBALS

#ifndef   EXTINT_GLOBALS
     #define EXTINT_EXT
#else
     #define EXTINT_EXT  extern
#endif


/*************************************************************************************************************************
														Ӳ������
*************************************************************************************************************************/
#define EXTINT_EICRA				EICRA					// �ⲿ�жϿ��ƼĴ��� A
/*************************************************************************************************************************
 *   7     6     5     4     3     2     1     0
 * ISC31 ISC30 ISC21 ISC20 ISC11 ISC10 ISC01 ISC00
 *
 * Bits 7..0 �C ISC31, ISC30 �C ISC00, ISC00: �ⲿ�ж� 3 - 0 ���е�ƽ����λ
 *
 * ISCn1	ISCn0			˵��
 *   0		  0		INTn Ϊ�͵�ƽʱ�����ж�����
 *   0		  1		����
 *   1		  0		INTn ���½��ز����첽�ж�����
 *   1		  0		INTn �������ز����첽�ж�����
 *
 * tINT	�첽 ( �ⲿ ) �жϵ���С�������	50ns
*************************************************************************************************************************/

#define EXTINT_EICRB				EICRB					// �ⲿ�жϿ��ƼĴ��� B
/*************************************************************************************************************************
 *   7     6     5     4     3     2     1     0
 * ISC71 ISC70 ISC61 ISC60 ISC51 ISC50 ISC41 ISC40
 *
 * Bits 7..0 �C ISC71, ISC70 - ISC41, ISC40: �ⲿ�ж� 7 - 4 ���е�ƽ����λ
 *
 * ISCn1	ISCn0			˵��
 *   0		  0		INTn Ϊ�͵�ƽʱ�����ж�����
 *   0		  1		INTn ������������߼���ƽ�任���������ж�
 *   1		  0		ֻҪ���β������� INTn �Ϸ������½��ؾͻ�����ж�����
 *   1		  1		ֻҪ���β������� INTn �Ϸ����������ؾͻ�����ж�����
 *
 * �ı� ISCn1/ISCn0 ʱһ��Ҫ��ͨ������ EIMSK �Ĵ������ж�ʹ��λ����ֹ�жϡ���
 * ���ڸı� ISCn1/ISCn0 �Ĺ����п��ܷ����жϡ�
*************************************************************************************************************************/

#define EXTINT_EIMSK				EIMSK					// �ⲿ�ж����μĴ���
/*************************************************************************************************************************
 *  7    6    5    4    3    2    1     0
 * INT7 INT6 INT5 INT4 INT3 INT2 INT1 IINT0
 *
 * Bits 7..0 �C INT7 �C INT0: �ⲿ�ж����� 7 - 0 ʹ��
 *
 * �� INT7 �C INT0 Ϊ '1��,����״̬�Ĵ��� SREG �� I ��־��λ,��Ӧ���ⲿ�����жϾ�ʹ���ˡ�
*************************************************************************************************************************/

#define EXTINT_EIFR					EIFR					// �ⲿ�жϱ�־�Ĵ���
/*************************************************************************************************************************
 *  7     6     5     4     3     2     1      0
 * INTF7 INTF6 INTF5 INTF4 INTF3 INTF2 INTF1 IINTF0
 *
 * Bits 7..0 �C INTF7 - INTF0: �ⲿ�жϱ�־ 7 - 0
 *
 * INT7:0 ���ŵ�ƽ��������ʱ�����ж�����,����λ��Ӧ���жϱ�־ INTF7:0
 * ��־λҲ����ͨ��д�� ��1�� �ķ�ʽ�����㡣
*************************************************************************************************************************/

#define EXTINT_PORT_INT0			PORTD
#define EXTINT_PORT_INT1			PORTD
#define EXTINT_PORT_INT2			PORTD
#define EXTINT_PORT_INT3			PORTD

#define EXTINT_PORT_INT4			PORTE
#define EXTINT_PORT_INT5			PORTE
#define EXTINT_PORT_INT6			PORTE
#define EXTINT_PORT_INT7			PORTE

#define EXTINT_PORT_BIT_INT0		PD0
#define EXTINT_PORT_BIT_INT1		PD1
#define EXTINT_PORT_BIT_INT2		PD2
#define EXTINT_PORT_BIT_INT3		PD3

#define EXTINT_PORT_BIT_INT4		PE4
#define EXTINT_PORT_BIT_INT5		PE5
#define EXTINT_PORT_BIT_INT6		PE6
#define EXTINT_PORT_BIT_INT7		PE7

#define EXTINT_DDR_INT0				DDRD
#define EXTINT_DDR_INT1				DDRD
#define EXTINT_DDR_INT2				DDRD
#define EXTINT_DDR_INT3				DDRD

#define EXTINT_DDR_INT4				DDRE
#define EXTINT_DDR_INT5				DDRE
#define EXTINT_DDR_INT6				DDRE
#define EXTINT_DDR_INT7				DDRE

#define EXTINT_DDR_BIT_INT0			DDD0
#define EXTINT_DDR_BIT_INT1			DDD1
#define EXTINT_DDR_BIT_INT2			DDD2
#define EXTINT_DDR_BIT_INT3			DDD3

#define EXTINT_DDR_BIT_INT4			DDE4
#define EXTINT_DDR_BIT_INT5			DDE5
#define EXTINT_DDR_BIT_INT6			DDE6
#define EXTINT_DDR_BIT_INT7			DDE7

/*************************************************************************************************************************
														���Ʋ�������
*************************************************************************************************************************/
#define EXTINT_INT0					0x00
#define EXTINT_INT1					0x01
#define EXTINT_INT2					0x02
#define EXTINT_INT3					0x03
#define EXTINT_INT4					0x04
#define EXTINT_INT5					0x05
#define EXTINT_INT6					0x06
#define EXTINT_INT7					0x07

#define EXTINT_MODE_MASK			0x03

#define EXTINT_MODE_LOW				0x00			// ���ƽ����
#define EXTINT_MODE_CHANGE			0x01			// ��ƽ�仯����(��INT0��INT3��Ч)
#define EXTINT_MODE_FALLING			0x02			// �½��ش���
#define EXTINT_MODE_RISING			0x03			// �����ش���


/*************************************************************************************************************************
														�ṹ�嶨��
*************************************************************************************************************************/
typedef struct SExtIntControl
{
	uint8				state;					// ״̬��

	tVoidVoid*			pInit;					// ��ʼ������
	tVoidUint8Uint8*	pSetMode;				// �����жϴ�������
	tVoidUint8*			pEnableInterrupt;		// ʹ���ж�
	tVoidUint8*			pDisableInterrupt;		// �����ж�
}EXTINT_CONTROL_STRUCT;


/*************************************************************************************************************************
														 ���ƽṹ������
*************************************************************************************************************************/
EXTINT_EXT EXTINT_CONTROL_STRUCT extint_control;


/*************************************************************************************************************************
														 ��������
*************************************************************************************************************************/


#endif
/*************************************************************************************************************************
** 														�ļ�����
*************************************************************************************************************************/