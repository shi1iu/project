#pragma once
#include "stdint.h"

/*
˵�����ÿ���빤��ΪVS2019
*��ӦcԴ��汾 V1.0.0
*/

#include "stdint.h"
#include "stddef.h"
//PCIeCard�豸���
typedef void* PCIeDeviceHandle;

typedef struct
{
	uint32_t FPGA_DriverVersion;
	uint32_t Aurora_LinkStatus;
	uint32_t DDR4_InitStatus;
}PCIeCard_Status_t;

//�豸����λ��
typedef struct
{
	uint32_t Slot;
	uint32_t Bus;
	uint32_t Dev;
	uint32_t Func;
}PCIeDevicePos_t;

/*****************************************************************************/
/*
* @brief    ��ȡ��ǰPCIeCard Lib�汾
*
* @param	out - VerString�����صİ汾�ַ�������ָ����ָ���������
*
*
* @return	��
*
******************************************************************************/
void PCIeCard_GetLibVersion(char* VerString);

/*****************************************************************************/
/*
* @brief    ��ȡ��ǰPC�����PCIeCard����
*
* @param	��
* @param	Value contains the 32 bit Value to be written at the specified
*           address.
*
* @return	-1  ��ʧ��
*           >=0 : ��ǰ�����������PCIeCard����
*
******************************************************************************/
int PCIeCard_GetDeviceNum();

/*****************************************************************************/
/*
* @brief    ͨ������ֵ��PCIeCard�豸
*
* @param	in  - DeviceIndex: PCIeCard����ֵ��������ֵ���ܴ��ڵ�ǰ����PCIeCard����
*
* @return	��ǰ�򿪵�PCIeCard�豸�����NULLΪ��ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/

PCIeDeviceHandle PCIeCard_OpenDeviceWithIdx(int DeviceIndex);

/*****************************************************************************/
/*
* @brief    ͨ���豸λ�ô�PCIeCard�豸
*
* @param	in  - PCIeCardPos: PCIeCard������λ�ã�����slot��bus��dev��func,
���ĸ�������ͨ���豸�������л�ȡ
*
* @return	��ǰ�򿪵�PCIeCard�豸�����NULLΪ��ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/
PCIeDeviceHandle PCIeCard_OpenDeviceWithPos(PCIeDevicePos_t PCIeCardPos);


/*****************************************************************************/
/*
* @brief    �ر�PCIeCard�豸
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
*
* @return	��
*
******************************************************************************/
void PCIeCard_CloseDevice(PCIeDeviceHandle PCIeDevHandle);

/*****************************************************************************/
/*
* @brief    ��ȡPCIe��������λ��
*
* @param	in  - DeviceIndex: PCIeCard����ֵ��������ֵ���ܴ��ڵ�ǰ����PCIeCard����
out - PCIeCardPos: PCIeCard������λ�ã�����slot��bus��dev��func
*
* @return	��
*
******************************************************************************/
int PCIeCard_GetPosition(int DeviceIndex, PCIeDevicePos_t* PCIeCardPos);


/*****************************************************************************/
/*
* @brief    ����PCIeCard�������������ڴ�ռ�
*
* @param	in  - size: ��Ҫ����Ĵ�С����λ���ֽڣ�
*
* @return	������ڴ�ռ��׵�ַ��NULLΪ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/
void* PCIeCard_Malloc(size_t size);

/*****************************************************************************/
/*
* @brief    �ͷ��ѷ����PCIeCard�������������ڴ�ռ�
*
* @param	in  - buffer: ��Ҫ�ͷŵ��ѷ�����ڴ�ռ��׵�ַ
*
* @return	��
*
******************************************************************************/
void PCIeCard_Free(void* buffer);


/*****************************************************************************/
/*
* @brief    ��ȡ��ǰ�豸״̬
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
*
* @return	PCIeCard_Status_t -  FPGA_DriverVersion : FPGA PCIe�����汾��
PCIeCard_Status_t -  Aurora_LinkStatus : Aurora����״̬��
1������������
0�����ӶϿ��������ת��ģ�鼰����
PCIeCard_Status_t -  DDR4_InitStatus : PCIeCard DDR4״̬
1:������0:�쳣
*
******************************************************************************/
PCIeCard_Status_t PCIeCard_GetStatus(PCIeDeviceHandle PCIeDevHandle);

/*****************************************************************************/
/*
* @brief    ���ö�д��ʱʱ��(V0.3.12���)
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	in  - Timeout_us: ��д��ʱʱ�䣬��λus
*
* @return	�ɹ��򷵻�0��-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/
uint32_t PCIeCard_SetTimeout(PCIeDeviceHandle PCIeDevHandle, uint32_t Timeout_us);


/*****************************************************************************/
/*
* @brief    ���ö�д��ʱʱ��(V0.3.12���)
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
*
* @return	�ɹ����д��ʱʱ�䣬��λus,-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/
uint32_t PCIeCard_GetTimeout(PCIeDeviceHandle PCIeDevHandle);


/*****************************************************************************/
/*
* @brief    ��ȡ�û��Ĵ���
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	in  - RegOffsetAddr: Ҫ��ȡ���û��Ĵ�����ַƫ������
*			user_in_reg0~15Ϊֻ���Ĵ�������Ӧ��ַΪ0x40~0x7C
*			user_out_reg0~15Ϊ�ɶ�д�Ĵ�������Ӧ��ַΪ0x0~0x3C
*			ÿ���Ĵ���ռ4�ֽڴ�С
*
* @return	��ַ��Ӧ�Ĵ�����ֵ
*
******************************************************************************/
uint32_t PCIeCard_ReadUserReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);

/*****************************************************************************/
/*
* @brief    д���û��Ĵ���
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	in  - RegOffsetAddr: Ҫд����û��Ĵ�����ַƫ������
*			user_out_reg0~15Ϊ��д�Ĵ�������Ӧ��ַΪ0x0~0x3C
*			ÿ���Ĵ���ռ4�ֽڴ�С
* @param    RegVal��Ҫд��Ĵ�����ֵ
*
* @return	0Ϊ�ɹ���-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
******************************************************************************/
int PCIeCard_WriteUserReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);

/*****************************************************************************/
/*
* @brief    ��ȡ�û��¼�
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	in  - EventID: �¼�ID�ţ���ΧΪ0~15����ӦFPGA�˵�user_irq0~15
*
* @return	0Ϊ�ɹ���-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
* @note		�ú���Ϊ�������������ú�ֱ���¼������Ż᷵��
******************************************************************************/
int PCIeCard_ReaduUserEvent(PCIeDeviceHandle PCIeDevHandle, int EventID);

/*****************************************************************************/
/*
* @brief    ��PCIeCardд������
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	in  - WrData: ���ݻ����׵�ַ���õ�ַ����ͨ��PCIeCard_Malloc��ȡ��buffer�׵�ַ
����ֵӦ����PCIeCard_Malloc����ֵһ�£�������ƫ��
* @param	in  - Size:   Ҫд�����ݵĴ�С����λ�ֽڣ���Size����4K��������ʱ��Aurora���͵����ݻ��

* @return	�ɹ��򷵻�д�����������-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
* @note		�ú���ֱ���������ݷ�����ϲŷ���
******************************************************************************/
size_t PCIeCard_Write(PCIeDeviceHandle PCIeDevHandle, char* WrData, size_t Size);

/*****************************************************************************/
/*
* @brief    ��PCIeCard��ȡ����
*
* @param	in  - PCIeDevHandle: PCIeCard�豸�����ͨ��PCIeCard_OpenDevice��ȡ
* @param	out - RdData: ���ݻ����׵�ַ���õ�ַ����ͨ��PCIeCard_Malloc��ȡ��buffer�׵�ַ��
����ֵӦ����PCIeCard_Malloc����ֵһ�£�������ƫ��
* @param	in  - Size:   Ҫ��ȡ���ݵĴ�С����λ�ֽ�

* @return	�ɹ��򷵻ض�ȡ����������-1Ϊʧ�ܣ����ʧ�ܣ�ͨ��GetLastError()�鿴ԭ��
*
* @note		�ú���Ϊ������ʽ����ʱʱ��Ϊ2s
��2s��δ�յ��������ݣ���᷵���Ѿ���ȡ��������
******************************************************************************/
size_t PCIeCard_Read(PCIeDeviceHandle PCIeDevHandle, char* RdData, size_t Size);



////���º���������ʹ�ã��˴�����˵��
int PCIeCard_WriteCtrlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
uint32_t PCIeCard_ReadCtrlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
int PCIeCard_WriteInternalReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
uint32_t PCIeCard_ReadInternalReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//int PCIeCard_WriteFireWallReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
//uint32_t PCIeCard_ReadFireWallReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//uint32_t PCIeCard_ReadControlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//int PCIeCard_ImmediatelyRead(PCIeDeviceHandle PCIeDevHandle, char* RdData, uintptr_t DeviceAddr, size_t Size);
//int PCIeCard_ImmediatelyWrite(PCIeDeviceHandle PCIeDevHandle, char* WrData, uintptr_t DeviceAddr, size_t Size);