#pragma once
#include "stdint.h"

/*
说明：该库编译工具为VS2019
*对应c源码版本 V1.0.0
*/

#include "stdint.h"
#include "stddef.h"
//PCIeCard设备句柄
typedef void* PCIeDeviceHandle;

typedef struct
{
	uint32_t FPGA_DriverVersion;
	uint32_t Aurora_LinkStatus;
	uint32_t DDR4_InitStatus;
}PCIeCard_Status_t;

//设备物理位置
typedef struct
{
	uint32_t Slot;
	uint32_t Bus;
	uint32_t Dev;
	uint32_t Func;
}PCIeDevicePos_t;

/*****************************************************************************/
/*
* @brief    获取当前PCIeCard Lib版本
*
* @param	out - VerString：返回的版本字符串，该指针所指区域需大于
*
*
* @return	无
*
******************************************************************************/
void PCIeCard_GetLibVersion(char* VerString);

/*****************************************************************************/
/*
* @brief    获取当前PC插入的PCIeCard数量
*
* @param	无
* @param	Value contains the 32 bit Value to be written at the specified
*           address.
*
* @return	-1  ：失败
*           >=0 : 当前服务器插入的PCIeCard数量
*
******************************************************************************/
int PCIeCard_GetDeviceNum();

/*****************************************************************************/
/*
* @brief    通过索引值打开PCIeCard设备
*
* @param	in  - DeviceIndex: PCIeCard索引值，该索引值不能大于当前插入PCIeCard数量
*
* @return	当前打开的PCIeCard设备句柄，NULL为打开失败，通过GetLastError()查看原因
*
******************************************************************************/

PCIeDeviceHandle PCIeCard_OpenDeviceWithIdx(int DeviceIndex);

/*****************************************************************************/
/*
* @brief    通过设备位置打开PCIeCard设备
*
* @param	in  - PCIeCardPos: PCIeCard的物理位置，包括slot，bus，dev，func,
这四个参数可通过设备管理器中获取
*
* @return	当前打开的PCIeCard设备句柄，NULL为打开失败，通过GetLastError()查看原因
*
******************************************************************************/
PCIeDeviceHandle PCIeCard_OpenDeviceWithPos(PCIeDevicePos_t PCIeCardPos);


/*****************************************************************************/
/*
* @brief    关闭PCIeCard设备
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
*
* @return	无
*
******************************************************************************/
void PCIeCard_CloseDevice(PCIeDeviceHandle PCIeDevHandle);

/*****************************************************************************/
/*
* @brief    获取PCIe卡的物理位置
*
* @param	in  - DeviceIndex: PCIeCard索引值，该索引值不能大于当前插入PCIeCard数量
out - PCIeCardPos: PCIeCard的物理位置，包括slot，bus，dev，func
*
* @return	无
*
******************************************************************************/
int PCIeCard_GetPosition(int DeviceIndex, PCIeDevicePos_t* PCIeCardPos);


/*****************************************************************************/
/*
* @brief    分配PCIeCard传输数据所用内存空间
*
* @param	in  - size: 需要分配的大小（单位：字节）
*
* @return	分配的内存空间首地址，NULL为分配失败，通过GetLastError()查看原因
*
******************************************************************************/
void* PCIeCard_Malloc(size_t size);

/*****************************************************************************/
/*
* @brief    释放已分配的PCIeCard传输数据所用内存空间
*
* @param	in  - buffer: 需要释放的已分配的内存空间首地址
*
* @return	无
*
******************************************************************************/
void PCIeCard_Free(void* buffer);


/*****************************************************************************/
/*
* @brief    获取当前设备状态
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
*
* @return	PCIeCard_Status_t -  FPGA_DriverVersion : FPGA PCIe驱动版本号
PCIeCard_Status_t -  Aurora_LinkStatus : Aurora连接状态，
1：连接正常，
0：连接断开，检查光电转换模块及光纤
PCIeCard_Status_t -  DDR4_InitStatus : PCIeCard DDR4状态
1:正常，0:异常
*
******************************************************************************/
PCIeCard_Status_t PCIeCard_GetStatus(PCIeDeviceHandle PCIeDevHandle);

/*****************************************************************************/
/*
* @brief    设置读写超时时间(V0.3.12添加)
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	in  - Timeout_us: 读写超时时间，单位us
*
* @return	成功则返回0，-1为失败，如果失败，通过GetLastError()查看原因
*
******************************************************************************/
uint32_t PCIeCard_SetTimeout(PCIeDeviceHandle PCIeDevHandle, uint32_t Timeout_us);


/*****************************************************************************/
/*
* @brief    设置读写超时时间(V0.3.12添加)
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
*
* @return	成功则读写超时时间，单位us,-1为失败，如果失败，通过GetLastError()查看原因
*
******************************************************************************/
uint32_t PCIeCard_GetTimeout(PCIeDeviceHandle PCIeDevHandle);


/*****************************************************************************/
/*
* @brief    读取用户寄存器
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	in  - RegOffsetAddr: 要读取的用户寄存器地址偏移量，
*			user_in_reg0~15为只读寄存器，对应地址为0x40~0x7C
*			user_out_reg0~15为可读写寄存器，对应地址为0x0~0x3C
*			每个寄存器占4字节大小
*
* @return	地址对应寄存器的值
*
******************************************************************************/
uint32_t PCIeCard_ReadUserReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);

/*****************************************************************************/
/*
* @brief    写入用户寄存器
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	in  - RegOffsetAddr: 要写入的用户寄存器地址偏移量，
*			user_out_reg0~15为读写寄存器，对应地址为0x0~0x3C
*			每个寄存器占4字节大小
* @param    RegVal：要写入寄存器的值
*
* @return	0为成功，-1为失败，如果失败，通过GetLastError()查看原因
*
******************************************************************************/
int PCIeCard_WriteUserReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);

/*****************************************************************************/
/*
* @brief    获取用户事件
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	in  - EventID: 事件ID号，范围为0~15，对应FPGA端的user_irq0~15
*
* @return	0为成功，-1为失败，如果失败，通过GetLastError()查看原因
*
* @note		该函数为阻塞函数，调用后直到事件触发才会返回
******************************************************************************/
int PCIeCard_ReaduUserEvent(PCIeDeviceHandle PCIeDevHandle, int EventID);

/*****************************************************************************/
/*
* @brief    向PCIeCard写入数据
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	in  - WrData: 数据缓存首地址，该地址必须通过PCIeCard_Malloc获取的buffer首地址
即该值应该与PCIeCard_Malloc返回值一致，不能有偏移
* @param	in  - Size:   要写入数据的大小，单位字节，当Size不是4K的整数倍时，Aurora发送的数据会多

* @return	成功则返回写入的数据量，-1为失败，如果失败，通过GetLastError()查看原因
*
* @note		该函数直到所有数据发送完毕才返回
******************************************************************************/
size_t PCIeCard_Write(PCIeDeviceHandle PCIeDevHandle, char* WrData, size_t Size);

/*****************************************************************************/
/*
* @brief    从PCIeCard读取数据
*
* @param	in  - PCIeDevHandle: PCIeCard设备句柄，通过PCIeCard_OpenDevice获取
* @param	out - RdData: 数据缓存首地址，该地址必须通过PCIeCard_Malloc获取的buffer首地址，
即该值应该与PCIeCard_Malloc返回值一致，不能有偏移
* @param	in  - Size:   要读取数据的大小，单位字节

* @return	成功则返回读取的数据量，-1为失败，如果失败，通过GetLastError()查看原因
*
* @note		该函数为阻塞方式，超时时间为2s
如2s内未收到所有数据，则会返回已经读取的数据量
******************************************************************************/
size_t PCIeCard_Read(PCIeDeviceHandle PCIeDevHandle, char* RdData, size_t Size);



////以下函数仅调试使用，此处不做说明
int PCIeCard_WriteCtrlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
uint32_t PCIeCard_ReadCtrlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
int PCIeCard_WriteInternalReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
uint32_t PCIeCard_ReadInternalReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//int PCIeCard_WriteFireWallReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr, uint32_t RegVal);
//uint32_t PCIeCard_ReadFireWallReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//uint32_t PCIeCard_ReadControlReg(PCIeDeviceHandle PCIeDevHandle, uint32_t RegOffsetAddr);
//int PCIeCard_ImmediatelyRead(PCIeDeviceHandle PCIeDevHandle, char* RdData, uintptr_t DeviceAddr, size_t Size);
//int PCIeCard_ImmediatelyWrite(PCIeDeviceHandle PCIeDevHandle, char* WrData, uintptr_t DeviceAddr, size_t Size);