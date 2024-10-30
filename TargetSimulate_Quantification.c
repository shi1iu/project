#include <stdint.h>
#include <time.h>
#include "TargetSimulate.h"
#include "PCIeCard.h"
#include "stdio.h"
#include <sys/resource.h>
#include <signal.h>
#include <string.h>
#include <stdlib.h>


extern Total_TargetSimulate_t Channel_TargetSimulate;
extern int offset;
extern char* combinedData_ptr;//下发参数指针
extern float g_Distance_points[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Distance_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Speed_points[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Speed_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Gain_points[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Gain_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Target_SwerlingModel_points[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float g_Target_SwerlingModel_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];
extern float Scatter_FIFO[Target_timeinterval];
extern float Scatter_EnableFIFO[Target_timeinterval];

extern int Package_Sequence;
extern int DeviceNum;
extern PCIeDeviceHandle* PCIeCardHandle_ptr;
extern char PCIeVersion[256];
extern Total_TargetSimulate_t Channel_TargetSimulate;
//extern char* combinedData_ptr;//下发参数指针
extern size_t headerSize;
extern PCIeDeviceHandle* PCIeCardHandle_ptr;

int g_Package_Sequence = 0;

long long time_ms() {
    // 获取当前时间
    time_t now;
    time(&now);

    // 将当前时间转换为UTC时间（GMT）
    struct tm* gmt = gmtime(&now);

    // 计算自当年1月1日以来的秒数
    gmt->tm_mon = 0;  // 设置月份为1月
    gmt->tm_mday = 1; // 设置日期为1日
    gmt->tm_hour = 0; // 设置小时为00
    gmt->tm_min = 0;  // 设置分钟为00
    gmt->tm_sec = 0;  // 设置秒为00

    // 将调整后的时间转换回time_t格式，得到当年1月1日的时间戳
    time_t year_start = mktime(gmt);

    // 计算当前时间与当年1月1日的差值（秒）
    double seconds_since_year_start = difftime(now, year_start);

    // 将秒转换为微秒
    long long microseconds_since_year_start = (long long)(seconds_since_year_start * 1000000);

    //printf("Microseconds since January 1st of this year: %lld\n", microseconds_since_year_start);
    return microseconds_since_year_start;
}
long long Unix_time() {
    struct timeval tv;
    long long milliseconds;

    gettimeofday(&tv, NULL);
    milliseconds = (long long)(tv.tv_sec) * 1000 + (long long)(tv.tv_usec) / 1000;
    //printf(" Unix Time: %lld\n", milliseconds);
    return milliseconds;
}
uint32_t swap_endian(uint32_t value) {//小端序转大端序
    //将数据转成大端序格式，则在内存中按小端序排序时就又可以恢复成原有数据,保证数据能按照输入值存入内存中
    return ((value & 0x000000FF) << 24) |
        ((value & 0x0000FF00) << 8) |
        ((value & 0x00FF0000) >> 8) |
        ((value & 0xFF000000) >> 24);
}

//制作发送的消息数据，打帧头，MakeMsg会分配MsgData空间
uint64_t TargetSimulate_MsgPackage( ParameterType type, void*MsgBodyPtr, uint64_t MsgBodySize, uint32_t DstID, char** MsgData)
{
	TargetSimulate_Header_t MsgHeader;
	uint64_t TrueMsgBodySize = MsgBodySize;	//有些控制指令无需由外部输入消息体，需重新赋值
	uint64_t TrueMsgBodyPtr = MsgBodyPtr;	//有些控制指令无需由外部输入消息体，此处进行重新赋值

    //memset(*MsgData, 0xAA, WRITE_SIZE_Header_param);	//先将所有数据填充

	EnvironmentConfig EnvironmentConfig_FPGA;
	ControlSignals ControlSignals_FPGA;
	ChannelControlParams  ChannelControlParams_FPGA;

    MsgHeader.Msg_FrameHeader = 0x55555555;
    //length
    MsgHeader.Msg_ID_Send = 0x30000000;
    //recv
	MsgHeader.Msg_ID_Recv = DstID;
    //topic
    MsgHeader.Msg_SeqNumber = Package_Sequence;
    MsgHeader.Msg_EffectiveTime = time_ms();
    MsgHeader.Msg_IntervalTime = Unix_time();
    MsgHeader.Msg_FileLength = 0;
    Message_ICD temp_version0 = { 0x1, 0x0, 0x1 };
    MsgHeader.Msg_ICDVersion = temp_version0;
    MsgHeader.Msg_Reserve = 0;
    MsgHeader.Msg_CRC_value = 0;

    switch (type) 
	{
		case Environment_SET:
		{
			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10001;

			break;
		}
		case ControlSignals_ResetAssert:
		{
			TrueMsgBodySize = WRITE_SIZE_ControlSignals;

			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10300;

			TrueMsgBodyPtr = &ControlSignals_FPGA;
			ControlSignals_FPGA.update_flag      = 0x0;
			ControlSignals_FPGA.start_end_flag   = 0x0;
			ControlSignals_FPGA.soft_reset       = 0x1;
			ControlSignals_FPGA.signal_threshold = 0xfff;

			break;
		}
		case ControlSignals_ResetDeAssert:
		{
			TrueMsgBodySize = WRITE_SIZE_ControlSignals;
			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10300;

			TrueMsgBodyPtr = &ControlSignals_FPGA;
			ControlSignals_FPGA.update_flag = 0x0;
			ControlSignals_FPGA.start_end_flag = 0x0;
			ControlSignals_FPGA.soft_reset = 0x0;
			ControlSignals_FPGA.signal_threshold = 0xfff;

			break;
		}
		case ControlSignals_Start:
		{
			TrueMsgBodySize = WRITE_SIZE_ControlSignals;

			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10300;
			memcpy(combinedData_ptr + offset, &MsgHeader, Write_Size_Header_Param);
			offset += Write_Size_Header_Param;

			TrueMsgBodyPtr = &ControlSignals_FPGA;

			ControlSignals_FPGA.update_flag = 0x1;
			ControlSignals_FPGA.start_end_flag = 0x1;
			ControlSignals_FPGA.soft_reset = 0x0;
			ControlSignals_FPGA.signal_threshold = 0xfff;

			break;
		}
		case ControlSignals_Stop:
		{
			TrueMsgBodySize = WRITE_SIZE_ControlSignals;
			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10300;

			TrueMsgBodyPtr = &ControlSignals_FPGA;

			ControlSignals_FPGA.update_flag = 0x0;
			ControlSignals_FPGA.start_end_flag = 0x0;
			ControlSignals_FPGA.soft_reset = 0x0;
			ControlSignals_FPGA.signal_threshold = 0xfff;

			break;
		}
		case ChannelControlParams_SET:
		{
			MsgHeader.Msg_Length = 0x80;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10002;

			break;
		}
		case Package_target_SET://目标模拟参数头
		{
			MsgHeader.Msg_Length = 0x3C0;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10100;//通道0
			break;
		}

    }
	//分配空间
	uint32_t malloc_size = Write_Size_Header_Param + TrueMsgBodySize;
	*MsgData = (char*)malloc(malloc_size);	//这个二重指针暂时没有用到，如果需要用到，变量跳出函数后，变量值变化则使用（5.30）
	if (*MsgData == NULL)
	{
		printf("MsgData malloc failed\n");
	}

	memcpy(*MsgData,&MsgHeader, Write_Size_Header_Param);
	memcpy(*MsgData+Write_Size_Header_Param, TrueMsgBodyPtr, TrueMsgBodySize);

    Package_Sequence++;

    return TrueMsgBodySize + Write_Size_Header_Param;
}
size_t CalcResultPackage(int IntervalNum, void* CalcResultPtr)
{
	char* tmpMsgData;
	int num = 24000;
	uint64_t TotalMsgSize=0;
	uint64_t MsgSize;
	//按照每1ms进行打包
	for (int interval = 0; interval < IntervalNum; interval++)
	{
		//消息体打包
		Channel_TargetSimulate.Package_target[interval].En_1 = 0x01; //使能
		//点目标
		for (unsigned int P_target = 0; P_target < TARGETS_PER_CHANNEL; P_target++)
		{
			//Channel_TargetSimulate.Package_target[interval].Point_Speed[P_target] = (int)(g_Speed_points[P_target][interval] * Speed_Quantization_factor);
			//Channel_TargetSimulate.Package_target[interval].Point_Distance[P_target] = (int)(g_Distance_points[P_target][interval] * round_trip * Distance_Quantization_factor);
			Channel_TargetSimulate.Package_target[interval].Point_Speed[P_target] = 1000;
			//Channel_TargetSimulate.Package_target[interval].Point_Distance[P_target] = num * P_target+ num + 1000 * interval;
			Channel_TargetSimulate.Package_target[interval].Point_Distance[P_target] = 1000 * P_target+ num + 5000 * interval;
			Channel_TargetSimulate.Package_target[interval].Point_Gain_points[P_target] = (int)(g_Gain_points[P_target][interval] * Gain_Quantization_factor);
			Channel_TargetSimulate.Package_target[interval].Point_SwerlingMode[P_target] = (int)g_Target_SwerlingModel_points[P_target][interval];
			Channel_TargetSimulate.Package_target[interval].Point_PulseInterval[P_target] = (int)0;
			printf("The target[%d] interval[%d] distance is : %d \n", P_target, interval,Channel_TargetSimulate.Package_target[interval].Point_Distance[P_target]);
			//printf("At %d ms,The target's Point_SwerlingMode is : %d \n", interval, Channel_TargetSimulate.Package_target[interval].Point_SwerlingMode[P_target]);
			//printf("The target's Point_Gain_points is : %d \n", Channel_TargetSimulate.Package_target[interval].Point_Gain_points[P_target]);
		}


		//散射点目标
		for (unsigned int S_target = 0; S_target < TARGETS_PER_CHANNEL; S_target++)
		{
			//Channel_TargetSimulate.Package_target[interval].Scatter_Speed[S_target] = (int)(g_Speed_scatters[S_target][interval] * Speed_Quantization_factor);
			//Channel_TargetSimulate.Package_target[interval].Scatter_Distance[S_target] = (int)(g_Distance_scatters[S_target][interval] * round_trip * Distance_Quantization_factor);
			Channel_TargetSimulate.Package_target[interval].Scatter_Speed[S_target] = 1000;
			Channel_TargetSimulate.Package_target[interval].Scatter_Distance[S_target] = num + 50000 * S_target + 1000 * interval;
			Channel_TargetSimulate.Package_target[interval].Scatter_Gain_points[S_target] = (int)(g_Gain_scatters[S_target][interval] * Gain_Quantization_factor);
			Channel_TargetSimulate.Package_target[interval].Scatter_SwerlingMode[S_target] = (int)g_Target_SwerlingModel_scatters[S_target][interval];
			Channel_TargetSimulate.Package_target[interval].Scatter_PulseInterval[S_target] = (int)0;
			//printf("At %d ms,The target's Scatter_SwerlingMode is : %d \n", interval,Package_target[interval].Scatter_SwerlingMode[S_target]);

		}
		for (unsigned int i = 0; i < 7; i++)
			Channel_TargetSimulate.Package_target[interval].fifo_delay[i] = 3;
		for (unsigned int j = 0; j < 3; j++)
			Channel_TargetSimulate.Package_target[interval].fifoenable[j] = 1;


		

		MsgSize = TargetSimulate_MsgPackage(Package_target_SET, &Channel_TargetSimulate.Package_target[interval], sizeof(Channel_TargetSimulate.Package_target[interval]), SRC_DST_ID_SPB2, &tmpMsgData);



		memcpy(CalcResultPtr + interval * MsgSize, tmpMsgData, MsgSize);//复制消息体数据

		free(tmpMsgData);

		TotalMsgSize += MsgSize;
	}

	return TotalMsgSize;
}


//void enable_target_simulation() 
//{
//
//    //上位机发送命令
//    char* MsgData = NULL;
//    //环境参数配置消息打包
//
//	EnvironmentConfig EnvironmentConfig_FPGA;
//	EnvironmentConfig_FPGA.setFrequencyMode = 0;
//	EnvironmentConfig_FPGA.manualFrequency = 0xfff;
//	EnvironmentConfig_FPGA.intermediateFrequencyOffset = 0x30;
//	EnvironmentConfig_FPGA.receiveRfAttenuation = 0x11;
//	EnvironmentConfig_FPGA.receiveIntermediateFrequencyAttenuation = 0x12;
//	EnvironmentConfig_FPGA.rfSignalCenterFrequency = 0xfff;
//	EnvironmentConfig_FPGA.frequencyMeasurementMode = 0x2;
//	EnvironmentConfig_FPGA.frequencyInterval = 0x666;
//	EnvironmentConfig_FPGA.operationalMode = 0x10;
//	EnvironmentConfig_FPGA.transmitChannelPowerAmplifierControl = 0x2;
//
//	uint64_t TargetSimulate_MsgPackage(ParameterType type, void* MsgBodyPtr, uint64_t MsgBodySize, uint8_t DstID, char** MsgData)
//
//    //复位有效
//    TargetSimulate_Header_t ControlSignals_FPGA_t1;
//    Package_Sequence = TargetSimulate_MsgPackage(ControlSignals_FPGA_t1, ControlSignals_ResetAssert, Package_Sequence, &MsgData);
//
//    //复位无效
//    TargetSimulate_Header_t  ChannelControlParams_FPGA_t;
//    Package_Sequence = TargetSimulate_MsgPackage(ChannelControlParams_FPGA_t, ControlSignals_ResetDeAssert, Package_Sequence, &MsgData);
//
//	ChannelControlParams_FPGA.channel_enable                           = 0x1;
//	ChannelControlParams_FPGA.channel0_analog_attenuation              = 0x1;
//	ChannelControlParams_FPGA.channel0_digital_I_amplitude_coefficient = 0x1;
//	ChannelControlParams_FPGA.channel0_digital_Q_amplitude_coefficient = 0x1;
//	ChannelControlParams_FPGA.channel1_analog_attenuation = 0x1;
//	ChannelControlParams_FPGA.channel1_digital_I_amplitude_coefficient = 0x1;
//	ChannelControlParams_FPGA.channel1_digital_Q_amplitude_coefficient = 0x1;
//    TargetSimulate_Header_t ControlSignals_FPGA_t0;
//    Package_Sequence = TargetSimulate_MsgPackage(ControlSignals_FPGA_t0, ControlSignals_SET0, Package_Sequence, &MsgData);
//
//}


