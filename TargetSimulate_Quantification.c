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
extern char* combinedData_ptr;//�·�����ָ��
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
//extern char* combinedData_ptr;//�·�����ָ��
extern size_t headerSize;
extern PCIeDeviceHandle* PCIeCardHandle_ptr;

int g_Package_Sequence = 0;

long long time_ms() {
    // ��ȡ��ǰʱ��
    time_t now;
    time(&now);

    // ����ǰʱ��ת��ΪUTCʱ�䣨GMT��
    struct tm* gmt = gmtime(&now);

    // �����Ե���1��1������������
    gmt->tm_mon = 0;  // �����·�Ϊ1��
    gmt->tm_mday = 1; // ��������Ϊ1��
    gmt->tm_hour = 0; // ����СʱΪ00
    gmt->tm_min = 0;  // ���÷���Ϊ00
    gmt->tm_sec = 0;  // ������Ϊ00

    // ���������ʱ��ת����time_t��ʽ���õ�����1��1�յ�ʱ���
    time_t year_start = mktime(gmt);

    // ���㵱ǰʱ���뵱��1��1�յĲ�ֵ���룩
    double seconds_since_year_start = difftime(now, year_start);

    // ����ת��Ϊ΢��
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
uint32_t swap_endian(uint32_t value) {//С����ת�����
    //������ת�ɴ�����ʽ�������ڴ��а�С��������ʱ���ֿ��Իָ���ԭ������,��֤�����ܰ�������ֵ�����ڴ���
    return ((value & 0x000000FF) << 24) |
        ((value & 0x0000FF00) << 8) |
        ((value & 0x00FF0000) >> 8) |
        ((value & 0xFF000000) >> 24);
}

//�������͵���Ϣ���ݣ���֡ͷ��MakeMsg�����MsgData�ռ�
uint64_t TargetSimulate_MsgPackage( ParameterType type, void*MsgBodyPtr, uint64_t MsgBodySize, uint32_t DstID, char** MsgData)
{
	TargetSimulate_Header_t MsgHeader;
	uint64_t TrueMsgBodySize = MsgBodySize;	//��Щ����ָ���������ⲿ������Ϣ�壬�����¸�ֵ
	uint64_t TrueMsgBodyPtr = MsgBodyPtr;	//��Щ����ָ���������ⲿ������Ϣ�壬�˴��������¸�ֵ

    //memset(*MsgData, 0xAA, WRITE_SIZE_Header_param);	//�Ƚ������������

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
		case Package_target_SET://Ŀ��ģ�����ͷ
		{
			MsgHeader.Msg_Length = 0x3C0;
			MsgHeader.Msg_ID_Recv = 0x30000012;
			MsgHeader.Msg_Topic = 0x40F10100;//ͨ��0
			break;
		}

    }
	//����ռ�
	uint32_t malloc_size = Write_Size_Header_Param + TrueMsgBodySize;
	*MsgData = (char*)malloc(malloc_size);	//�������ָ����ʱû���õ��������Ҫ�õ����������������󣬱���ֵ�仯��ʹ�ã�5.30��
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
	//����ÿ1ms���д��
	for (int interval = 0; interval < IntervalNum; interval++)
	{
		//��Ϣ����
		Channel_TargetSimulate.Package_target[interval].En_1 = 0x01; //ʹ��
		//��Ŀ��
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


		//ɢ���Ŀ��
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



		memcpy(CalcResultPtr + interval * MsgSize, tmpMsgData, MsgSize);//������Ϣ������

		free(tmpMsgData);

		TotalMsgSize += MsgSize;
	}

	return TotalMsgSize;
}


//void enable_target_simulation() 
//{
//
//    //��λ����������
//    char* MsgData = NULL;
//    //��������������Ϣ���
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
//    //��λ��Ч
//    TargetSimulate_Header_t ControlSignals_FPGA_t1;
//    Package_Sequence = TargetSimulate_MsgPackage(ControlSignals_FPGA_t1, ControlSignals_ResetAssert, Package_Sequence, &MsgData);
//
//    //��λ��Ч
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


