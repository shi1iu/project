/*********************************************************************************************************************

��д��Ա����ҫ��
��д���ڣ�2024-5-25
�汾��V2.0
��Ҫ���ܣ��״����ص�Ŀ��ģ������ϵͳЭ��
�ļ�˵��������������Э�飬����PC����·��  (���ԣ�

**********************************************************************************************************************/
#pragma pack(1)
#include <stdint.h>
#include "BasicProtocol.h"

#ifndef NULL
#define NULL ((void *)0)
#endif
#define bytes_to_bits                 8 //�ֽ�ת����
#define Msg_alignment                 512 //�ṹ���·� �������
#define round_trip                    2//˫��
#define Speed_Quantization_factor    (1 << 1) //�ٶ���������(�������ȣ�
#define Distance_Quantization_factor (1 << 2) //������������(�������ȣ�
#define Gain_Quantization_factor     (1 << 16) //������������(�������ȣ�


#define READ_SIZE                       0x10000000
#define WRITE_SIZE                      sizeof(TargetSimulate_t)
#define WRITE_SIZE_Header               64
#define WRITE_SIZE_ControlSignals       64 //ͨ�ÿ����ź�������
#define WRITE_SIZE_ChannelControlParams 64 //ͨ�����Ʋ�������
#define WRITE_SIZE_Environment          64 //������������������
#define Write_Size_Header_Param         sizeof(TargetSimulate_Header_t) //�·���Ϣͷ������
#define WRITE_SIZE_param                sizeof(TargetSimulate_t)//�·���Ϣ��������
#define TotalSize                       (10*(WRITE_SIZE_Header+WRITE_SIZE)) //10ms��������
#define TotalSize_param                 sizeof(Total_TargetSimulate_t) //�·���������������


#define SRC_DST_ID_UC   0x30000000	   //��λ��
#define SRC_DST_ID_IFB  0x30000001	   //�ӿڰ�
#define SRC_DST_ID_CLKB 0x30000002	   //ʱ�Ӱ�
#define SRC_DST_ID_SPB0 0x30000010	   //�źŴ����0
#define SRC_DST_ID_SPB1 0x30000011	   //�źŴ����1
#define SRC_DST_ID_SPB2 0x30000012	   //�źŴ����2
#define SRC_DST_ID_SPB3 0x30000013	   //�źŴ����3
#define SRC_DST_ID_SPB4 0x30000014	   //�źŴ����4
#define SRC_DST_ID_BC   0xFFFFFFFF	   //�㲥



//��Ϣ�����ʽ
typedef struct {
    uint8_t messageType;   // ��Ϣ���ͣ�1�ֽڣ�
    uint8_t messageCategory; // ��Ϣ���1�ֽڣ�
    uint8_t messageID_First;    // ��ϢID��2�ֽڣ��������  һ��ID
    uint8_t messageID_Second;   //����ID
} MessageHeader;

//ICD�汾�Ÿ�ʽ
typedef struct {
    uint8_t ICD_version_X;        // ICD�汾�ŵ�Xλ
    uint8_t ICD_version_Y;        // ICD�汾�ŵ�Yλ
    uint8_t ICD_version_Z;        // ICD�汾�ŵ�Zλ
} Message_ICD;

//��Ϣͷ��ʽ 64B
typedef struct {
    uint32_t Msg_FrameHeader;               // ֡ͷ���̶�ֵ0x55555555
    uint32_t Msg_Length;                    // ��Ϣ���ȣ���λΪ�ֽ�
    uint32_t Msg_ID_Send;                   // Դ�߼�ID
    uint32_t Msg_ID_Recv;                   // Ŀ���߼�ID
    uint32_t Msg_Topic;                     // ��Ϣ����
    uint32_t Msg_SeqNumber;                 // ��Ϣ�������
    uint64_t Msg_EffectiveTime;             // ������Чʱ�̣�΢��ʱ���
    uint64_t Msg_IntervalTime;              // ��Ϣ����ʱ�䣬ms
    uint32_t Msg_FileLength;                // �ļ�����
    Message_ICD Msg_ICDVersion;             // ICD�汾��
    uint8_t  Msg_Reserve;                   //����λ
    uint32_t Msg_CRC_value;                 // CRCУ��ֵ
    uint8_t add[12];                        // ���
}TargetSimulate_Header_t;


//��Ϣ���ʽ 896B
typedef struct {
    uint32_t En_1; // ����ʹ��

    uint32_t Point_Speed[TARGETS_PER_CHANNEL]; // ��Ŀ���ٶ�
    uint32_t Point_Distance[TARGETS_PER_CHANNEL]; // ��Ŀ�����
    uint16_t Point_Gain_points[TARGETS_PER_CHANNEL]; // ��Ŀ������
    uint8_t  Point_SwerlingMode[TARGETS_PER_CHANNEL]; // ��Ŀ��Swerlingģʽ
    uint16_t Point_PulseInterval[TARGETS_PER_CHANNEL]; // ��Ŀ��Swerling1����3ģʽ������䶯��������

    uint32_t Scatter_Speed[TARGETS_PER_CHANNEL]; // ɢ���Ŀ���ٶ�
    uint32_t Scatter_Distance[TARGETS_PER_CHANNEL]; // ɢ���Ŀ�����
    uint16_t Scatter_Gain_points[TARGETS_PER_CHANNEL]; // ɢ���Ŀ������
    uint8_t  Scatter_SwerlingMode[TARGETS_PER_CHANNEL]; // ɢ���Ŀ��Swerlingģʽ
    uint16_t Scatter_PulseInterval[TARGETS_PER_CHANNEL]; // ɢ���Ŀ��Swerling1����3ģʽ������䶯��������

    uint16_t fifo_delay[7]; //  fifo�ӳ�ֵ
    uint8_t fifoenable[3]; // ��Ч��� fifo��
    uint8_t add[43]; // ����
}TargetSimulate_t;

//���廷������������Ϣ
typedef struct {
    uint16_t setFrequencyMode; // ��Ƶ��ʽ��0��ʾ�ֶ���Ƶ��1��ʾ��Ƶ��Ƶ
    uint16_t manualFrequency; // �ֶ���Ƶ����ƵƵ�㣨MHz�� ����޶�ģʽ�Ĳ�Ƶ���ޣ�MHz��
    uint16_t intermediateFrequencyOffset; // ��ƵƵƫ��MHz��
    uint16_t receiveRfAttenuation; // ������Ƶ˥����dB��
    uint16_t receiveIntermediateFrequencyAttenuation; // ������Ƶ˥����dB��
    uint64_t rfSignalCenterFrequency; // ��Ƶ�ź�����Ƶ�㣨Hz��
    uint16_t frequencyMeasurementMode; // ��Ƶģʽ��01��ʾ����޶���10��ʾ����ģʽ
    uint16_t frequencyInterval; // Ƶ�ʼ��ֵ
    uint16_t operationalMode; // ����ģʽ����������ģʽ��Tx��ƵУ׼��Rx��ƵУ׼������1��2ͨ���Լ졢Rx��Ƶ���
    // bit0��Rx��Ƶ���
    // bit1��ͨ��2�Լ�
    // bit2��ͨ��1�Լ�
    // bit3��Rx��ƵУ׼
    // bit4��Tx��ƵУ׼
    // bit5����������ģʽ
    uint16_t transmitChannelPowerAmplifierControl; // ����ͨ�����ſ���
    uint16_t add[19];// ����
}EnvironmentConfig;

// ����ͨ�ÿ����źŽṹ��
typedef struct {
    uint32_t update_flag;       // ���±�ʶ��0��1��ʾ����
    uint32_t start_end_flag;    // ��ʼ/������־��1��ʼ��0����
    uint32_t soft_reset;        // ��λ��1��λ��0������λ
    uint16_t signal_threshold;  // �źż������
    uint16_t add[25];      // ����
}ControlSignals;

// ����ͨ�����Ʋ����ṹ��
typedef struct {
    uint32_t channel_enable; // ͨ��ʹ�ܣ�bit0��ͨ��0ʹ�ܣ�bit1��ͨ��1ʹ��
    uint32_t channel0_analog_attenuation; // ͨ��0ģ��˥������λ1dB����Χ0~90
    uint16_t channel0_digital_I_amplitude_coefficient; // ͨ��0����I·����ϵ��
    uint16_t channel0_digital_Q_amplitude_coefficient; // ͨ��0����Q·����ϵ��
    uint32_t channel1_analog_attenuation; // ͨ��1ģ��˥������λ1dB����Χ0~90
    uint16_t channel1_digital_I_amplitude_coefficient; // ͨ��1����I·����ϵ��
    uint16_t channel1_digital_Q_amplitude_coefficient; // ͨ��1����Q·����ϵ��
    uint32_t add[11]; // ����
}ChannelControlParams;


typedef struct {
    //��������������Ϣ����
    TargetSimulate_Header_t EnvironmentConfig_FPGA_t;
    EnvironmentConfig EnvironmentConfig_FPGA;
    //ͨ�ÿ����ź�����
    TargetSimulate_Header_t ControlSignals_FPGA_t1;
    ControlSignals ControlSignals_FPGA1;

    TargetSimulate_Header_t ControlSignals_FPGA_t0;
    ControlSignals ControlSignals_FPGA0;
    //ͨ�����Ʋ�������
    TargetSimulate_Header_t  ChannelControlParams_FPGA_t;
    ChannelControlParams  ChannelControlParams_FPGA;
    //Ŀ��ģ���������
    TargetSimulate_Header_t Package_target_t[Target_timeinterval];
    TargetSimulate_t Package_target[Target_timeinterval];
}Total_TargetSimulate_t;

typedef enum {
    Environment_SET,
	ControlSignals_ResetAssert,
	ControlSignals_ResetDeAssert,
	ControlSignals_Start,
	ControlSignals_Stop,
    ChannelControlParams_SET,
    Package_target_SET,
} ParameterType;//������ֵʹ��
