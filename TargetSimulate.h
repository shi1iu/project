/*********************************************************************************************************************

编写人员：刘耀乐
编写日期：2024-5-25
版本：V2.0
主要功能：雷达主控到目标模拟器子系统协议
文件说明：封包（打包）协议，用于PC传给路由  (测试）

**********************************************************************************************************************/
#pragma pack(1)
#include <stdint.h>
#include "BasicProtocol.h"

#ifndef NULL
#define NULL ((void *)0)
#endif
#define bytes_to_bits                 8 //字节转比特
#define Msg_alignment                 512 //结构体下发 对齐规则
#define round_trip                    2//双程
#define Speed_Quantization_factor    (1 << 1) //速度量化因子(量化精度）
#define Distance_Quantization_factor (1 << 2) //距离量化因子(量化精度）
#define Gain_Quantization_factor     (1 << 16) //增益量化因子(量化精度）


#define READ_SIZE                       0x10000000
#define WRITE_SIZE                      sizeof(TargetSimulate_t)
#define WRITE_SIZE_Header               64
#define WRITE_SIZE_ControlSignals       64 //通用控制信号数据量
#define WRITE_SIZE_ChannelControlParams 64 //通道控制参数配置
#define WRITE_SIZE_Environment          64 //环境参数配置数据量
#define Write_Size_Header_Param         sizeof(TargetSimulate_Header_t) //下发消息头数据量
#define WRITE_SIZE_param                sizeof(TargetSimulate_t)//下发消息体数据量
#define TotalSize                       (10*(WRITE_SIZE_Header+WRITE_SIZE)) //10ms的数据量
#define TotalSize_param                 sizeof(Total_TargetSimulate_t) //下发参数的总数据量


#define SRC_DST_ID_UC   0x30000000	   //上位机
#define SRC_DST_ID_IFB  0x30000001	   //接口板
#define SRC_DST_ID_CLKB 0x30000002	   //时钟板
#define SRC_DST_ID_SPB0 0x30000010	   //信号处理板0
#define SRC_DST_ID_SPB1 0x30000011	   //信号处理板1
#define SRC_DST_ID_SPB2 0x30000012	   //信号处理板2
#define SRC_DST_ID_SPB3 0x30000013	   //信号处理板3
#define SRC_DST_ID_SPB4 0x30000014	   //信号处理板4
#define SRC_DST_ID_BC   0xFFFFFFFF	   //广播



//消息主题格式
typedef struct {
    uint8_t messageType;   // 消息类型（1字节）
    uint8_t messageCategory; // 消息类别（1字节）
    uint8_t messageID_First;    // 消息ID（2字节），大端序  一级ID
    uint8_t messageID_Second;   //二级ID
} MessageHeader;

//ICD版本号格式
typedef struct {
    uint8_t ICD_version_X;        // ICD版本号的X位
    uint8_t ICD_version_Y;        // ICD版本号的Y位
    uint8_t ICD_version_Z;        // ICD版本号的Z位
} Message_ICD;

//消息头格式 64B
typedef struct {
    uint32_t Msg_FrameHeader;               // 帧头，固定值0x55555555
    uint32_t Msg_Length;                    // 消息长度，单位为字节
    uint32_t Msg_ID_Send;                   // 源逻辑ID
    uint32_t Msg_ID_Recv;                   // 目的逻辑ID
    uint32_t Msg_Topic;                     // 消息主题
    uint32_t Msg_SeqNumber;                 // 消息发送序号
    uint64_t Msg_EffectiveTime;             // 仿真生效时刻，微秒时间戳
    uint64_t Msg_IntervalTime;              // 消息发送时间，ms
    uint32_t Msg_FileLength;                // 文件长度
    Message_ICD Msg_ICDVersion;             // ICD版本号
    uint8_t  Msg_Reserve;                   //保留位
    uint32_t Msg_CRC_value;                 // CRC校验值
    uint8_t add[12];                        // 填充
}TargetSimulate_Header_t;


//消息体格式 896B
typedef struct {
    uint32_t En_1; // 工作使能

    uint32_t Point_Speed[TARGETS_PER_CHANNEL]; // 点目标速度
    uint32_t Point_Distance[TARGETS_PER_CHANNEL]; // 点目标距离
    uint16_t Point_Gain_points[TARGETS_PER_CHANNEL]; // 点目标增益
    uint8_t  Point_SwerlingMode[TARGETS_PER_CHANNEL]; // 点目标Swerling模式
    uint16_t Point_PulseInterval[TARGETS_PER_CHANNEL]; // 点目标Swerling1或者3模式下增益变动的脉冲间隔

    uint32_t Scatter_Speed[TARGETS_PER_CHANNEL]; // 散射点目标速度
    uint32_t Scatter_Distance[TARGETS_PER_CHANNEL]; // 散射点目标距离
    uint16_t Scatter_Gain_points[TARGETS_PER_CHANNEL]; // 散射点目标增益
    uint8_t  Scatter_SwerlingMode[TARGETS_PER_CHANNEL]; // 散射点目标Swerling模式
    uint16_t Scatter_PulseInterval[TARGETS_PER_CHANNEL]; // 散射点目标Swerling1或者3模式下增益变动的脉冲间隔

    uint16_t fifo_delay[7]; //  fifo延迟值
    uint8_t fifoenable[3]; // 有效输出 fifo号
    uint8_t add[43]; // 保留
}TargetSimulate_t;

//定义环境参数配置消息
typedef struct {
    uint16_t setFrequencyMode; // 置频方式：0表示手动置频，1表示测频置频
    uint16_t manualFrequency; // 手动置频的射频频点（MHz） 宽带限定模式的测频门限（MHz）
    uint16_t intermediateFrequencyOffset; // 中频频偏（MHz）
    uint16_t receiveRfAttenuation; // 接收射频衰减（dB）
    uint16_t receiveIntermediateFrequencyAttenuation; // 接收中频衰减（dB）
    uint64_t rfSignalCenterFrequency; // 射频信号中心频点（Hz）
    uint16_t frequencyMeasurementMode; // 测频模式：01表示宽带限定，10表示连续模式
    uint16_t frequencyInterval; // 频率间隔值
    uint16_t operationalMode; // 工作模式，包括正常模式、Tx中频校准、Rx中频校准、发射1、2通道自检、Rx射频诊断
    // bit0：Rx射频诊断
    // bit1：通道2自检
    // bit2：通道1自检
    // bit3：Rx中频校准
    // bit4：Tx中频校准
    // bit5：工作正常模式
    uint16_t transmitChannelPowerAmplifierControl; // 发射通道功放控制
    uint16_t add[19];// 保留
}EnvironmentConfig;

// 定义通用控制信号结构体
typedef struct {
    uint32_t update_flag;       // 更新标识，0到1表示更新
    uint32_t start_end_flag;    // 开始/结束标志，1开始，0结束
    uint32_t soft_reset;        // 软复位，1复位，0结束复位
    uint16_t signal_threshold;  // 信号检测门限
    uint16_t add[25];      // 保留
}ControlSignals;

// 定义通道控制参数结构体
typedef struct {
    uint32_t channel_enable; // 通道使能，bit0：通道0使能，bit1：通道1使能
    uint32_t channel0_analog_attenuation; // 通道0模拟衰减，单位1dB，范围0~90
    uint16_t channel0_digital_I_amplitude_coefficient; // 通道0数字I路幅相系数
    uint16_t channel0_digital_Q_amplitude_coefficient; // 通道0数字Q路幅相系数
    uint32_t channel1_analog_attenuation; // 通道1模拟衰减，单位1dB，范围0~90
    uint16_t channel1_digital_I_amplitude_coefficient; // 通道1数字I路幅相系数
    uint16_t channel1_digital_Q_amplitude_coefficient; // 通道1数字Q路幅相系数
    uint32_t add[11]; // 保留
}ChannelControlParams;


typedef struct {
    //环境参数配置消息数据
    TargetSimulate_Header_t EnvironmentConfig_FPGA_t;
    EnvironmentConfig EnvironmentConfig_FPGA;
    //通用控制信号数据
    TargetSimulate_Header_t ControlSignals_FPGA_t1;
    ControlSignals ControlSignals_FPGA1;

    TargetSimulate_Header_t ControlSignals_FPGA_t0;
    ControlSignals ControlSignals_FPGA0;
    //通道控制参数数据
    TargetSimulate_Header_t  ChannelControlParams_FPGA_t;
    ChannelControlParams  ChannelControlParams_FPGA;
    //目标模拟参数数据
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
} ParameterType;//参数赋值使用
