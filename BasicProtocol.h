#pragma once
/*********************************************************************************************************************

编写人员：刘耀乐
编写日期：2024-5-6
版本：V2.0
主要功能：雷达主控到目标模拟器子系统协议
文件说明：

**********************************************************************************************************************/
#pragma pack(1)

#define CHANNELS 1
#define Target_timeinterval 10 //10ms目标时间监测
#define TARGETS_PER_CHANNEL 32 //每个通道的最大目标
#define TARGETS_PER_SCATTER 8  //一个散射点目标中的点目标个数
#define MAX_SCATTER 4          //散射点目标最大个数


//固定/实时目标参数
struct Constant_Parameters {
	unsigned int ChannelNumber ;//通道号 1-10
	unsigned int ChannelType ;//0：点目标 1：多散射点目标
	unsigned int Target_SwerlingModel ;//目标Swerling模型 0~4
	unsigned int Target_SwerlingModelNumber ;//目标模型号
	double Target_Position[3] ;//目标坐标位置
	double Target_Speed[3] ;//目标速度
	double Target_Acceleration[3] ;//目标加速度
	double  Positions_points[Target_timeinterval][3];//每个目标 10ms的位置 (保存变量)
	double  Speeds_points[Target_timeinterval][3];//每个目标 10ms的速度 (保存变量)
};
//点目标通道数据参数
struct ChannelData{
	int channel_id;//通道号
	int target_sum;//目标个数
	struct Constant_Parameters cp[TARGETS_PER_CHANNEL];
};
//多散射点目标通道数据参数
struct ScatterChannelData{
	int channel_id;
	int target_sum;
	struct Constant_Parameters cp[MAX_SCATTER][TARGETS_PER_SCATTER];
	
};
//每ms最小距离参数
struct ms_min_distance {
	float min_distances[Target_timeinterval][MAX_SCATTER];
};
//总参数
typedef struct {
	unsigned int Totaltargets; //目标总数
	unsigned int TotalChannels; //通道总数
}Total_Parameters;
