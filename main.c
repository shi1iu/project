#define _POSIX_C_SOURCE 199309L
#include "BasicProtocol.h"
#include "TargetSimulate.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <winsock.h>
#include <winsock2.h>
#include <math.h>
#include <sys/time.h>
#include <pthread.h>
#include <sched.h>
#include <semaphore.h>
#include "PCIeCard.h"

#define PORT 1234
#define TARGETS 8
#define Max_Target 4 //目标数量

struct ms_min_distance ms_min_dis[CHANNELS];
Total_TargetSimulate_t Channel_TargetSimulate; //目标模拟 消息体+消息头
//TargetSimulate_t Package_target[Target_timeinterval]; //目标模拟  消息体 
//TargetSimulate_Header_t Package_target_t[Target_timeinterval]; //目标模拟 消息头 
sem_t sem;
sem_t sem1;
pthread_mutex_t mutex;
pthread_mutex_t mutex1;
//PCIe
int Status;
int DeviceNum;
char* WriteBuffer_ptr;
char* ReadBuffer_ptr;
PCIeDeviceHandle* PCIeCardHandle_ptr;
char PCIeVersion[256];
//下发参数
int offset = 0; // 用于跟踪combinedData_ptr中的当前位置
int Package_Sequence = 0;//每发一帧（消息头+消息体）计数加1
char* combinedData_ptr;//下发参数指针

void *Scattercalculate_func(void* argument);
void* calculate_thread(void* argument);
void find_min_distances(struct ScatterChannelData* channeldata, int k, struct ms_min_distance ranges[]);
void sort_rows(struct ScatterChannelData* channeldata, int k, struct ms_min_distance ranges[]);
void convertScatterToChannel(struct ScatterChannelData scatterData[], struct ChannelData channelData[]);
void enable_target_simulation();
size_t CalcResultPackage(int IntervalNum, void* CalcResultPtr);
uint64_t TargetSimulate_MsgPackage(ParameterType type, void* MsgBodyPtr, uint64_t MsgBodySize, uint8_t DstID, char** MsgData);

int main(int argc,int *argv[]) {
    struct timespec start, end;
    double interval;
      // 初始化随机数发生器
    srand(time(NULL));

    struct ChannelData channel_data[CHANNELS];
    pthread_t Channel_threads[CHANNELS];
    pthread_t ScatterChannel_threads[CHANNELS];
    pthread_attr_t attr;
    struct sched_param param;
    int recv_result;
    pthread_attr_t attr1;
    struct sched_param param1;
    //初始化下发参数指针
    combinedData_ptr = (char*)PCIeCard_Malloc(TotalSize_param * CHANNELS);
    //printf("TotalSize_param:%d\n", TotalSize_param);
    if (combinedData_ptr == NULL) {
        return -1;
    }

	//PCIe初始化
	Status = -1;
	//PCIe库使用步骤如下
	/*1.获取Lib版本号(非必要) */
	PCIeCard_GetLibVersion(PCIeVersion);
	printf("PCIeCard Lib Version:%s\n", PCIeVersion);

	/*2.获取插入的PCIe设备数量(设备管理器中会显示个数，非必要)*/
	DeviceNum = PCIeCard_GetDeviceNum();
	printf("Devices found: %d\n", DeviceNum);
	if (DeviceNum < 1)
	{
		return -1;
	}

	//3.根据设备数量创建设备句柄(必要，也可直接定义PCIeDeviceHandle PCIeCardHandle)*/
	for (int i = 0; i < DeviceNum; i++)
	{
		PCIeCardHandle_ptr = (PCIeDeviceHandle*)malloc(sizeof(PCIeDeviceHandle) * DeviceNum);
		if (PCIeCardHandle_ptr == 0)
			printf("Error:PCIeCardHandle_ptr malloc\n");
	}

	/*4.以下两种方式打开设备并获取句柄(必要)*/
	PCIeCard_Status_t PCIeCard_Status;
	PCIeDevicePos_t PCIeCardPos;
	//(1)根据id号打开设备，获取设备句柄，单PCIe设备可使用该函数
	for (int i = 0; i < DeviceNum; i++)
	{
		PCIeCardHandle_ptr[i] = PCIeCard_OpenDeviceWithIdx(i);
		if (PCIeCardHandle_ptr[i] == NULL)	//判断是否已打开设备
			return -1;
		//获取设备插槽位置，当多板卡插入时，用于区分板卡
		PCIeCard_GetPosition(i, &PCIeCardPos);
		printf("Device %d: Bus %u, Slot:%u, Func:%u, Dev:%u\n", i, PCIeCardPos.Bus, PCIeCardPos.Slot, PCIeCardPos.Func, PCIeCardPos.Dev);

		//判断设备状态
		PCIeCard_GetStatus(PCIeCardHandle_ptr[i]);
		PCIeCard_Status = PCIeCard_GetStatus(PCIeCardHandle_ptr[i]);
		printf("FPGA_DriverVersion:%u\n", PCIeCard_Status.FPGA_DriverVersion);
		printf("Aurora_LinkStatus:%u\n", PCIeCard_Status.Aurora_LinkStatus);

	}


    //散射点目标计算
    int sockfd, newsockfd;
    socklen_t clilen;
    struct sockaddr_in serv_addr, cli_addr;


    struct ScatterChannelData Scatterchannel_data[CHANNELS];
    float distances[TARGETS_PER_SCATTER];
    float min_distances[CHANNELS][Max_Target];
    int i;


    // 创建socket
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);//TCP
    if (sockfd < 0) {
        perror("ERROR opening socket");
        exit(1);
    }

    // 绑定socket到地址
    memset((char*)&serv_addr, 0, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_addr.s_addr = INADDR_ANY;
    serv_addr.sin_port = htons(PORT);

    if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
        perror("ERROR on binding");
        exit(1);
    }

    // 监听连接
    listen(sockfd, 5);
    clilen = sizeof(cli_addr);

    int buffer_size = 1024 * 1024 * 64; // 您想要设置的缓冲区大小，例如64KB
    socklen_t optlen = sizeof(buffer_size);

    // 设置发送缓冲区大小
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &buffer_size, optlen) == -1) {
        perror("ERROR setsockopt");
    }
    struct timeval timeout;
    timeout.tv_sec = 10;  // 超时时间（秒）
    timeout.tv_usec = 0; // 超时时间（微秒）

    // 设置接收超时
    if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, (char*)&timeout, sizeof(timeout)) < 0) {
        perror("setsockopt failed\n");
    }
    int opt = 1;
    if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) == -1) {
        perror("setsockopt multi-port failed\n");
    }

    // 接受连接
	while (1)
	{
		newsockfd = accept(sockfd, (struct sockaddr*) & cli_addr, &clilen);
		if (newsockfd < 0) {
			perror("ERROR on accept");
			//exit(1);
		}
		else
		{
			break;
		}
	}

    clock_gettime(CLOCK_MONOTONIC, &start);//建立连接后开始计时

    // 接收数据
    //read(newsockfd, Scatterchannel_data, sizeof(Scatterchannel_data));
    for (int i = 0; i < 10; i++) 
	{
        if (recv_result = recv(newsockfd, &Scatterchannel_data[i], sizeof(struct ScatterChannelData), 0) < 0)
            printf("recv failed");
        else
            //printf("recv num:%d\n",i);
            continue;
    }



    //// 打印排序后的结果
    //for (int k = 0; k < CHANNELS; k++) {
    //    printf("第%d通道\n", k + 1);
    //    for (int i = 0; i < MAX_SCATTER; i++) {
    //        for (int j = 0; j < TARGETS_PER_SCATTER; j++) {
    //            printf("Target(%f, %f, %f) \n", Scatterchannel_data[k].cp[i][j].Target_Position[0], Scatterchannel_data[k].cp[i][j].Target_Position[1], Scatterchannel_data[k].cp[i][j].Target_Position[2]);
    //        }
    //        printf("************************");
    //        printf("\n");
    //    }
    //}
    ////打印每个通道内散射点目标的最短距离
    //for (int k = 0; k < CHANNELS; k++) {
    //    printf("第%d通道\n", k + 1);
    //    printf("min_distances(%f, %f,%f, %f)\n ", min_distances[k][0], min_distances[k][1], min_distances[k][2], min_distances[k][3]);
    //}
    
    //转换
    convertScatterToChannel(Scatterchannel_data, channel_data);



    int pthread_result;
    pthread_mutex_init(&mutex, NULL);
    //点目标线程属性
    pthread_attr_init(&attr);
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
    //散射点目标线程属性
    pthread_mutex_init(&mutex1, NULL);
    pthread_attr_init(&attr1);
    pthread_attr_setschedpolicy(&attr1, SCHED_FIFO);
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    pthread_attr_setschedparam(&attr1, &param);
    pthread_attr_setdetachstate(&attr1, PTHREAD_CREATE_JOINABLE);

    // 初始化信号量，最后一个参数设置信号量的值
    sem_init(&sem, 0, 1);
    //sem_init(&sem1,0, 1);
    // 初始化通道数据并创建线程
    int Channel_Status;
    for (int i = 0; i < CHANNELS; ++i) 
	{
        channel_data[i].channel_id = i + 1;
        channel_data[i].target_sum = TARGETS_PER_CHANNEL;
        Scatterchannel_data[i].channel_id = i + 1;
        Scatterchannel_data[i].target_sum = TARGETS_PER_CHANNEL;
        // 创建点目标线程
        if (pthread_result = pthread_create(&Channel_threads[i], NULL, (void*)calculate_thread, (void*)&channel_data[i])) //计算线程
		{
            fprintf(stderr, "Error creating thread\n");
            return 1;
        }

        //// 创建散射点目标线程
        //if (pthread_result=pthread_create(&ScatterChannel_threads[i], NULL, Scattercalculate_func, (void*)&Scatterchannel_data[i])) {
        //    fprintf(stderr, "Error creating Scatter_thread\n");
        //    return 1;
        //}
       
        //等待运算完毕，释放信号量，使得线程可以继续执行
        sem_post(&sem);
        
        //enable_target_simulation();
       
    }
    for (int i = 0; i < CHANNELS; ++i) {
        if (Channel_Status = pthread_join(Channel_threads[i], NULL)) {
            fprintf(stderr, "Error joining thread\n");
            return 2;
        }
       /* if (Channel_Status = pthread_join(ScatterChannel_threads[i], NULL)) {
            fprintf(stderr, "Error joining Scatter_thread\n");
            return 2;
        }*/
    }
	//打包并发送控制信号
	char* MsgData = NULL;
	size_t MsgSize;
	uint64_t WriteSize;
	//环境参数配置消息打包
	EnvironmentConfig EnvironmentConfig_FPGA;
	EnvironmentConfig_FPGA.setFrequencyMode = 0;
	EnvironmentConfig_FPGA.manualFrequency = 0xfff;
	EnvironmentConfig_FPGA.intermediateFrequencyOffset = 0x30;
	EnvironmentConfig_FPGA.receiveRfAttenuation = 0x11;
	EnvironmentConfig_FPGA.receiveIntermediateFrequencyAttenuation = 0x12;
	EnvironmentConfig_FPGA.rfSignalCenterFrequency = 0xfff;
	EnvironmentConfig_FPGA.frequencyMeasurementMode = 0x2;
	EnvironmentConfig_FPGA.frequencyInterval = 0x666;
	EnvironmentConfig_FPGA.operationalMode = 0x10;
	EnvironmentConfig_FPGA.transmitChannelPowerAmplifierControl = 0x2;

	MsgSize = TargetSimulate_MsgPackage(Environment_SET, &EnvironmentConfig_FPGA, sizeof(EnvironmentConfig), SRC_DST_ID_SPB2, &MsgData);
	memcpy(combinedData_ptr, MsgData, MsgSize);
	free(MsgData);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);

	//发送复位信号
	MsgSize = TargetSimulate_MsgPackage(ControlSignals_ResetAssert, NULL, 0, SRC_DST_ID_SPB2 ,&MsgData);
	memcpy(combinedData_ptr, MsgData, MsgSize);
	free(MsgData);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);
	usleep(1000);
	//复位无效
	MsgSize = TargetSimulate_MsgPackage(ControlSignals_ResetDeAssert, NULL, 0, SRC_DST_ID_SPB2 ,&MsgData);
	memcpy(combinedData_ptr, MsgData, MsgSize);
	free(MsgData);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);
	usleep(1000);
	//通道参数
	ChannelControlParams ChannelControlParams_FPGA;
	ChannelControlParams_FPGA.channel_enable = 0x1;
	ChannelControlParams_FPGA.channel0_analog_attenuation = 0x1;
	ChannelControlParams_FPGA.channel0_digital_I_amplitude_coefficient = 0x1;
	ChannelControlParams_FPGA.channel0_digital_Q_amplitude_coefficient = 0x1;
	ChannelControlParams_FPGA.channel1_analog_attenuation = 0x1;
	ChannelControlParams_FPGA.channel1_digital_I_amplitude_coefficient = 0x1;
	ChannelControlParams_FPGA.channel1_digital_Q_amplitude_coefficient = 0x1;
	
	MsgSize = TargetSimulate_MsgPackage(ChannelControlParams_SET, &ChannelControlParams_FPGA, sizeof(ChannelControlParams), SRC_DST_ID_SPB2, &MsgData);
	memcpy(combinedData_ptr, MsgData, MsgSize);
	free(MsgData);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);


	//推算参数封包(按通道）
	MsgSize = CalcResultPackage(10, combinedData_ptr);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);


	//开始信号
	MsgSize = TargetSimulate_MsgPackage(ControlSignals_Start, NULL, 0, SRC_DST_ID_SPB2, &MsgData);
	memcpy(combinedData_ptr, MsgData, MsgSize);
	free(MsgData);
	WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);

	MsgSize = CalcResultPackage(10, combinedData_ptr);
	while (1)
	{
		usleep(8000);
		//推算参数封包(按通道）
		WriteSize = PCIeCard_Write(PCIeCardHandle_ptr[0], combinedData_ptr, MsgSize);
		
	}
	


    printf("All threads completed successfully\n");
    pthread_mutex_destroy(&mutex);
    pthread_mutex_destroy(&mutex1);
    // 销毁线程属性对象
    pthread_attr_destroy(&attr);
    pthread_attr_destroy(&attr1);
    // 销毁信号量
    sem_destroy(&sem);
    //sem_destroy(&sem1);

    // 计算时间间隔
    clock_gettime(CLOCK_MONOTONIC, &end);
    interval = (double)(end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1E9; // 秒数
    interval *= 1000; // 毫秒

    printf("代码执行时间：%f 毫秒\n", interval);

    /*8.释放内存空间(必要)*/
    PCIeCard_Free(WriteBuffer_ptr);
    PCIeCard_Free(ReadBuffer_ptr);
    PCIeCard_Free(combinedData_ptr);

    /*9.关闭PCIe设备句柄(必要)*/
    for (int i = 0; i < DeviceNum; i++)
    {
        PCIeCard_CloseDevice(PCIeCardHandle_ptr[i]);
    }
    /*10.释放句柄空间(必要)*/
    free(PCIeCardHandle_ptr);

    //关闭socket
    close(newsockfd);
    close(sockfd);

    return 0;
}
