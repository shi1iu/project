//由于目标位置下发的协议还未定，现在的位置都是随机生成的（2024.4.19)
//目标增益计算（场强计算）
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "BasicProtocol.h"
#include "TargetSimulate.h"
#include <pthread.h>
#include <semaphore.h>
#define DT 1 // 时间步长，单位ms
#define  M_PI 3.14

extern pthread_mutex_t mutex;
extern pthread_mutex_t mutex1;
extern sem_t sem1;
extern struct ms_min_distance ms_min_dis[CHANNELS];
//接收功率计算，初始化参数
const double Pt = 10; // 发射功率 W 0.01
const double G = 10; // 天线增益 dB
const double lambda = 10; // 波长 m  1/18GHZ 0.0167

float channelReceivedPower[CHANNELS];                               // 初始化通道接收功率变量
float g_Distance_points[TARGETS_PER_CHANNEL][Target_timeinterval];  //每1ms目标距离雷达的距离 (点目标)
float g_Distance_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];//每1ms目标距离雷达的距离 (散射点目标)
float g_Speed_points[TARGETS_PER_CHANNEL][Target_timeinterval];     //每1ms目标距离雷达的距离 (点目标)
float g_Speed_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];   //每1ms目标距离雷达的距离 (散射点目标)
float g_Gain_points[TARGETS_PER_CHANNEL][Target_timeinterval];      //存第x个目标，第y+1 ms的平均功率增益(点目标)
float g_Gain_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];    //存第x个目标，第y+1 ms的平均功率增益(散射点目标)
float g_Target_SwerlingModel_points[TARGETS_PER_CHANNEL][Target_timeinterval];//Swerling模型(点目标)
float g_Target_SwerlingModel_scatters[TARGETS_PER_CHANNEL][Target_timeinterval];//Swerling模型(散射点目标)

struct ChannelData* g_Recv_channel_data=NULL;//接收散射点目标排序完后的参数


double randi(int m, int n) { // 生成一个在[m, n]范围内的随机整数
    int num;
    num = rand() % (n - m + 1) + m;
    return num;
}
double calculateR(double positions[], int length) {//三维坐标下的距离计算  以及 速度矢量计算(一维数组）
    double sum = 0;
    for (int i = 0; i < length; i++) {
        sum += pow(positions[i], 2);
    }
    
    return sqrt(sum);
}
int calculate_distanceInt(double positions[][3], int ms) {// 每1ms位置的距离计算  （二维数组）
    int sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += pow(positions[ms][i], 2);
    }
    return sqrt(sum);
}
float Pr_yMS(float Target_Pr[][10], int y) {//确定单通道多目标在 y ms的接收频率
    float channelReceivedPower = 0;
    for (int x = 0; x < TARGETS_PER_CHANNEL; x++) {
        channelReceivedPower = (float)channelReceivedPower + (float)Target_Pr[x][y];
    }
    return channelReceivedPower;
}
double calculateAverage(int arr[], int size) {// 函数用于计算数组的平均数
    int sum = 0;
    for (int i = 0; i < size; i++) {
        sum += arr[i];
    }
    return (double)sum / size;
}

void* calculate_thread(void* argument) {
    pthread_mutex_lock(&mutex);
    struct ChannelData* channel_data = (struct ChannelData*)argument;//将通道的结构体指针传过来

    double sigma[TARGETS_PER_CHANNEL]; // 每个目标雷达目标截面积 m^2
    double Aver_sigma;
    int t = 0; // 时间，单位ms
    float Positions_points[Target_timeinterval][3]; // 每个目标10个位置点

    float Target_Pr[TARGETS_PER_CHANNEL][Target_timeinterval]; //存第 x 个目标，第 y+1 ms的Pr
    
    struct Constant_Parameters cp[TARGETS_PER_CHANNEL];
    //printf("*************************************************\n");
    //printf("第%d个通道:\n", channel_data->channel_id);



    for (unsigned int target = 0; target < channel_data->target_sum; target++) {
        t = 0;
        //printf("********第%d个目标********:\n", target + 1);
        // 初始化目标参数
        channel_data->cp[target].ChannelNumber = channel_data->channel_id;
        channel_data->cp[target].ChannelType = randi(1, 2);
        channel_data->cp[target].Target_SwerlingModel = randi(0, 4);
        channel_data->cp[target].Target_SwerlingModelNumber = randi(1, 10);
        //sigma[target] = randi(1, 10);
        sigma[target] = 10;

        /*for (int i = 0; i < 3; i++) {
            channel_data->cp[target].Target_Position[i] = randi(1, 5);
            channel_data->cp[target].Target_Speed[i] = randi(1, 5);
            channel_data->cp[target].Target_Acceleration[i] = randi(1, 5);
        }*/

        // 模拟运动,一个目标每1ms的位置变化
        for (int j = 0; j < Target_timeinterval; j++) {
            for (int k = 0; k < 3; k++) {//3代表xyz
                channel_data->cp[target].Target_Position[k] = channel_data->cp[target].Target_Position[k] + channel_data->cp[target].Target_Speed[k] * (DT / 1000.0) + 0.5 * channel_data->cp[target].Target_Acceleration[k] * (DT / 1000.0) * (DT / 1000.0);
                channel_data->cp[target].Target_Speed[k] = channel_data->cp[target].Target_Speed[k] + channel_data->cp[target].Target_Acceleration[k] * (DT / 1000.0);
                channel_data->cp[target].Positions_points[j][k] = channel_data->cp[target].Target_Position[k];//每1ms目标的位置
                g_Distance_points[target][j] = calculateR(channel_data->cp[target].Target_Position, 3);
                g_Speed_points[target][j] = calculateR(channel_data->cp[target].Target_Speed, 3);
            }
            g_Target_SwerlingModel_points[target][j] = channel_data->cp[target].Target_SwerlingModel;
            t += DT;
            Target_Pr[target][j] = (double)((Pt * G * G * lambda * lambda * sigma[target]) / (pow((4 * M_PI), 3) * pow(g_Distance_points[target][j], 4)));

           //printf("第%d通道第%d毫秒位置: (%f, %f, %f)\n", channel_data->channel_id,t, (float)channel_data->cp[target].Target_Position[0], (float)channel_data->cp[target].Target_Position[1], (float)channel_data->cp[target].Target_Position[2]);
        }
       
    }

    //确定单个通道多目标在每1ms的接收频率
    for (int y = 0; y < Target_timeinterval; y++) {
        channelReceivedPower[y] = Pr_yMS(Target_Pr, y);
        if (isinf(channelReceivedPower[y])) {
            //printf("channelReceivedPower is infinite due to overflow.\n");
        }
        else {
            //printf("At %d ms, Total Received Power for Channel %d: %lf\n", y + 1, channel_data->channel_id , channelReceivedPower[y]);
        }
    }
    //每个目标的平均功率增益
    for (unsigned int target = 0; target < channel_data->target_sum; target++) {
        //printf("********第%d个目标********:\n", target + 1);
        for (int j = 0; j < Target_timeinterval; j++) {
            g_Gain_points[target][j] = Target_Pr[target][j] / channelReceivedPower[j]; //第x个目标，第j+1 ms的平均功率增益
            g_Gain_points[target][j] = sqrt(g_Gain_points[target][j]);
            //printf("第 %d 个目标，第 %d ms的平均功率增益为：%f\n", target+1, j + 1, g_Gain_points[target][j]);
        }
    }
    pthread_mutex_unlock(&mutex);
    return NULL;
}
void* Scattercalculate_func(void* argument) {
    // 等待信号量
    //sem_wait(&sem1);
    pthread_mutex_lock(&mutex1);
    struct ScatterChannelData* channel_data = (void*)argument;//将通道的结构体指针传过来

    //double sigma[TARGETS_PER_CHANNEL] ; // 每个目标雷达目标截面积 m^2
    double sigma;
    double Aver_sigma;
    int t = 0; // 时间，单位ms
    float Positions_points[Target_timeinterval][3]; // 每个目标10个位置点
    float Target_Pr[TARGETS_PER_CHANNEL][Target_timeinterval]; //存第 x 个目标，第 y+1 ms的Pr

    // printf("*************************************************\n");
    //printf("第%d个通道:\n", channel_data->channel_id);
   
    for (int target = 0, row = 0; row < MAX_SCATTER; row++) {
        for (int column = 0; column < TARGETS_PER_SCATTER; column++) {
            target++;
            t = 0;
            //printf("********第%d个目标********:\n", column + row * TARGETS_PER_SCATTER + 1);

            // 初始化目标参数
            channel_data->cp[row][column].ChannelNumber = channel_data->channel_id;
            channel_data->cp[row][column].ChannelType = randi(1, 2);
            channel_data->cp[row][column].Target_SwerlingModel = randi(0, 4);
            channel_data->cp[row][column].Target_SwerlingModelNumber = randi(1, 10);
            sigma = 10;

            // for (int i = 0; i < 3; i++) {
            //     channel_data->cp[row][column].Target_Position[i] = randi(1,5);
            //     channel_data->cp[row][column].Target_Speed[i] = randi(1, 5);
            //     channel_data->cp[row][column].Target_Acceleration[i] = randi(1, 5);
            // }

            // 模拟运动,一个目标10ms位置变化
            for (int j = 0; j < Target_timeinterval; j++) {
                for (int k = 0; k < 3; k++) {
                    channel_data->cp[row][column].Target_Position[k] = channel_data->cp[row][column].Target_Position[k] + channel_data->cp[row][column].Target_Speed[k] * (DT / 1000.0) + 0.5 * channel_data->cp[row][column].Target_Acceleration[k] * (DT / 1000.0) * (DT / 1000.0);
                    channel_data->cp[row][column].Target_Speed[k] = channel_data->cp[row][column].Target_Speed[k] + channel_data->cp[row][column].Target_Acceleration[k] * (DT / 1000.0);
                    channel_data->cp[row][column].Positions_points[j][k] = channel_data->cp[row][column].Target_Position[k];//每1ms目标的位置
                    channel_data->cp[row][column].Speeds_points[j][k] = channel_data->cp[row][column].Target_Speed[k];//每1ms目标的速度                   
                }
                g_Target_SwerlingModel_scatters[target][j] = channel_data->cp[row][column].Target_SwerlingModel;
                t += DT;
                //printf("At %d ms\n", t);
                //printf("Positions_points(%f ,%f ,%f)\n", channel_data->cp[row][column].Positions_points[j][0],channel_data->cp[row][column].Positions_points[j][1],channel_data->cp[row][column].Positions_points[j][2]);
                //printf("Speeds_points(%f ,%f ,%f)\n", channel_data->cp[row][column].Speeds_points[j][0], channel_data->cp[row][column].Speeds_points[j][1], channel_data->cp[row][column].Speeds_points[j][2]);
                //printf("第%d通道第%d毫秒位置: (%f, %f, %f)\n", channel_data->channel_id, t, (float)channel_data->cp[row][column].Target_Position[0], (float)channel_data->cp[row][column].Target_Position[1], (float)channel_data->cp[row][column].Target_Position[2]);
            }
        }

    }
    find_min_distances(channel_data, channel_data->channel_id-1, ms_min_dis);
    sort_rows(channel_data, channel_data->channel_id-1, ms_min_dis);
    //排序完后再给参数赋值
    g_Recv_channel_data = malloc(sizeof(struct ChannelData));
    if (g_Recv_channel_data == NULL) {
        printf("g_Recv_channel_data error\n");
    }
    for (int target = 0, row = 0; row < MAX_SCATTER; row++) {//排序完转换为点目标，方便下发参数
        for (int column = 0; column < TARGETS_PER_SCATTER; column++) {
            if (target < MAX_SCATTER * TARGETS_PER_SCATTER)
            {
                g_Recv_channel_data->cp[target] = channel_data->cp[row][column];
                //printf("g_Recv_channel_data:\n");
                //printf("********第%d个目标********:\n", target + 1);
                //for (int j = 0; j < Target_timeinterval; j++) {               
                //    printf("At %d ms\n", j+1);
                //    printf("Positions_points(%f ,%f ,%f)\n", g_Recv_channel_data->cp[target].Positions_points[j][0], g_Recv_channel_data->cp[target].Positions_points[j][1], g_Recv_channel_data->cp[target].Positions_points[j][2]);
                //    printf("Speeds_points(%f ,%f ,%f)\n", g_Recv_channel_data->cp[target].Speeds_points[j][0], g_Recv_channel_data->cp[target].Speeds_points[j][1], g_Recv_channel_data->cp[target].Speeds_points[j][2]);
                //}
                target++;
            }
            else
            {
                printf("target is out of array\n");
                break;
            }
        }
        
    }
    for (int target = 0; target < TARGETS_PER_CHANNEL; target++)
    {
        for (int j = 0; j < Target_timeinterval; j++) 
        {
            g_Distance_scatters[target][j] = calculate_distanceInt(g_Recv_channel_data->cp[target].Positions_points,j);
            g_Speed_scatters[target][j] = calculate_distanceInt(g_Recv_channel_data->cp[target].Speeds_points,j);
            
            Target_Pr[target][j] = (double)((Pt * G * G * lambda * lambda * sigma) / (pow((4 * M_PI), 3) * pow(g_Distance_scatters[target][j], 4)));//接收频率计算
            //printf("Distance_scatter:(%d)\n", g_Distance_scatters[target][j]);
            //printf("g_Speed_scatters:(%d)\n", g_Speed_scatters[target][j]);
        }
    }
    

    //确定单个通道多目标在每1ms的接收频率
    for (int y = 0; y < Target_timeinterval; y++) {
        channelReceivedPower[y] = Pr_yMS(Target_Pr, y);
        if (isinf(channelReceivedPower[y])) {
            //printf("channelReceivedPower is infinite due to overflow.\n");
        }
        else {
            //printf("At %d ms, Total Received Power for Channel %d: %lf\n", y + 1, channel_data->channel_id, channelReceivedPower[y]);
        }
    }
     //每个目标的平均功率增益
     for (unsigned int target = 0; target < channel_data->target_sum; target++) {
         //printf("********第%d个目标********:\n", target + 1);
         for (int j = 0; j < Target_timeinterval; j++) {
             g_Gain_scatters[target][j] = Target_Pr[target][j] / channelReceivedPower[j]; //第x个目标，第j+1 ms的平均功率增益
             g_Gain_scatters[target][j] = sqrt(g_Gain_scatters[target][j]);
             //printf("第 %d 个目标，第 %d ms的平均功率增益为：%f\n", target+1, j + 1, g_Gain_scatters[target][j]);
         }
     }
    pthread_mutex_unlock(&mutex1);
    // 释放信号量
    //sem_post(&sem1);
    
    return NULL;
}