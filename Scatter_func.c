#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "BasicProtocol.h"
#include "TargetSimulate.h"
#define MAX_DELAY_VALUE 2048 //最大延时值2048，可改2000
#define C_COARSE 3e8//光速_粗略值
#define C_ACCURATE 299792458//光速_精确值
#define FS 1200e6
#define FCLK 300e6//工作时钟
#define FIFO_Num 7 //FIFO个数
#define FIFO_Enable 3 //用于记录计算差分距离延迟值时的当前FIFO号

extern Total_TargetSimulate_t Channel_TargetSimulate;
//extern TargetSimulate_t Package_target[Target_timeinterval];
float Scatter_FIFO[Target_timeinterval];
float Scatter_EnableFIFO[Target_timeinterval];

double calculate_distance(double positions[][3], int ms) {// 每1ms位置的距离计算  （二维数组）
    double sum = 0;
    for (int i = 0; i < 3; i++) {
        sum += pow(positions[ms][i], 2);
    }
    return sqrt(sum);
}
 //找出每一行的最小距离
void find_min_distances(struct ScatterChannelData* channeldata, int k, struct ms_min_distance ranges[]){
    //printf("第%d通道\n", k + 1);
    for (int ms = 0; ms < Target_timeinterval; ms++) {
        for (int i = 0; i < MAX_SCATTER; i++) {
            float min_dist = calculate_distance(channeldata->cp[i][0].Positions_points, ms);
            //printf("第%d个散射点目标第%d毫秒第%d个散射点的距离为:%f\n",i+1,ms+1,1,min_dist);
            for (int j = 1; j < TARGETS_PER_SCATTER; j++) {
                float dist = calculate_distance(channeldata->cp[i][j].Positions_points, ms);
                //printf("第%d个散射点目标第%d毫秒第%d个散射点的距离为:%f\n",i+1,ms+1,j+1,dist);
                if (dist < min_dist) {
                    min_dist = dist;
                }
            }
            ranges[k].min_distances[ms][i] = min_dist;
            // printf("The %d Target,at %d ms,the shortest distance is: %f\n",i+1,ms+1, ranges[k].min_distances[ms][i]);

        }
       // printf("**********************************\n");
    }
}
// 根据最小距离升序排序行
void sort_rows(struct ScatterChannelData* channeldata, int k, struct ms_min_distance ranges[]) {//传递二维数组的行过来
    //printf("第%d通道\n",k+1);
    for (int ms = 0; ms < Target_timeinterval; ms++) {
        for (int i = 0; i < MAX_SCATTER - 1; i++) {
            for (int j = i + 1; j < MAX_SCATTER; j++) {
                if (ranges[k].min_distances[ms][i] > ranges[k].min_distances[ms][j]) {
                    // 交换最小距离
                    float temp_dist = ranges[k].min_distances[ms][i];
                    ranges[k].min_distances[ms][i] = ranges[k].min_distances[ms][j];
                    ranges[k].min_distances[ms][j] = temp_dist;
                    // 交换行
                    struct Constant_Parameters temp_targets;
                    for (int f = 0; f < TARGETS_PER_SCATTER; f++) {
                        temp_targets = channeldata->cp[i][f];
                        channeldata->cp[i][f] = channeldata->cp[j][f];
                        channeldata->cp[j][f] = temp_targets;
                    }
                }
            }
        }
        //printf("min_distances(%f, %f,%f, %f)\n ", ranges[k].min_distances[ms][0], ranges[k].min_distances[ms][1], ranges[k].min_distances[ms][2], ranges[k].min_distances[ms][3]);
        int delay0;
        int fifo_delay[7];
        int NO[3];

        // 调用函数
        clc_scattering(ranges[k].min_distances[ms][0], ranges[k].min_distances[ms][1], ranges[k].min_distances[ms][2], ranges[k].min_distances[ms][3], &delay0, fifo_delay, NO);
        
        //// 打印结果
        //printf("delay0: %d\n", delay0);
        for (int i = 0; i < FIFO_Num; i++) {
            Channel_TargetSimulate.Package_target[ms].fifo_delay[i] = (int)fifo_delay[i];
            //printf("fifo_delay[%d]: %d\n", i, fifo_delay[i]);
        }
        for (int i = 0; i < FIFO_Enable; i++) {
            Channel_TargetSimulate.Package_target[ms].fifoenable[i] = (int)NO[i];
            //printf("NO[%d]: %d\n", i, NO[i]);
        }
    }
    
     
     /*for (int ms = 0; ms < Target_timeinterval; ms++) {
         printf("***********%d ms*************\n",ms+1);
         for (int i = 0; i < MAX_SCATTER; i++) {
             for (int j = 0; j < TARGETS_PER_SCATTER; j++) {
                 printf("第%d个目标第%dms,Target(%f, %f, %f) \n",i*TARGETS_PER_SCATTER+j+1,ms+1,channeldata->cp[i][j].Positions_points[ms][0],  channeldata->cp[i][j].Positions_points[ms][1],  channeldata->cp[i][j].Positions_points[ms][2]);
             }
             printf("************************");
             printf("\n");
         }
     }*/

}
// 转换scatterData至channelData
void convertScatterToChannel(struct ScatterChannelData scatterData[], struct ChannelData channelData[]) {
    for (int i = 0; i < CHANNELS; i++) {
        channelData[i].channel_id = scatterData[i].channel_id;
        channelData[i].target_sum = scatterData[i].target_sum;
        int row = 0;//二维数组的行
        int column = 0;//列
        for (int j = 0; j < TARGETS_PER_CHANNEL; j++) {
            channelData[i].cp[j] = scatterData[i].cp[row][column];
            column++;
            if ((j + 1) % 8 == 0)
            {
                row++;
                column = 0;
                if (row == MAX_SCATTER)
                    break;
            }

        }
    }
}
//计算FIFO的延迟值
void clc_scattering(double range0, double range1, double range2, double range3, int* delay0, int fifo_delay[7], int NO[3]) {
    int diff0 = range1 - range0;
    int diff1 = range2 - range1;
    int diff2 = range3 - range2;

    *delay0 = floor(range0 / C_ACCURATE * FCLK);
    int delay_diff0 = floor(diff0 / C_COARSE * FCLK);//差分距离的延迟值
    int delay_diff1 = floor(diff1 / C_COARSE * FCLK);
    int delay_diff2 = floor(diff2 / C_COARSE * FCLK);

    for (int i = 0; i < 7; i++) {
        fifo_delay[i] = 3; // FIFO默认为3
    }

    for (int i = 0; i < 3; i++) {
        NO[i] = 0; // 初始化NO数组
    }

    int delay_diff0_frac = delay_diff0 % MAX_DELAY_VALUE;
    int delay_diff0_int = ceil((double)delay_diff0 / MAX_DELAY_VALUE);

    if (delay_diff0 == 0) {
        NO[0] = 0;
    }
    else if (delay_diff0_frac == 0) { //2048
        for (int i = 0; i < delay_diff0_int; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        NO[0] = delay_diff0_int;
    }
    else {
        for (int i = 0; i < delay_diff0_int - 1; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        fifo_delay[delay_diff0_int - 1] = delay_diff0_frac;
        NO[0] = delay_diff0_int;
    }

    int delay_diff1_frac = delay_diff1 % MAX_DELAY_VALUE;
    int delay_diff1_int = (int)ceil((double)delay_diff1 / MAX_DELAY_VALUE);

    if (delay_diff1 == 0) {
        NO[1] = NO[0];
    }
    else if (delay_diff1_frac == 0) { // 值为2048
        for (int i = delay_diff0_int; i < delay_diff0_int + delay_diff1_int; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        NO[1] = delay_diff1_int + delay_diff0_int;
    }
    else if (delay_diff1_int > 1) {
        for (int i = delay_diff0_int; i < delay_diff0_int + delay_diff1_int - 1; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        fifo_delay[delay_diff0_int + delay_diff1_int - 1] = delay_diff1_frac;
        NO[1] = delay_diff1_int + delay_diff0_int;
    }
    else {
        fifo_delay[delay_diff0_int + delay_diff1_int - 1] = delay_diff1_frac;
        NO[1] = delay_diff1_int + delay_diff0_int;
    }

    int delay_diff2_frac = delay_diff2 % MAX_DELAY_VALUE;
    int delay_diff2_int = (int)ceil((double)delay_diff2 / MAX_DELAY_VALUE);

    if (delay_diff2 == 0) {
        NO[2] = NO[1];
    }
    else if (delay_diff2_frac == 0) {
        for (int i = delay_diff0_int + delay_diff1_int; i < delay_diff0_int + delay_diff1_int + delay_diff2_int; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        NO[2] = delay_diff0_int + delay_diff1_int + delay_diff2_int;
    }
    else if (delay_diff2_int > 1) {
        for (int i = delay_diff0_int + delay_diff1_int; i < delay_diff0_int + delay_diff1_int + delay_diff2_int - 1; i++) {
            fifo_delay[i] = MAX_DELAY_VALUE;
        }
        fifo_delay[delay_diff0_int + delay_diff1_int + delay_diff2_int - 1] = delay_diff2_frac;
        NO[2] = delay_diff0_int + delay_diff1_int + delay_diff2_int;
    }
    else {
        fifo_delay[delay_diff0_int + delay_diff1_int + delay_diff2_int - 1] = delay_diff2_frac;
        NO[2] = delay_diff0_int + delay_diff1_int + delay_diff2_int;
    }
}

