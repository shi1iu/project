///*********************************************************************************************************************
//
//编写人员：刘耀乐
//编写日期：2024-8-8
//版本：V1.0
//主要功能：模拟上位机发送动态目标
//文件说明：此版本为发送32个目标随机值，范围由外部键盘决定，通过外部输入确定这32个目标的起始位置、速度、加速度
//(参数定义为x\y\z的最小值以及最大值，每6个值为一组参数，分别代表位置，速度，加速度）
//
//命令：gcc -o client client.c -lm
//      ./client 1.0 10.0 1.0 10.0 1.0 10.0 1.0 7.0 1.0 7.0 1.0 7.0 1.0 5.0 1.0 5.0 1.0 5.0
//**********************************************************************************************************************/
//#include <stdio.h>
//#include "BasicProtocol.h"
//#include <stdlib.h>
//#include <string.h>
//#include <unistd.h>
//#include <sys/socket.h>
//#include <netinet/in.h>
//#include <arpa/inet.h>
//#include <math.h>
//#include <time.h>
//
//#define SERVER_IP "192.168.6.222" // 服务器的IP地址
//#define SERVER_PORT 1234      // 服务器的端口号
//
//
//double randi(int m, int n) { // 生成一个在[m, n]范围内的随机整数
//    int num;
//    num = rand() % (n - m + 1) + m;
//    return num;
//}
//
//double randf(double m, double n) { // 生成一个在[m, n]范围内的随机浮点数
//    double f = (double)rand() / RAND_MAX;
//    return m + f * (n - m);
//}
//
//int main(int argc, char* argv[]) {
//    if (argc < 19) {
//        printf("Usage: %s <pos_x_min> <pos_x_max> <pos_y_min> <pos_y_max> <pos_z_min> <pos_z_max> <speed_x_min> <speed_x_max> <speed_y_min> <speed_y_max> <speed_z_min> <speed_z_max> <accel_x_min> <accel_x_max> <accel_y_min> <accel_y_max> <accel_z_min> <accel_z_max>\n", argv[0]);
//        return -1;
//    }
//
//    float pos_min[3], pos_max[3], speed_min[3], speed_max[3], accel_min[3], accel_max[3];
//    for (int i = 0; i < 3; i++) {
//        pos_min[i] = atof(argv[i * 2 + 1]);
//        pos_max[i] = atof(argv[i * 2 + 2]);
//        speed_min[i] = atof(argv[i * 2 + 7]);
//        speed_max[i] = atof(argv[i * 2 + 8]);
//        accel_min[i] = atof(argv[i * 2 + 13]);
//        accel_max[i] = atof(argv[i * 2 + 14]);
//    }
//
//    int sockfd;
//    struct sockaddr_in server_addr;
//    struct ScatterChannelData Scatterchannel_data[CHANNELS];
//
//    // 初始化随机数发生器
//    srand(time(NULL));
//
//    // 创建socket
//    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
//    int on = 1;
//    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
//    if (sockfd < 0) {
//        perror("Cannot create socket");
//        return -1;
//    }
//
//    int buffer_size = 1024 * 1024 * 64; // 您想要设置的缓冲区大小，例如64KB
//    socklen_t optlen = sizeof(buffer_size);
//
//    // 设置发送缓冲区大小
//    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buffer_size, optlen) == -1) {
//        perror("ERROR setsockopt");
//    }
//
//    // 设置服务器地址
//    memset(&server_addr, 0, sizeof(server_addr));
//    server_addr.sin_family = AF_INET;
//    server_addr.sin_port = htons(SERVER_PORT);
//    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
//
//    for (int k = 0; k < CHANNELS; k++) {
//        Scatterchannel_data[k].channel_id = k + 1;
//        Scatterchannel_data[k].target_sum = TARGETS_PER_CHANNEL;
//        printf("第%d通道,存在%d个目标\n", k + 1, Scatterchannel_data[k].target_sum);
//        for (int i = 0; i < MAX_SCATTER; i++) {
//            for (int j = 0; j < TARGETS_PER_SCATTER; j++) {
//                for (int f = 0; f < 3; f++) {
//                    Scatterchannel_data[k].cp[i][j].Target_Position[f] = randf(pos_min[f], pos_max[f]);
//                    Scatterchannel_data[k].cp[i][j].Target_Speed[f] = randf(speed_min[f], speed_max[f]);
//                    Scatterchannel_data[k].cp[i][j].Target_Acceleration[f] = randf(accel_min[f], accel_max[f]);
//                }
//
//                for (int ms = 0; ms < Target_timeinterval; ms++) {
//                    for (int t = 0; t < 3; t++) {
//                        Scatterchannel_data[k].cp[i][j].Positions_points[ms][t] = 0;
//                        Scatterchannel_data[k].cp[i][j].Speeds_points[ms][t] = 0;
//                    }
//                }
//                printf("Target(%f, %f, %f) \n", Scatterchannel_data[k].cp[i][j].Target_Position[0], Scatterchannel_data[k].cp[i][j].Target_Position[1], Scatterchannel_data[k].cp[i][j].Target_Position[2]);
//            }
//            printf("************************\n");
//        }
//    }
//
//    // 连接到服务器
//    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
//        perror("Cannot connect to server");
//        close(sockfd);
//        return -1;
//    }
//
//    for (int k = 0; k < CHANNELS; k++) {
//        // 发送目标数据到服务器
//        if (send(sockfd, &Scatterchannel_data[k], sizeof(struct ScatterChannelData), 0) < 0) {
//            printf("error\n");
//            perror("Cannot send data");
//            close(sockfd);
//            return -1;
//        }
//    }
//
//    printf("Targets sent to server successfully\n");
//    printf("Size of Constant_Parameters: %lu bytes\n", sizeof(struct ScatterChannelData));
//
//    // 关闭socket
//    close(sockfd);
//
//    return 0;
//}
