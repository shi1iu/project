
  /*********************************************************************************************************************

  ��д��Ա����ҫ��
  ��д���ڣ�2024-8-8
  �汾��V1.0
  ��Ҫ���ܣ�ģ����λ�����Ͷ�̬Ŀ��
  �ļ�˵�����˰汾Ϊ����32��Ŀ���Ϊһ���Ķ�ֵ��ͨ���ⲿ����ȷ����32��Ŀ�����ʼλ�á��ٶȡ����ٶ�
  (��������Ϊx\y\z��ֵ��ÿ3��ֵ��x\y\z)Ϊһ��������ֱ����λ�ã��ٶȣ����ٶȣ�
  
  ���gcc -o client_constant client_constant.c -lm
        ./client_constant 50000 0 0 0 0 0 0 0 0
  **********************************************************************************************************************/
#include <stdio.h>
#include "BasicProtocol.h"
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <math.h>
#include <time.h>

#define SERVER_IP "192.168.6.222" // ��������IP��ַ
#define SERVER_PORT 1234      // �������Ķ˿ں�


double randi(int m, int n) { // ����һ����[m, n]��Χ�ڵ��������
    int num;
    num = rand() % (n - m + 1) + m;
    return num;
}

double randf(double m, double n) { // ����һ����[m, n]��Χ�ڵ����������
    double f = (double)rand() / RAND_MAX;
    return m + f * (n - m);
}

int main(int argc, char* argv[]) {
    if (argc < 10) {//argc=10���ǶԵ�
        printf("Usage: %s <pos_x> <pos_y> <pos_z> <speed_x> <speed_y> <speed_z> <accel_x> <accel_y> <accel_z>\n", argv[0]);
        return -1;
    }

    float pos[3], speed[3], accel[3];
    for (int i = 0; i < 3; i++) {
        pos[i] = atof(argv[i + 1]);//���ַ���ת���ɸ�����
        speed[i] = atof(argv[i + 4]);
        accel[i] = atof(argv[i + 7]);
    }

    int sockfd;
    struct sockaddr_in server_addr;
    struct ScatterChannelData Scatterchannel_data[CHANNELS];

    // ��ʼ�������������
    srand(time(NULL));

    // ����socket
    sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    int on = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    if (sockfd < 0) {
        perror("Cannot create socket");
        return -1;
    }

    int buffer_size = 1024 * 1024 * 64; // ����Ҫ���õĻ�������С������64KB
    socklen_t optlen = sizeof(buffer_size);

    // ���÷��ͻ�������С
    if (setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &buffer_size, optlen) == -1) {
        perror("ERROR setsockopt");
    }

    // ���÷�������ַ
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(SERVER_PORT);
    server_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

    for (int k = 0; k < CHANNELS; k++) {
        Scatterchannel_data[k].channel_id = k + 1;
        Scatterchannel_data[k].target_sum = TARGETS_PER_CHANNEL;
        printf("��%dͨ��,����%d��Ŀ��\n", k + 1, Scatterchannel_data[k].target_sum);
        for (int i = 0; i < MAX_SCATTER; i++) {
            for (int j = 0; j < TARGETS_PER_SCATTER; j++) {
                for (int f = 0; f < 3; f++) {
                    if (f == 0)
                    {
                        Scatterchannel_data[k].cp[i][j].Target_Position[f] = pos[f];
                        Scatterchannel_data[k].cp[i][j].Target_Speed[f] = speed[f];
                        Scatterchannel_data[k].cp[i][j].Target_Acceleration[f] = accel[f];
                     //   pos[f] = 50000;
                       pos[f] += 50000;
                    }else{
                        Scatterchannel_data[k].cp[i][j].Target_Position[f] = pos[f];
                        Scatterchannel_data[k].cp[i][j].Target_Speed[f] = speed[f];
                        Scatterchannel_data[k].cp[i][j].Target_Acceleration[f] = accel[f];
                    }
                }

                for (int ms = 0; ms < Target_timeinterval; ms++) {
                    for (int t = 0; t < 3; t++) {
                        Scatterchannel_data[k].cp[i][j].Positions_points[ms][t] = 0;
                        Scatterchannel_data[k].cp[i][j].Speeds_points[ms][t] = 0;
                    }
                }
                printf("Target(%f, %f, %f) \n", Scatterchannel_data[k].cp[i][j].Target_Position[0], Scatterchannel_data[k].cp[i][j].Target_Position[1], Scatterchannel_data[k].cp[i][j].Target_Position[2]);
            }
            printf("************************\n");
        }
    }

    // ���ӵ�������
    if (connect(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        perror("Cannot connect to server");
        close(sockfd);
        return -1;
    }

    for (int k = 0; k < CHANNELS; k++) {
        // ����Ŀ�����ݵ�������
        if (send(sockfd, &Scatterchannel_data[k], sizeof(struct ScatterChannelData), 0) < 0) {
            printf("error\n");
            perror("Cannot send data");
            close(sockfd);
            return -1;
        }
    }

    printf("Targets sent to server successfully\n");
    printf("Size of Constant_Parameters: %lu bytes\n", sizeof(struct ScatterChannelData));

    // �ر�socket
    close(sockfd);

    return 0;
}
