#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string.h>
#include <pthread.h>
#include <stdbool.h>
#include <ifaddrs.h>

#include "json-c/json.h"

#define SEEK_DEVICE     "SeekDevice"
#define RUN             "Run"
#define STOP            "Stop"
#define EX_1            "cmos_rightcam_rtsp_test"
#define EX_2            "cmos_leftcam_rtsp_test"
#define EX_3            "cmos_tof_rtsp_test"
#define EX_4            "cmos_thermalcam_rtsp_test"
#define EX_5            "cmos_person_detection_test"
#define EX_6            "cmos_object_detection_test"
#define EX_7            "cmos_stereo_depthmap_test"


int udp_broadcast_tx(char *msg)
{
    int sock;
    struct sockaddr_in peer_addr;
    const int opt = 1;
    //char msg[100] = "Msg from udp broadcast client!";
    socklen_t peer_addrlen = 0;
    int ret = 0;
 
    bzero(&peer_addr, sizeof(struct sockaddr_in));
    peer_addr.sin_family = AF_INET;
    peer_addr.sin_port = htons(6868);
    peer_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    peer_addrlen = sizeof(struct sockaddr_in);
 
    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
        printf("Ceate sock fail\n");
 
    ret = setsockopt(sock, SOL_SOCKET, SO_BROADCAST, (char *)&opt, sizeof(opt));
    if (ret == -1)
        printf("Set sock to broadcast format fail\n");
 
    sendto(sock, msg, strlen(msg), 0,
            (struct sockaddr *)&peer_addr, peer_addrlen);
    //LOG_INFO("Done\n");
}

int main() {
    int sock;
    struct sockaddr_in own_addr, peer_addr;
    struct ifaddrs *ifap, *ifa;
    struct sockaddr_in *sa;
    const int opt = 1;
    char recv_msg[1024] = {0};
    socklen_t peer_addrlen = 0;
    char peer_name[30] = {0};
    char *ip_address;
    int ret = 0;
    bool state = true;
    
    bzero(&own_addr, sizeof(struct sockaddr_in));
    bzero(&peer_addr, sizeof(struct sockaddr_in));
    own_addr.sin_family = AF_INET;
    own_addr.sin_port = htons(6868);
    own_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock == -1)
        printf("Ceate sock fail\n");

    ret = bind(sock, (struct sockaddr *)&own_addr, sizeof(struct sockaddr_in));
    if (ret == -1)
        printf("Bind addr fail\n");
    
    if (getifaddrs(&ifap) == -1) {
        perror("getifaddrs");
        return 1;
    }

    for (ifa = ifap; ifa; ifa = ifa->ifa_next) {
        if (ifa->ifa_addr && ifa->ifa_addr->sa_family == AF_INET) {
            sa = (struct sockaddr_in *)ifa->ifa_addr;
            ip_address = inet_ntoa(sa->sin_addr);
            
            // 출력
            printf("Interface: %s, IP Address: %s\n", ifa->ifa_name, ip_address);
        }
    }
    freeifaddrs(ifap);
    
    while (1) {
        printf("1\n");
        ret = recvfrom(sock, recv_msg, sizeof(recv_msg), 0,
            (struct sockaddr *)&peer_addr, &peer_addrlen);  //메시지 전송 받기.
        if (ret > 0) {
            printf("2\n");
            inet_ntop(AF_INET, &peer_addr.sin_addr.s_addr,
                    peer_name, sizeof(peer_name));
            //LOG_INFO("Recv from %s, msg[%s]\n", peer_name, recv_msg);
            json_object *j_cfg = json_tokener_parse(recv_msg);
            if (j_cfg) {
                printf("3\n");
                char *sender = (char *)json_object_get_string(json_object_object_get(j_cfg, "Sender"));
                //LOG_INFO("%s,sender = %s\n", __func__, sender);
                if (!strcmp("client", sender)) {
                    printf("4\n");
                    char *cmd = (char *)json_object_get_string(json_object_object_get(j_cfg, "Cmd"));
                    if (!strcmp(cmd, SEEK_DEVICE)) {
                        printf("seekdevice\n");
                        
                        json_object *j_rx = json_object_new_object();
                        json_object_object_add(j_rx, "Cmd", json_object_new_string(SEEK_DEVICE));
                        json_object_object_add(j_rx, "Sender", json_object_new_string("server"));
                        json_object_object_add(j_rx, "Data", json_object_new_string(""));
                        udp_broadcast_tx((char *)json_object_to_json_string(j_rx));
                        json_object_put(j_rx);
                    }
                    else if (!strcmp(cmd, RUN))
                    {       
                        printf("run\n");             
                        char *data = (char *)json_object_get_string(json_object_object_get(j_cfg, "Data"));
                        if (!strcmp(data, ip_address)) {
                            printf("data : %s\n", data); 
                            char *exam = (char *)json_object_get_string(json_object_object_get(j_cfg, "Exam"));
                            printf("exam : %s\n", exam);
                            char *tmp;
                            if(!strcmp(exam, EX_1)) {
                                system("cmos_rightcam_rtsp_test &");
                            }
                            else if(!strcmp(exam, EX_2)) {
                                system("cmos_leftcam_rtsp_test &");
                            }
                            else if(!strcmp(exam, EX_3)) {
                                system("cmos_tof_rtsp_test");
                            }
                            else if(!strcmp(exam, EX_4)) {
                                system("cmos_thermalcam_rtsp_test &"); 
                            }
                            else if(!strcmp(exam, EX_5)) {
                                system("cmos_person_detection_test &");
                            }
                            else if(!strcmp(exam, EX_6)) {
                               
                            }
                            else if(!strcmp(exam, EX_7)) {
                                
                            }
                            else {
                                
                            }
                        }    
                    }
                    else if (!strcmp(cmd, STOP)) {
                        char *data = (char *)json_object_get_string(json_object_object_get(j_cfg, "Data"));
                        if (!strcmp(data, ip_address)) {
                            char *exam = (char *)json_object_get_string(json_object_object_get(j_cfg, "Exam"));
                            char *tmp;
                            if(!strcmp(exam, EX_1)) {
                                system("killall cmos_rightcam_rtsp_test");
                            }
                            else if(!strcmp(exam, EX_2)) {
                                system("killall cmos_leftcam_rtsp_test");
                            }
                            else if(!strcmp(exam, EX_3)) {
                                system("killall cmos_tof_rtsp_test");
                            }  
                            else if(!strcmp(exam, EX_4)) {
                                system("killall cmos_thermalcam_rtsp_test");
                            }
                            else if(!strcmp(exam, EX_5)) {
                                system("killall cmos_person_detection_test");
                            }
                            else if(!strcmp(exam, EX_6)) {
                                
                            }
                            else if(!strcmp(exam, EX_7)) {
                                
                            }
                            else {
                            
                            }

                        }
                    }
                    else {

                    }
                }
            } 
            else {
                printf("Recv msg err\n");
            }
            json_object_put(j_cfg);
            bzero(recv_msg, sizeof(recv_msg));
        }
    }
    
    return 0;
}