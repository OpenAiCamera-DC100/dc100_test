// Copyright 2020 Rockchip Electronics Co., Ltd. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <vector>

#include <sys/time.h>

#include <rga/im2d.h>
#include <rga/rga.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "librtsp/rtsp_demo.h"
#include "dcmedia_api.h"

#include "rockx/rockx.h"

#include "common/sample_common.h"

#define MAIN_WIDHT 1920
#define MAIN_HEIGHT 1080
#define SUB_WIDTH 1280
#define SUB_HEIGHT 720
#define MAX_ROCKX_LIST_NUM 10
#define MAX_RING_SIZE 128

rockx_config_t rockx_configs;
//rockx_object_array_t person_array;

rockx_handle_t person_det_handle;
rockx_handle_t object_track_handle;

rockx_module_t rockx;

typedef struct {              
    float x;    
    float y;  
    float w;  
    float h;   
} box_t;

typedef struct node {
    long timeval;
    rockx_object_array_t detect_result_group;
    struct node *next;
} Node;

typedef struct my_stack {
    int size;
    Node *top;
} rockx_list;

void create_rockx_list(rockx_list **s) {
    if (*s != NULL)
        return;
    *s = (rockx_list *)malloc(sizeof(rockx_list));
    (*s)->top = NULL;
    (*s)->size = 0;
    printf("create rockx_list success\n");
}

void destory_rockx_list(rockx_list **s) {
    Node *t = NULL;
    if (*s == NULL)
        return;
    while ((*s)->top)
    {
        t = (*s)->top;
        (*s)->top = t->next;
        free(t);
    }
    free(*s);
    *s = NULL;
}

void rockx_list_push(rockx_list *s, long timeval, rockx_object_array_t detect_result_group) {
    Node *t = NULL;
    t = (Node *)malloc(sizeof(Node));
    t->timeval = timeval;
    t->detect_result_group = detect_result_group;
    if (s->top == NULL)
    {
        s->top = t;
        t->next = NULL;
    }
    else
    {
        t->next = s->top;
        s->top = t;
    }
    s->size++;
}

void rockx_list_pop(rockx_list *s, long *timeval, rockx_object_array_t *detect_result_group) {
    Node *t = NULL;
    if (s == NULL || s->top == NULL)
        return;
    t = s->top;
    *timeval = t->timeval;
    *detect_result_group = t->detect_result_group;
    s->top = t->next;
    free(t);
    s->size--;
}

void rockx_list_drop(rockx_list *s) {
    Node *t = NULL;
    if (s == NULL || s->top == NULL)
        return;
    t = s->top;
    s->top = t->next;
    free(t);
    s->size--;
}

int rockx_list_size(rockx_list *s) {
    if (s == NULL)
        return -1;
    return s->size;
}

rockx_list *rockx_list_;

typedef struct
{              // 링버퍼 안의 개별 버퍼
    int id;    // 개별 버퍼에 복사된 실제 데이터 길이
    char data; // 개별 버퍼의 실제 저장 장소
} rign_item_t; // 링버퍼 내 개별 저장소

typedef struct
{
    int tag_head;                    // 쓰기 위치
    int tag_tail;                    // 읽기 위치
    rign_item_t item[MAX_RING_SIZE]; // 링버퍼 내의 저장 장소
} ring_t;                            // 링버퍼

/** ----------------------------------------------------------------------------
@brief  링버퍼에 데이터 저장
@remark 저장할 데이터는 최대 MAX_RING_DATA_SIZE로 저장
@param  ring : 링버퍼 포인터
@param  data : 저장할 데이터
@param  sz_data : 데이터 길이
@return -
 -----------------------------------------------------------------------------*/
void ring_put(ring_t *ring, int id, char data)
{

    // ring에 데이터 저장
    ring->item[ring->tag_head].id = id;
    ring->item[ring->tag_head].data = data;

    // ring tag 조정
    ring->tag_head = (ring->tag_head + 1) % MAX_RING_SIZE; // head 증가
    if (ring->tag_head == ring->tag_tail)
    {                                                          // 버퍼가 모두 찼다면
        ring->tag_tail = (ring->tag_tail + 1) % MAX_RING_SIZE; // tail 증가
    }
}

/** ----------------------------------------------------------------------------
@brief 링버퍼 초기화
@remark -
@param ring : 링버퍼 포인터
 -----------------------------------------------------------------------------*/
void ring_init(ring_t *ring)
{

    ring->tag_head = ring->tag_tail = 0; // 태그 값을 0으로 초기화
}

static long getCurrentTimeMsec() {
    long msec = 0;
    char str[20] = {0};
    struct timeval stuCurrentTime;

    gettimeofday(&stuCurrentTime, NULL);
    sprintf(str, "%ld%03ld", stuCurrentTime.tv_sec,
            (stuCurrentTime.tv_usec) / 1000);
    for (size_t i = 0; i < strlen(str); i++)
    {
        msec = msec * 10 + (str[i] - '0');
    }

    return msec;
}

static long int crv_tab[256];
static long int cbu_tab[256];
static long int cgu_tab[256];
static long int cgv_tab[256];
static long int tab_76309[256];
static unsigned char clp[1024]; // for clip in CCIR601

void init_yuv420p_table() {
    long int crv, cbu, cgu, cgv;
    int i, ind;
    static int init = 0;

    if (init == 1)
        return;

    crv = 104597;
    cbu = 132201; /* fra matrise i global.h */
    cgu = 25675;
    cgv = 53279;

    for (i = 0; i < 256; i++) {
        crv_tab[i] = (i - 128) * crv;
        cbu_tab[i] = (i - 128) * cbu;
        cgu_tab[i] = (i - 128) * cgu;
        cgv_tab[i] = (i - 128) * cgv;
        tab_76309[i] = 76309 * (i - 16);
    }

    for (i = 0; i < 384; i++)
        clp[i] = 0;
    ind = 384;
    for (i = 0; i < 256; i++)
        clp[ind++] = i;
    ind = 640;
    for (i = 0; i < 384; i++)
        clp[ind++] = 255;
    init = 1;
}

void nv12_to_rgb24(unsigned char *yuvbuffer, unsigned char *rga_buffer, int width, int height) {
    int y1, y2, u, v;
    unsigned char *py1, *py2;
    int i, j, c1, c2, c3, c4;
    unsigned char *d1, *d2;
    unsigned char *src_u;

    src_u = yuvbuffer + width * height; // u

    py1 = yuvbuffer; // y
    py2 = py1 + width;
    d1 = rga_buffer;
    d2 = d1 + 3 * width;

    init_yuv420p_table();

    for (j = 0; j < height; j += 2) {
        for (i = 0; i < width; i += 2) {
            u = *src_u++;
            v = *src_u++; // v immediately follows u, in the next position of u

            c4 = crv_tab[v];
            c2 = cgu_tab[u];
            c3 = cgv_tab[v];
            c1 = cbu_tab[u];

            // up-left
            y1 = tab_76309[*py1++];
            *d1++ = clp[384 + ((y1 + c1) >> 16)];
            *d1++ = clp[384 + ((y1 - c2 - c3) >> 16)];
            *d1++ = clp[384 + ((y1 + c4) >> 16)];

            // down-left
            y2 = tab_76309[*py2++];
            *d2++ = clp[384 + ((y2 + c1) >> 16)];
            *d2++ = clp[384 + ((y2 - c2 - c3) >> 16)];
            *d2++ = clp[384 + ((y2 + c4) >> 16)];

            // up-right
            y1 = tab_76309[*py1++];
            *d1++ = clp[384 + ((y1 + c1) >> 16)];
            *d1++ = clp[384 + ((y1 - c2 - c3) >> 16)];
            *d1++ = clp[384 + ((y1 + c4) >> 16)];

            // down-right
            y2 = tab_76309[*py2++];
            *d2++ = clp[384 + ((y2 + c1) >> 16)];
            *d2++ = clp[384 + ((y2 - c2 - c3) >> 16)];
            *d2++ = clp[384 + ((y2 + c4) >> 16)];
        }
        d1 += 3 * width;
        d2 += 3 * width;
        py1 += width;
        py2 += width;
    }

    // save bmp
    // int filesize = 54 + 3 * width * height;
    // FILE *f;
    // unsigned char bmpfileheader[14] = {'B', 'M', 0, 0,  0, 0, 0,
    //                                    0,   0,   0, 54, 0, 0, 0};
    // unsigned char bmpinfoheader[40] = {40, 0, 0, 0, 0, 0, 0,  0,
    //                                    0,  0, 0, 0, 1, 0, 24, 0};
    // unsigned char bmppad[3] = {0, 0, 0};

    // bmpfileheader[2] = (unsigned char)(filesize);
    // bmpfileheader[3] = (unsigned char)(filesize >> 8);
    // bmpfileheader[4] = (unsigned char)(filesize >> 16);
    // bmpfileheader[5] = (unsigned char)(filesize >> 24);

    // bmpinfoheader[4] = (unsigned char)(width);
    // bmpinfoheader[5] = (unsigned char)(width >> 8);
    // bmpinfoheader[6] = (unsigned char)(width >> 16);
    // bmpinfoheader[7] = (unsigned char)(width >> 24);
    // bmpinfoheader[8] = (unsigned char)(height);
    // bmpinfoheader[9] = (unsigned char)(height >> 8);
    // bmpinfoheader[10] = (unsigned char)(height >> 16);
    // bmpinfoheader[11] = (unsigned char)(height >> 24);

    // f = fopen("/tmp/tmp.bmp", "wb");
    // fwrite(bmpfileheader, 1, 14, f);
    // fwrite(bmpinfoheader, 1, 40, f);
    // for (int k = 0; k < height; k++) {
    //   fwrite(rga_buffer + (width * (height - k - 1) * 3), 3, width, f);
    //   fwrite(bmppad, 1, (4 - (width * 3) % 4) % 4, f);
    // }
    // fclose(f);
}

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

static bool quit = false;
static void sigterm_handler(int sig) {
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

static void *GetMediaBuffer(void *data) {
    int ret;
    MEDIA_BUFFER scrBuf;
    cv::Mat *img = NULL;

    //create a person detection handle
    ret = rockx_create(&person_det_handle, rockx, &rockx_configs, sizeof(rockx_config_t));
    if (ret != ROCKX_RET_SUCCESS) {
        printf("init rockx module ROCKX_MODULE error %d\n", ret);
        return NULL;
    }

    ret = rockx_create(&object_track_handle, ROCKX_MODULE_OBJECT_TRACK, nullptr, 0);
    if (ret != ROCKX_RET_SUCCESS) {
        printf("init rockx module ROCKX_MODULE_OBJECT_DETECTION error %d\n", ret);
    }

    rockx_image_t input_image;

    input_image.size = SUB_WIDTH * SUB_HEIGHT * 3;
    input_image.width = SUB_WIDTH;
    input_image.height = SUB_HEIGHT;
    input_image.pixel_format = ROCKX_PIXEL_FORMAT_RGB888;

    while(!quit) {
        scrBuf = DC_MPI_SYS_GetMediaBuffer(DC_ID_VI, 1, -1); //RGB로 변환된 0번 카메라 버퍼 받아오기
        if (scrBuf == NULL) {
            printf("VI[1] buffer empty!\n");
            continue;
        }

        //nv12 to rgb24
        int rga_buffer_size = SUB_WIDTH * SUB_HEIGHT * 3; // nv12 3/2, rgb 3
        unsigned char *rga_buffer = (unsigned char *)malloc(rga_buffer_size);

        nv12_to_rgb24((unsigned char *)DC_MPI_MB_GetPtr(scrBuf), rga_buffer, SUB_WIDTH, SUB_HEIGHT);


        input_image.data = (uint8_t *)rga_buffer;

        //detect
        rockx_object_array_t detect_array;
        rockx_object_array_t out_track_objects;
        
        memset(&detect_array, 0, sizeof(rockx_object_array_t));

        ret = rockx_person_detect(person_det_handle, &input_image, &detect_array, nullptr);
        if (ret != ROCKX_RET_SUCCESS) {
            printf("rockx_body_detect error %d\n", ret); 
            return NULL;
        }

        int max_track_time = 4;
        ret = rockx_object_track(object_track_handle, input_image.width,  input_image.height, max_track_time,
        &detect_array, &out_track_objects);
        if (ret != ROCKX_RET_SUCCESS) {
            printf("rockx_object_track error %d\n", ret);
            return NULL;
        }

        if (out_track_objects.count > 0) {
            rockx_list_push(rockx_list_, getCurrentTimeMsec(), out_track_objects);
            int size = rockx_list_size(rockx_list_);
            if (size >= MAX_ROCKX_LIST_NUM)
                rockx_list_drop(rockx_list_);
            //printf("size is %d\n", size);
        }

        DC_MPI_MB_ReleaseBuffer(scrBuf);
        if (rga_buffer)
            free(rga_buffer);
    }
    rockx_destroy(person_det_handle);
    rockx_destroy(object_track_handle);
    return NULL;
}

static void *MainStream(void *arg)
{
    box_t xywh[128];
    int track_id[128];
    int count;
    char in_cnt_flag = 0;
    char out_cnt_flag = 0;
    unsigned int in_cnt_num = 0;
    unsigned int out_cnt_num = 0;

    ring_t cnt_buf;
    ring_init(&cnt_buf);

    MEDIA_BUFFER buffer;
    cv::Mat *scr = NULL;
    // float scale_w = MAIN_WIDHT / SUB_WIDTH;
    // float scale_h = MAIN_HEIGHT / SUB_HEIGHT;

    std::vector<cv::Point> in_area = {
    cv::Point(530, 890),
    cv::Point(530, 740),
    cv::Point(1300, 740),
    cv::Point(1300, 890)};

    std::vector<cv::Point> out_area = {
    cv::Point(530, 890),
    cv::Point(530, 1040),
    cv::Point(1300, 1040),
    cv::Point(1300, 890)};

    
     while (!quit)
    {
        buffer = DC_MPI_SYS_GetMediaBuffer(DC_ID_RGA, 0, -1);
        if (!buffer)
        {
            continue;
        }

        scr = new cv::Mat(cv::Size(MAIN_WIDHT, MAIN_HEIGHT), CV_8UC3, (char *)DC_MPI_MB_GetPtr(buffer));

        if (rockx_list_size(rockx_list_))
        {

            long time_before;
            rockx_object_array_t detect_result_group;
            memset(&detect_result_group, 0, sizeof(detect_result_group));
            rockx_list_pop(rockx_list_, &time_before, &detect_result_group);
            // printf("time interval is %ld\n", getCurrentTimeMsec() - time_before);

            count = detect_result_group.count;

            for (unsigned long i = 0; i < count; i++)
            {
                detect_result_group.object[i].box.left *= 1.5;
                detect_result_group.object[i].box.top *= 1.5;
                detect_result_group.object[i].box.right *= 1.5;
                detect_result_group.object[i].box.bottom *= 1.5;
                
                xywh[i].x = detect_result_group.object[i].box.left;
                xywh[i].y = detect_result_group.object[i].box.top;
                xywh[i].w = detect_result_group.object[i].box.right - detect_result_group.object[i].box.left;
                xywh[i].h = detect_result_group.object[i].box.bottom - detect_result_group.object[i].box.top;
            
                track_id[i] = detect_result_group.object[i].id;

                if (xywh[i].w * xywh[i].h > 20) {
                    in_cnt_flag = (char)cv::pointPolygonTest(in_area, cv::Point(xywh[i].x + (xywh[i].w / 2), xywh[i].y + (xywh[i].h / 2)), false);
                    out_cnt_flag = (char)cv::pointPolygonTest(out_area, cv::Point(xywh[i].x + (xywh[i].w / 2), xywh[i].y + (xywh[i].h / 2)), false);

                    if (in_cnt_flag == 1) {
                        bool tmp = false;
                        for (int j = 0; j < MAX_RING_SIZE; j++)
                        {
                            if (track_id[i] == cnt_buf.item[j].id)
                            {
                                tmp = true;
                                if (cnt_buf.item[j].data == 2)
                                {
                                    in_cnt_num++;
                                    (&cnt_buf)->item[j].data = 1;

                                    break;
                                }
                            }
                        }
                        if (tmp == false)
                        {
                            ring_put(&cnt_buf, track_id[i], 1);
                        }
                        cv::circle(*scr, cv::Point(xywh[i].x + (xywh[i].w / 2), xywh[i].y + (xywh[i].h / 2)), 5, cv::Scalar(0, 0, 255), -1);

                    }
                    else if (out_cnt_flag == 1) {
                        bool tmp = false;
                        for (int j = 0; j < MAX_RING_SIZE; j++)
                        {
                            if (track_id[i] == cnt_buf.item[j].id)
                            {
                                tmp = true;
                                if (cnt_buf.item[j].data == 1)
                                {
                                    out_cnt_num++;
                                    (&cnt_buf)->item[j].data = 2;

                                    break;
                                }
                            }
                        }
                        if (tmp == false)
                        {
                            ring_put(&cnt_buf, track_id[i], 2);
                        }
                        cv::circle(*scr, cv::Point(xywh[i].x + (xywh[i].w / 2), xywh[i].y + (xywh[i].h / 2)), 5, cv::Scalar(255, 0, 0), -1);
                    

                    }
                    else {

                        
                    }
                
                }

            }

            for (unsigned long i = 0; i < count; i++) {
                    //cv::rectangle(*scr, cv::Rect(xywh[i].x, xywh[i].y - 40, xywh[i].w, 40), cv::Scalar(0, 0, 0), -1);
                    cv::rectangle(*scr, cv::Rect(xywh[i].x, xywh[i].y, xywh[i].w, xywh[i].h), cv::Scalar(0, 0, 255), 6);
                    cv::putText(*scr, cv::format("%d",track_id[i]), cv::Point(xywh[i].x, xywh[i].y - 10),
                                0, 1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }
        cv::line(*scr, cv::Point(530, 890), cv::Point(1300, 890), cv::Scalar(0, 0, 200), 4);
        // cv::arrowedLine(*scr, cv::Point(1370, 290), cv::Point(1370, 240), cv::Scalar(0, 0, 200), 3);
        // cv::arrowedLine(*scr, cv::Point(1370, 310), cv::Point(1370, 360), cv::Scalar(0, 0, 200), 3);
        // cv::putText(*scr, "IN", cv::Point(1380, 270), 0, 2, cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
        // cv::putText(*scr, "OUT", cv::Point(1380, 365), 0, 2, cv::Scalar(0, 0, 255), 4, cv::LINE_AA);
        cv::putText(*scr, cv::format("IN : %d", in_cnt_num), cv::Point(15, 55), 0, 2, cv::Scalar(18, 246, 246), 4, cv::LINE_AA);
        cv::putText(*scr, cv::format("OUT : %d", out_cnt_num), cv::Point(15, 110), 0, 2, cv::Scalar(18, 246, 246), 4, cv::LINE_AA);
        

        

        
        // send from VI to VENC
        delete scr;
        DC_MPI_SYS_SendMediaBuffer(DC_ID_VENC, 0, buffer);
        DC_MPI_MB_ReleaseBuffer(buffer);
    }
}

void video_packet_cb(MEDIA_BUFFER mb) { // Venc 데이터 정산, rtsp 전송 콜백
    static DC_S32 packet_cnt = 0;
    if (quit)
        return;

    //printf("#Get packet-%d, size %zu\n", packet_cnt, DC_MPI_MB_GetSize(mb));

    if (g_rtsplive && g_rtsp_session) {
        rtsp_tx_video(g_rtsp_session, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb)); // RTSP 전송，수신 MB 는 데이터 가상 주소, 크기, 타임스탬프를 인코딩한 다음 전송합니다.
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

static DC_CHAR optstr[] = "?::x:";
static const struct option long_options[] = {
    {"rockx_data", required_argument, NULL, 'x'},
    {"help", optional_argument, NULL, '?'},
    {NULL, 0, NULL, 0},
};

static void print_usage(const DC_CHAR *name) {
    printf("usage example:\n");
    printf("\t%s [-x face_v1 = face_detection] "  //face_detection_v3_fast
             "[   face_v2 = face_detection_v2]"  //face_detection_v2
             "[   face_v3 = face_detection_v3]"  //face_detection_v3
             "[   face_vl = face_detection_v3_large]"  //face_detection_v3_large
             "[   head_v1 = head_detection_v1]"   //head_detection
             "[   head_v2 = head_detection_v2]"  //head_detection_v2
            "\n",
            name);
}

int main(int argc, char **argv) {
    int ret = 0;

    DC_CHAR *pIqfilesPath = (char *)"/etc/iqfiles";

    int c;
    while ((c = getopt_long(argc, argv, optstr, long_options, NULL)) != -1) {
        const char *tmp_optarg = optarg;
        switch (c) {
        case 'x':
            if (!strcmp(optarg, "face_v1")) {
                rockx = ROCKX_MODULE_FACE_DETECTION;
                printf("face_v1 select.\n");
            } else if (!strcmp(optarg, "face_v2")) {
                rockx = ROCKX_MODULE_FACE_DETECTION_V2;
                printf("face_v2 select.\n");
            } else if (!strcmp(optarg, "face_v3")) {
                rockx = ROCKX_MODULE_FACE_DETECTION_V3;
                printf("face_v3 select.\n");
            } else if (!strcmp(optarg, "face_vl")) {
                rockx = ROCKX_MODULE_FACE_DETECTION_V3_LARGE;
                printf("face_vl select.\n");
            } else if (!strcmp(optarg, "head_v1")) {
                rockx = ROCKX_MODULE_HEAD_DETECTION;
                printf("head_v1 select.\n");
            } else if (!strcmp(optarg, "head_v2")) {
                rockx = ROCKX_MODULE_HEAD_DETECTION_V2;
                printf("head_v2 select.\n");
            } else {
                printf("ERROR: Invalid rockx type.\n");
                return 0;
            }
            break;
        case '?':
        default:
            print_usage(argv[0]);
            return 0;
        }
    }

    if (pIqfilesPath) { // RKAIQ iqfile 경로

        dc_aiq_working_mode_t hdr_mode = DC_AIQ_WORKING_MODE_NORMAL; 
        int fps = 25;
        SAMPLE_COMM_ISP_Init(0, hdr_mode, DC_FALSE, pIqfilesPath);
        SAMPLE_COMM_ISP_Run(0);
        SAMPLE_COMM_ISP_SetFrameRate(0, fps);
    }

    // init rtsp
    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");
    rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);

    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime()); //rtsp 현재 네트워크 시간 동기화

    DC_MPI_SYS_Init();  //ch init

    //0번 카메라 인풋 채널 생성
    VI_CHN_ATTR_S vi_chn_attr;
    vi_chn_attr.pcVideoNode = (char *)"rkispp_scale0";
    vi_chn_attr.u32BufCnt = 3;
    vi_chn_attr.u32Width = MAIN_WIDHT;
    vi_chn_attr.u32Height = MAIN_HEIGHT;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;

    ret |= DC_MPI_VI_SetChnAttr(0, 0, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(0, 0);
    if (ret) {
        printf("Create VI[0] failed! ret=%d\n", ret);
    }

    //1번 카메라 인풋 채널 생성
    vi_chn_attr.pcVideoNode = (char *)"rkispp_scale1";
    vi_chn_attr.u32BufCnt = 3;
    vi_chn_attr.u32Width = SUB_WIDTH;
    vi_chn_attr.u32Height = SUB_HEIGHT;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;

    ret |= DC_MPI_VI_SetChnAttr(0, 1, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(0, 1);
    if (ret) {
        printf("Create VI[1] failed! ret=%d\n", ret);
    }

    //0번 카메라 RGA채널 생성 (AI 분석을 위해 NV12 -> RGB888 변환) 
    RGA_ATTR_S rga_chn_attr;
    memset(&rga_chn_attr, 0, sizeof(rga_chn_attr));
    rga_chn_attr.bEnBufPool = RK_TRUE;       
    rga_chn_attr.u16BufPoolCnt = 3;         
    rga_chn_attr.u16Rotaion = 0;            
    // input image information 
    rga_chn_attr.stImgIn.u32X = 0;           
    rga_chn_attr.stImgIn.u32Y = 0;           
    rga_chn_attr.stImgIn.imgType = IMAGE_TYPE_NV12;   
    rga_chn_attr.stImgIn.u32Width = MAIN_WIDHT;       
    rga_chn_attr.stImgIn.u32Height = MAIN_HEIGHT;     
    rga_chn_attr.stImgIn.u32HorStride = MAIN_WIDHT;   
    rga_chn_attr.stImgIn.u32VirStride = MAIN_HEIGHT;  
    // Output image information 
    rga_chn_attr.stImgOut.u32X = 0;        
    rga_chn_attr.stImgOut.u32Y = 0;         
    rga_chn_attr.stImgOut.imgType = IMAGE_TYPE_BGR888; 
    rga_chn_attr.stImgOut.u32Width = MAIN_WIDHT;       
    rga_chn_attr.stImgOut.u32Height = MAIN_HEIGHT;
    rga_chn_attr.stImgOut.u32HorStride = MAIN_WIDHT;
    rga_chn_attr.stImgOut.u32VirStride = MAIN_HEIGHT;
    ret = DC_MPI_RGA_CreateChn(0, &rga_chn_attr);
    if (ret) {
        printf("Create RGA[0] failed! ret=%d\n", ret);
    }

    //0번 카메라 VENC 채널
    VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_RGB888;
	venc_chn_attr.stVencAttr.u32PicWidth = MAIN_WIDHT;
	venc_chn_attr.stVencAttr.u32PicHeight = MAIN_HEIGHT;
	venc_chn_attr.stVencAttr.u32VirWidth = MAIN_WIDHT;
	venc_chn_attr.stVencAttr.u32VirHeight = MAIN_HEIGHT;
	venc_chn_attr.stVencAttr.u32Profile = 77;

    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
	venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
	venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = 30;
	venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate = MAIN_WIDHT * MAIN_HEIGHT;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateNum = 25;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateNum = 25;

	ret = DC_MPI_VENC_CreateChn(0, &venc_chn_attr);
    if (ret) {
        printf("ERROR: create VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    MPP_CHN_S stEncChn;
    stEncChn.enModId = DC_ID_VENC;
    stEncChn.s32DevId = 0;
    stEncChn.s32ChnId = 0;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_cb);
    if (ret) {
        printf("ERROR: register output callback for VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    //***************채널 만들기 끝

    //VI - RGA 바인드
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32ChnId = 0;
    stDestChn.enModId = DC_ID_RGA;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind VI[0]-RGA[0] failed! ret=%d\n", ret);
    }
    DC_MPI_VI_StartStream(0, 1);


    //********************바인드 끝
    
    printf("%s initial finish\n", __func__);

    rockx_add_config(&rockx_configs, (char *)ROCKX_CONFIG_DATA_PATH, "/oem/usr/lib/");

    create_rockx_list(&rockx_list_);

    pthread_t read_thread;
    pthread_create(&read_thread, NULL, GetMediaBuffer, NULL);

    // The mainstream draws a box asynchronously based on the recognition result
    pthread_t main_stream_thread;
    pthread_create(&main_stream_thread, NULL, MainStream, NULL);
    
    signal(SIGINT, sigterm_handler);
    while (!quit) {
        usleep(500000);
    }

    rtsp_del_demo(g_rtsplive);
    pthread_join(read_thread, NULL);
    pthread_join(main_stream_thread, NULL);


    // unbind start
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32ChnId = 0;
    stDestChn.enModId = DC_ID_RGA;
    stDestChn.s32ChnId = 0;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Unbind VI[0] - RGA[0] failed! ret=%d\n", ret);
    }
    // unbind end

    // Destroy ch start
    DC_MPI_VENC_DestroyChn(0);
    DC_MPI_RGA_DestroyChn(0);
    DC_MPI_VI_DisableChn(0, 1);
    DC_MPI_VI_DisableChn(0, 0);
    
    // Destroy ch end
    if (pIqfilesPath) {
        SAMPLE_COMM_ISP_Stop(0);
    }
}


