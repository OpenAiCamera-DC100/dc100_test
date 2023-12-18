#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <errno.h>
#include <rga/im2d.h>
#include <rga/rga.h>
#include <vector>
#include <sys/time.h>

#include "dcmedia_api.h"
#include "rknn_api.h"
#include "common/sample_common.h"
#include "librtsp/rtsp_demo.h"
#include "yolov5/YOLOv562Detector.h"
#include "tracker/bytetrack/include/BYTETracker.h"
// #include "yolov5/postprocess.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#define RKNN_INPUT 640
#define MAIN_WIDHT 1920
#define MAIN_HEIGHT 1080
#define SUB_WIDTH 1280
#define SUB_HEIGHT 720
#define MAX_RKNN_LIST_NUM 10
#define MAX_RING_SIZE 50 // 링버퍼가 갖는 아이템 개수

YOLOv562Detector detector;

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

typedef struct
{              
    float x;    
    float y;  
    float w;  
    float h;   
} box_t; 

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

static bool quit = false;
static void sigterm_handler(int sig)
{
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

void video_packet_cb(MEDIA_BUFFER mb)
{
    static DC_S32 packet_cnt = 0;
    size_t buffSize;
    if (quit)
        return;

    buffSize = DC_MPI_MB_GetSize(mb);

    // printf("# CAM A: Get packet-%d, size %zu\n", packet_cnt, buffSize);

    if (g_rtsplive && g_rtsp_session)
    {
        rtsp_tx_video(g_rtsp_session, (uint8_t const *)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb));
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

static long getCurrentTimeMsec()
{
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

void init_yuv420p_table()
{
    long int crv, cbu, cgu, cgv;
    int i, ind;
    static int init = 0;

    if (init == 1)
        return;

    crv = 104597;
    cbu = 132201; /* fra matrise i global.h */
    cgu = 25675;
    cgv = 53279;

    for (i = 0; i < 256; i++)
    {
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

int rgb24_resize(unsigned char *input_rgb, unsigned char *output_rgb, int width,
                 int height, int outwidth, int outheight)
{
    rga_buffer_t src =
        wrapbuffer_virtualaddr(input_rgb, width, height, RK_FORMAT_RGB_888);
    rga_buffer_t dst = wrapbuffer_virtualaddr(output_rgb, outwidth, outheight,
                                              RK_FORMAT_RGB_888);
    rga_buffer_t pat = {0};
    im_rect src_rect = {0, 0, width, height};
    im_rect dst_rect = {0, 0, outwidth, outheight};
    im_rect pat_rect = {0};
    IM_STATUS STATUS = improcess(src, dst, pat, src_rect, dst_rect, pat_rect, 0);
    if (STATUS != IM_STATUS_SUCCESS)
    {
        printf("imcrop failed: %s\n", imStrError(STATUS));
        quit = false;
        return -1;
    }
    return 0;
}

void nv12_to_rgb24(unsigned char *yuvbuffer, unsigned char *rga_buffer,
                   int width, int height)
{
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

    for (j = 0; j < height; j += 2)
    {
        for (i = 0; i < width; i += 2)
        {
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

static void printRKNNTensor(rknn_tensor_attr *attr)
{
    printf("index=%d name=%s n_dims=%d dims=[%d %d %d %d] n_elems=%d size=%d "
           "fmt=%d type=%d qnt_type=%d fl=%d zp=%d scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2],
           attr->dims[1], attr->dims[0], attr->n_elems, attr->size, 0, attr->type,
           attr->qnt_type, attr->fl, attr->zp, attr->scale);
}

static void *GetMediaBuffer(void *arg)
{
    printf("#Start %s thread, arg:%p\n", __func__, arg);

    ModelConfig cfg;
    cfg.nc = 80;
    cfg.model_file_path = (char *)"/oem/usr/share/yolov5s_v6.2_output3_4.rknn";
    cfg.nms_threshold = 0.45;
    cfg.conf_thres = 0.25;

    cv::Mat *scr = NULL;
    detector.load_model(&cfg);

    MEDIA_BUFFER buffer = NULL;
    while (!quit)
    {
        buffer = DC_MPI_SYS_GetMediaBuffer(DC_ID_VI, 1, -1);
        if (!buffer)
        {
            continue;
        }

        int rga_buffer_size = SUB_WIDTH * SUB_HEIGHT * 3; // nv12 3/2, rgb 3
        // int rga_buffer_model_input_size = RKNN_INPUT * RKNN_INPUT * 3;
        unsigned char *rga_buffer = (unsigned char *)malloc(rga_buffer_size);
        // unsigned char *rga_buffer_model_input = (unsigned char *)malloc(rga_buffer_model_input_size);

        nv12_to_rgb24((unsigned char *)DC_MPI_MB_GetPtr(buffer), rga_buffer, SUB_WIDTH, SUB_HEIGHT);
        // rgb24_resize(rga_buffer, rga_buffer_model_input, SUB_WIDTH, SUB_HEIGHT, RKNN_INPUT, RKNN_INPUT);

        scr = new cv::Mat(cv::Size(SUB_WIDTH, SUB_HEIGHT), CV_8UC3, rga_buffer);

        detect_result_group_t detect_result_group = detector.inference(*scr, RKNN_INPUT);

        delete scr;

        // Post Process

        if (detect_result_group.count > 0)
        {
            detector.rknn_list_push(detector.rknn_list_, getCurrentTimeMsec(), detect_result_group);
            int size = detector.rknn_list_size(detector.rknn_list_);
            if (size >= MAX_RKNN_LIST_NUM)
                detector.rknn_list_drop(detector.rknn_list_);
            // printf("size is %d\n", size);
        }

        DC_MPI_MB_ReleaseBuffer(buffer);
        if (rga_buffer)
            free(rga_buffer);
        // if (rga_buffer_model_input)
        //     free(rga_buffer_model_input);
    }
    // release
    // if (ctx)
    //    rknn_destroy(ctx);
    // if (model)
    //    free(model);

    return NULL;
}

static void *MainStream(void *arg)
{
    char in_cnt_flag = false;
    char out_cnt_flag = false;
    unsigned int in_cnt_num = 0;
    unsigned int out_cnt_num = 0;
    ring_t cnt_buf;
    ring_init(&cnt_buf);

    int _output_stack_size = 0;
    cv::Scalar _s[64];
    box_t xywh[64];
    int _track_id[64];
    char _name[64][OBJ_NAME_MAX_SIZE];
    unsigned char no_blink = 0;

    MEDIA_BUFFER buffer;
    cv::Mat *scr = NULL;
    // float scale_w = MAIN_WIDHT / SUB_WIDTH;
    // float scale_h = MAIN_HEIGHT / SUB_HEIGHT;

    int fps = 15;
    BYTETracker bytetracker(fps, 30);

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

        if (detector.rknn_list_size(detector.rknn_list_))
        {
            char text[256];
            long time_before;
            detect_result_group_t detect_result_group;
            memset(&detect_result_group, 0, sizeof(detect_result_group));
            detector.rknn_list_pop(detector.rknn_list_, &time_before, &detect_result_group);
            // printf("time interval is %ld\n", getCurrentTimeMsec() - time_before);

            std::vector<detect_result_t> objects;

            for (detect_result_t dr : detect_result_group.results)
            {
                
                if (dr.classId == 0) // person
                {
                    objects.push_back(dr);
                }
                
                //objects.push_back(dr);
            }

            std::vector<STrack> output_stracks = bytetracker.update(objects);

            _output_stack_size = output_stracks.size();
            for (unsigned long i = 0; i < _output_stack_size; i++)
            {
                std::vector<float> tlwh = output_stracks[i].tlwh;
                tlwh[0] *= 1.5;
                tlwh[1] *= 1.5;
                tlwh[2] *= 1.5;
                tlwh[3] *= 1.5;
                
                xywh[i].x = tlwh[0];
                xywh[i].y = tlwh[1];
                xywh[i].w = tlwh[2];
                xywh[i].h = tlwh[3];
                _track_id[i] = output_stracks[i].track_id;
                strcpy(_name[i], detect_result_group.results[i].name);

                bool vertical = (tlwh[2] - tlwh[0]) / tlwh[3] > 1.6;
                if (tlwh[2] * tlwh[3] > 20 && !vertical)
                {
                    in_cnt_flag = (char)cv::pointPolygonTest(in_area, cv::Point(tlwh[0] + (tlwh[2] / 2), tlwh[1] + tlwh[3]), false);
                    out_cnt_flag = (char)cv::pointPolygonTest(out_area, cv::Point(tlwh[0] + (tlwh[2] / 2), tlwh[1] + tlwh[3]), false);

                    if (in_cnt_flag == 1)
                    {
                        bool tmp = false;
                        for (int j = 0; j < MAX_RING_SIZE; j++)
                        {
                            if (output_stracks[i].track_id == cnt_buf.item[j].id)
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
                            ring_put(&cnt_buf, output_stracks[i].track_id, 1);
                        }
                        cv::circle(*scr, cv::Point(tlwh[0] + (tlwh[2] / 2), tlwh[1] + (tlwh[3] / 2)), 5, cv::Scalar(0, 0, 255), -1);
                    }
                    else if (out_cnt_flag == 1)
                    {
                        bool tmp = false;
                        for (int j = 0; j < MAX_RING_SIZE; j++)
                        {
                            if (output_stracks[i].track_id == cnt_buf.item[j].id)
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
                            ring_put(&cnt_buf, output_stracks[i].track_id, 2);
                        }
                        cv::circle(*scr, cv::Point(tlwh[0] + (tlwh[2] / 2), tlwh[1] + (tlwh[3] / 2)), 5, cv::Scalar(255, 0, 0), -1);
                    }
                    else
                    {
                    }

                    cv::Point objectPosition(tlwh[0] + (tlwh[2] / 2), tlwh[1] + (tlwh[3] / 2));

                    cv::Scalar s = bytetracker.get_color(output_stracks[i].track_id);
                    _s[i] = s;


                }
            }
            no_blink = 0;
        }

        if (no_blink < 3)
        {
            for (unsigned long i = 0; i < _output_stack_size; i++) {
                    cv::rectangle(*scr, cv::Rect(xywh[i].x, xywh[i].y - 40, xywh[i].w, 40), cv::Scalar(0, 0, 0), -1);
                    cv::rectangle(*scr, cv::Rect(xywh[i].x, xywh[i].y, xywh[i].w, xywh[i].h), _s[i], 6);
                    cv::putText(*scr, cv::format("%d #%s",_track_id[i], _name[i]), cv::Point(xywh[i].x, xywh[i].y - 10),
                                0, 1, cv::Scalar(255, 255, 255), 2, cv::LINE_AA);
            }
        }
        no_blink++;
         if (no_blink > 250)
        {
            no_blink = 3;
        }
        // cv::line(*scr, cv::Point(1370, 300), cv::Point(1875, 300), cv::Scalar(0, 0, 200), 4);
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

int main()
{

    // dcmedia
    DC_U32 u32Width = 1920;
    DC_U32 u32Height = 1080;
    // DC_U32 bitrate = 5000000;
    // DC_U32 frame = 30;
    DC_BOOL bMultictx = DC_FALSE;
    DC_CHAR *pIqfilesPath = (char *)"/etc/iqfiles";
    CODEC_TYPE_E enCodecType = DC_CODEC_TYPE_H264;
    DC_CHAR *pCodecName = (char *)"H264";

    int ret;

    // rknn init
    // printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n",
    //       box_conf_threshold, nms_threshold);

    // init cam
    dc_aiq_working_mode_t hdr_mode = DC_AIQ_WORKING_MODE_NORMAL;
    int fps = 15;
    SAMPLE_COMM_ISP_Init(0, hdr_mode, bMultictx, pIqfilesPath);
    SAMPLE_COMM_ISP_Run(0);
    SAMPLE_COMM_ISP_SetFrameRate(0, fps);

    // init rtsp
    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");
    rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime());

    // dcmedia init
    DC_MPI_SYS_Init();

    VI_CHN_ATTR_S vi_chn_attr;
    vi_chn_attr.pcVideoNode = (char *)"rkispp_scale0";
    vi_chn_attr.u32BufCnt = 3;
    vi_chn_attr.u32Width = MAIN_WIDHT;
    vi_chn_attr.u32Height = MAIN_HEIGHT;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    ret = DC_MPI_VI_SetChnAttr(0, 0, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(0, 0);
    if (ret)
    {
        printf("ERROR: create VI[0] error! ret=%d\n", ret);
        return 0;
    }

    vi_chn_attr.pcVideoNode = (char *)"rkispp_scale1";
    vi_chn_attr.u32BufCnt = 3;
    vi_chn_attr.u32Width = SUB_WIDTH;
    vi_chn_attr.u32Height = SUB_HEIGHT;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    ret = DC_MPI_VI_SetChnAttr(0, 1, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(0, 1);
    if (ret)
    {
        printf("ERROR: create VI[1] error! ret=%d\n", ret);
        return 0;
    }

    RGA_ATTR_S stRgaAttr;
    memset(&stRgaAttr, 0, sizeof(stRgaAttr));
    stRgaAttr.bEnBufPool = RK_TRUE; // Enable buffer pool.
    stRgaAttr.u16BufPoolCnt = 3;    // Buffer pool count.
    stRgaAttr.u16Rotaion = 0;       // Rotation angle. Values : 0, 90, 180, 270.
    // input image information
    stRgaAttr.stImgIn.u32X = 0;                   // X-axis coordinate of camera.
    stRgaAttr.stImgIn.u32Y = 0;                   // Y-axis coordinate of camera.
    stRgaAttr.stImgIn.imgType = IMAGE_TYPE_NV12;  // Image format type.
    stRgaAttr.stImgIn.u32Width = MAIN_WIDHT;      // The width of RGA.
    stRgaAttr.stImgIn.u32Height = MAIN_HEIGHT;    // The height of RGA.
    stRgaAttr.stImgIn.u32HorStride = MAIN_WIDHT;  // Virture width.
    stRgaAttr.stImgIn.u32VirStride = MAIN_HEIGHT; // Virture height.
    // Output image information
    stRgaAttr.stImgOut.u32X = 0;                    // X-axis coordinate of RGA.
    stRgaAttr.stImgOut.u32Y = 0;                    // Y-axis coordinate of RGA.
    stRgaAttr.stImgOut.imgType = IMAGE_TYPE_BGR888; // Image format type.
    stRgaAttr.stImgOut.u32Width = MAIN_WIDHT;       // The width of RGA.
    stRgaAttr.stImgOut.u32Height = MAIN_HEIGHT;
    stRgaAttr.stImgOut.u32HorStride = MAIN_WIDHT;
    stRgaAttr.stImgOut.u32VirStride = MAIN_HEIGHT;
    ret = DC_MPI_RGA_CreateChn(0, &stRgaAttr);
    if (ret)
    {
        printf("Create RGA[0] failed! ret=%d\n", ret);
    }

    VENC_CHN_ATTR_S venc_chn_attr;
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));

    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
    venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = 30;
    venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = u32Width * u32Height * 3;
    // frame rate: in 30/1, out 30/1.
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = 30;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = 30;

    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_RGB888;
    venc_chn_attr.stVencAttr.u32PicWidth = MAIN_WIDHT;
    venc_chn_attr.stVencAttr.u32PicHeight = MAIN_HEIGHT;
    venc_chn_attr.stVencAttr.u32VirWidth = MAIN_WIDHT;
    venc_chn_attr.stVencAttr.u32VirHeight = MAIN_HEIGHT;
    venc_chn_attr.stVencAttr.u32Profile = 77;
    ret = DC_MPI_VENC_CreateChn(0, &venc_chn_attr);
    if (ret)
    {
        printf("ERROR: create VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    MPP_CHN_S stEncChn;
    stEncChn.enModId = DC_ID_VENC;
    stEncChn.s32DevId = 0;
    stEncChn.s32ChnId = 0;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_cb);
    if (ret)
    {
        printf("ERROR: register output callback for VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    MPP_CHN_S stSrcChn;
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;

    MPP_CHN_S stDestChn;
    stDestChn.enModId = DC_ID_RGA;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret)
    {
        printf("ERROR: Bind VI[0] and RGA[0] error! ret=%d\n", ret);
        return 0;
    }
    DC_MPI_VI_StartStream(0, 1);

    detector.create_rknn_list(&(detector.rknn_list_));

    pthread_t read_thread;
    pthread_create(&read_thread, NULL, GetMediaBuffer, NULL);

    // The mainstream draws a box asynchronously based on the recognition result
    pthread_t main_stream_thread;
    pthread_create(&main_stream_thread, NULL, MainStream, NULL);

    printf("%s initial finish\n", __func__);
    signal(SIGINT, sigterm_handler);

    while (!quit)
    {
        usleep(500000);
    }
    printf("%s exit!\n", __func__);

    rtsp_del_demo(g_rtsplive);

    pthread_join(read_thread, NULL);
    pthread_join(main_stream_thread, NULL);

    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;

    stDestChn.enModId = DC_ID_RGA;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret)
    {
        printf("ERROR: UnBind VI[0] and RGA[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_VENC_DestroyChn(0);
    if (ret)
    {
        printf("ERROR: Destroy VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_RGA_DestroyChn(0);
    if (ret)
    {
        printf("ERROR: Destroy VI[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_VI_DisableChn(0, 1);
    if (ret)
    {
        printf("ERROR: Destroy VI[1] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_VI_DisableChn(0, 0);
    if (ret)
    {
        printf("ERROR: Destroy VI[0] error! ret=%d\n", ret);
        return 0;
    }

    if (pIqfilesPath)
    {
        SAMPLE_COMM_ISP_Stop(0);
    }
    return 0;
}