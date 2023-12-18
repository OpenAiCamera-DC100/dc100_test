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
#include "yolov5/postprocess.h"

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

// rknn
char *model_name = (char *)"/oem/usr/share/yolov5s_v6.2_output3_4.rknn";
const float nms_threshold = NMS_THRESH;
const float box_conf_threshold = BOX_THRESH;

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

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

static unsigned char *load_data(FILE *fp, size_t ofst, size_t sz)
{
    unsigned char *data;
    int ret;

    data = NULL;

    if (NULL == fp)
    {
        return NULL;
    }

    ret = fseek(fp, ofst, SEEK_SET);
    if (ret != 0)
    {
        printf("blob seek failure.\n");
        return NULL;
    }

    data = (unsigned char *)malloc(sz);
    if (data == NULL)
    {
        printf("buffer malloc failure.\n");
        return NULL;
    }
    ret = fread(data, 1, sz, fp);
    return data;
}

static unsigned char *load_model(const char *filename, int *model_size)
{
    FILE *fp;
    unsigned char *data;

    fp = fopen(filename, "rb");
    if (NULL == fp)
    {
        printf("Open file %s failed.\n", filename);
        return NULL;
    }

    fseek(fp, 0, SEEK_END);
    int size = ftell(fp);

    data = load_data(fp, 0, size);

    fclose(fp);

    *model_size = size;
    return data;
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

int nv12_border(char *pic, int pic_w, int pic_h, int rect_x, int rect_y,
                int rect_w, int rect_h, int R, int G, int B)
{
    /* Set up the rectangle border size */
    const int border = 5;

    /* RGB convert YUV */
    int Y, U, V;
    Y = 0.299 * R + 0.587 * G + 0.114 * B;
    U = -0.1687 * R + 0.3313 * G + 0.5 * B + 128;
    V = 0.5 * R - 0.4187 * G - 0.0813 * B + 128;
    /* Locking the scope of rectangle border range */
    int j, k;
    for (j = rect_y; j < rect_y + rect_h; j++)
    {
        for (k = rect_x; k < rect_x + rect_w; k++)
        {
            if (k < (rect_x + border) || k > (rect_x + rect_w - border) ||
                j < (rect_y + border) || j > (rect_y + rect_h - border))
            {
                /* Components of YUV's storage address index */
                int y_index = j * pic_w + k;
                int u_index =
                    (y_index / 2 - pic_w / 2 * ((j + 1) / 2)) * 2 + pic_w * pic_h;
                int v_index = u_index + 1;
                /* set up YUV's conponents value of rectangle border */
                pic[y_index] = Y;
                pic[u_index] = U;
                pic[v_index] = V;
            }
        }
    }

    return 0;
}

static void printRKNNTensor(rknn_tensor_attr *attr)
{
    printf("index=%d name=%s n_dims=%d dims=[%d %d %d %d] n_elems=%d size=%d "
           "fmt=%d type=%d qnt_type=%d fl=%d zp=%d scale=%f\n",
           attr->index, attr->name, attr->n_dims, attr->dims[3], attr->dims[2],
           attr->dims[1], attr->dims[0], attr->n_elems, attr->size, 0, attr->type,
           attr->qnt_type, attr->fl, attr->zp, attr->scale);
}

typedef struct node
{
    long timeval;
    detect_result_group_t detect_result_group;
    struct node *next;
} Node;

typedef struct my_stack
{
    int size;
    Node *top;
} rknn_list;

void create_rknn_list(rknn_list **s)
{
    if (*s != NULL)
        return;
    *s = (rknn_list *)malloc(sizeof(rknn_list));
    (*s)->top = NULL;
    (*s)->size = 0;
    printf("create rknn_list success\n");
}

void destory_rknn_list(rknn_list **s)
{
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

void rknn_list_push(rknn_list *s, long timeval,
                    detect_result_group_t detect_result_group)
{
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

void rknn_list_pop(rknn_list *s, long *timeval,
                   detect_result_group_t *detect_result_group)
{
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

void rknn_list_drop(rknn_list *s)
{
    Node *t = NULL;
    if (s == NULL || s->top == NULL)
        return;
    t = s->top;
    s->top = t->next;
    free(t);
    s->size--;
}

int rknn_list_size(rknn_list *s)
{
    if (s == NULL)
        return -1;
    return s->size;
}

rknn_list *rknn_list_;

static void *GetMediaBuffer(void *arg)
{
    printf("#Start %s thread, arg:%p\n", __func__, arg);

    rknn_context ctx;
    int ret;
    int model_len = 0;
    unsigned char *model;

    printf("Loading model ...\n");
    model = load_model(model_name, &model_len);
    ret = rknn_init(&ctx, model, model_len, 0);
    if (ret < 0)
    {
        printf("rknn_init fail! ret=%d\n", ret);
        return NULL;
    }

    rknn_sdk_version version;
    ret = rknn_query(ctx, RKNN_QUERY_SDK_VERSION, &version,
                     sizeof(rknn_sdk_version));
    if (ret < 0)
    {
        printf("rknn_query error ret=%d\n", ret);
        return NULL;
    }

    printf("sdk version: %s driver version: %s\n", version.api_version,
           version.drv_version);

    // Get Model Input Output Info
    rknn_input_output_num io_num;
    ret = rknn_query(ctx, RKNN_QUERY_IN_OUT_NUM, &io_num, sizeof(io_num));
    if (ret != RKNN_SUCC)
    {
        printf("rknn_query fail! ret=%d\n", ret);
        return NULL;
    }

    printf("model input num: %d, output num: %d\n", io_num.n_input, io_num.n_output);

    printf("input tensors:\n");
    rknn_tensor_attr input_attrs[io_num.n_input];
    memset(input_attrs, 0, sizeof(input_attrs));
    for (unsigned int i = 0; i < io_num.n_input; i++)
    {
        input_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_INPUT_ATTR, &(input_attrs[i]),
                         sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return NULL;
        }
        printRKNNTensor(&(input_attrs[i]));
        // dump_tensor_attr(&(input_attrs[i]));
    }

    printf("output tensors:\n");
    rknn_tensor_attr output_attrs[io_num.n_output];
    memset(output_attrs, 0, sizeof(output_attrs));
    for (unsigned int i = 0; i < io_num.n_output; i++)
    {
        output_attrs[i].index = i;
        ret = rknn_query(ctx, RKNN_QUERY_OUTPUT_ATTR, &(output_attrs[i]),
                         sizeof(rknn_tensor_attr));
        if (ret != RKNN_SUCC)
        {
            printf("rknn_query fail! ret=%d\n", ret);
            return NULL;
        }
        printRKNNTensor(&(output_attrs[i]));
        // dump_tensor_attr(&(output_attrs[i]));
    }

    int channel = 3;
    int width = 0;
    int height = 0;
    if (input_attrs[0].fmt == RKNN_TENSOR_NCHW)
    {
        printf("model is NCHW input fmt\n");
        width = input_attrs[0].dims[0];
        height = input_attrs[0].dims[1];
    }
    else
    {
        printf("model is NHWC input fmt\n");
        width = input_attrs[0].dims[1];
        height = input_attrs[0].dims[2];
    }

    printf("model input height=%d, width=%d, channel=%d\n", height, width, channel);

    MEDIA_BUFFER buffer = NULL;
    while (!quit)
    {
        buffer = DC_MPI_SYS_GetMediaBuffer(DC_ID_VI, 1, -1);
        if (!buffer)
        {
            continue;
        }

        int rga_buffer_size = SUB_WIDTH * SUB_HEIGHT * 3; // nv12 3/2, rgb 3
        int rga_buffer_model_input_size = RKNN_INPUT * RKNN_INPUT * 3;
        unsigned char *rga_buffer = (unsigned char *)malloc(rga_buffer_size);
        unsigned char *rga_buffer_model_input = (unsigned char *)malloc(rga_buffer_model_input_size);

        nv12_to_rgb24((unsigned char *)DC_MPI_MB_GetPtr(buffer), rga_buffer, SUB_WIDTH, SUB_HEIGHT);
        rgb24_resize(rga_buffer, rga_buffer_model_input, SUB_WIDTH, SUB_HEIGHT, RKNN_INPUT, RKNN_INPUT);

        // Set Input Data
        rknn_input inputs[1];
        memset(inputs, 0, sizeof(inputs));
        inputs[0].index = 0;
        inputs[0].type = RKNN_TENSOR_UINT8;
        inputs[0].size = rga_buffer_model_input_size;
        inputs[0].fmt = RKNN_TENSOR_NHWC;
        inputs[0].buf = rga_buffer_model_input;

        ret = rknn_inputs_set(ctx, io_num.n_input, inputs);
        if (ret < 0)
        {
            printf("rknn_input_set fail! ret=%d\n", ret);
            return NULL;
        }

        // Run
        // printf("rknn_run\n");
        ret = rknn_run(ctx, NULL);
        if (ret < 0)
        {
            printf("rknn_run fail! ret=%d\n", ret);
            return NULL;
        }

        // Get Output
        /*
        rknn_output outputs[2];
        memset(outputs, 0, sizeof(outputs));
        outputs[0].want_float = 1;
        outputs[1].want_float = 1;
        */

        rknn_output outputs[io_num.n_output];
        memset(outputs, 0, sizeof(outputs));
        for (int i = 0; i < io_num.n_output; i++)
        {
            outputs[i].want_float = 0;
        }

        ret = rknn_outputs_get(ctx, io_num.n_output, outputs, NULL);
        if (ret < 0)
        {
            printf("rknn_outputs_get fail! ret=%d\n", ret);
            return NULL;
        }

        // Post Process
        float scale_w = (float)width / MAIN_WIDHT;
        float scale_h = (float)height / MAIN_HEIGHT;

        detect_result_group_t detect_result_group;
        std::vector<float> out_scales;
        std::vector<uint32_t> out_zps;
        for (int i = 0; i < io_num.n_output; ++i)
        {
            out_scales.push_back(output_attrs[i].scale);
            out_zps.push_back(output_attrs[i].zp);
        }
        post_process((uint8_t *)outputs[0].buf, (uint8_t *)outputs[1].buf, (uint8_t *)outputs[2].buf, height, width,
                     box_conf_threshold, nms_threshold, scale_w, scale_h, out_zps, out_scales, &detect_result_group);
        // Release rknn_outputs
        rknn_outputs_release(ctx, 2, outputs);

        if (detect_result_group.count > 0)
        {
            rknn_list_push(rknn_list_, getCurrentTimeMsec(), detect_result_group);
            int size = rknn_list_size(rknn_list_);
            if (size >= MAX_RKNN_LIST_NUM)
                rknn_list_drop(rknn_list_);
            // printf("size is %d\n", size);
        }

        DC_MPI_MB_ReleaseBuffer(buffer);
        if (rga_buffer)
            free(rga_buffer);
        if (rga_buffer_model_input)
            free(rga_buffer_model_input);
    }
    // release
    if (ctx)
        rknn_destroy(ctx);
    if (model)
        free(model);

    return NULL;
}

static void *MainStream(void *arg)
{
    MEDIA_BUFFER buffer;
    cv::Mat *scr = NULL;

    while (!quit)
    {
        buffer = DC_MPI_SYS_GetMediaBuffer(DC_ID_RGA, 0, -1);
        if (!buffer)
        {
            continue;
        }

        scr = new cv::Mat(cv::Size(MAIN_WIDHT, MAIN_HEIGHT), CV_8UC3, (char *)DC_MPI_MB_GetPtr(buffer));

        if (rknn_list_size(rknn_list_))
        {
            char text[256];
            long time_before;
            detect_result_group_t detect_result_group;
            memset(&detect_result_group, 0, sizeof(detect_result_group));
            rknn_list_pop(rknn_list_, &time_before, &detect_result_group);
            // printf("time interval is %ld\n", getCurrentTimeMsec() - time_before);

            for (int i = 0; i < detect_result_group.count; i++)
            {
                //if (detect_result_group.results[i].prop < 4.0)
                //    continue;
                sprintf(text, "%s %.2f", detect_result_group.results[i].name, detect_result_group.results[i].prop);
                // printf("%s @ (%d %d %d %d) %f\n", detect_result_group.results[i].name, detect_result_group.results[i].box.left, detect_result_group.results[i].box.top,
                //        detect_result_group.results[i].box.right, detect_result_group.results[i].box.bottom, detect_result_group.results[i].prop);
                int x1 = detect_result_group.results[i].box.left;
                int y1 = detect_result_group.results[i].box.top;
                int x2 = detect_result_group.results[i].box.right;
                int y2 = detect_result_group.results[i].box.bottom;

                cv::rectangle(*scr, cv::Point(x1, y1 + 30), cv::Point(x2, y1), cv::Scalar(51, 0, 153), -1);
                cv::rectangle(*scr, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(51, 0, 153), 3);
                cv::putText(*scr, text, cv::Point(x1, y1), 0, 1, cv::Scalar(255, 255, 255), 2);
            }
        }
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
    printf("post process config: box_conf_threshold = %.2f, nms_threshold = %.2f\n",
           box_conf_threshold, nms_threshold);

    // init cam
    dc_aiq_working_mode_t hdr_mode = DC_AIQ_WORKING_MODE_NORMAL;
    int fps = 20;
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
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = 20;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = 20;

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

    create_rknn_list(&rknn_list_);

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