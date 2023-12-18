// Copyright 2020 Fuzhou Rockchip Electronics Co., Ltd. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <assert.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <sys/stat.h>
#include <errno.h>

#include "common/sample_common.h"
#include "librtsp/rtsp_demo.h"
#include "dcmedia_api.h"

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

static bool quit = false;
static void sigterm_handler(int sig) {
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

void video_packet_cb(MEDIA_BUFFER mb) {
    static DC_S32 packet_cnt = 0;
    size_t buffSize;
    if (quit)
        return;

    buffSize = DC_MPI_MB_GetSize(mb);

    //printf("# CAM A: Get packet-%d, size %zu\n", packet_cnt, buffSize);

    if (g_rtsplive && g_rtsp_session) {
        rtsp_tx_video(g_rtsp_session, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb));
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

static DC_CHAR optstr[] = "?::a::w:h:c:e:d:I:M:";
static const struct option long_options[] = {
    {"aiq", optional_argument, NULL, 'a'},
    {"bitrate", optional_argument, NULL, 'b'},
    {"device_name", required_argument, NULL, 'd'},
    {"width", required_argument, NULL, 'w'},
    {"height", required_argument, NULL, 'h'},
    {"frame_cnt", required_argument, NULL, 'c'},
    {"encode", required_argument, NULL, 'e'},
    {"camid", required_argument, NULL, 'I'},
    {"multictx", required_argument, NULL, 'M'},
    {"help", optional_argument, NULL, '?'},
    {NULL, 0, NULL, 0},
};

static void print_usage(const DC_CHAR *name) {
    printf("usage example:\n");

    printf("\t%s [-a [iqfiles_dir]] [-w 1920] "
            "[-h 1080]"
            "[-d rkispp_scale0] "
             "[-e 0] "
            "[-I 0] "
            "[-M 0] "
            "\n",
            name);
    printf("\t-a | --aiq: enable aiq with dirpath provided, eg:-a "
            "/oem/etc/iqfiles/, "
            "set dirpath emtpty to using path by default, without this option aiq "
            "should run in other application\n");
    printf("\t-M | --multictx: switch of multictx in isp, set 0 to disable, set "
             "1 to enable. Default: 0\n");

    printf("\t%s [-w 1920] "
             "[-h 1080]"
             "[-I 0] "
            "[-d rkispp_scale0] "
            "[-e 0] "
            "\n",
            name);

    printf("\t-I | --camid: camera ctx id, Default 0\n");
    printf("\t-w | --width: VI width, Default:1920\n");
    printf("\t-h | --heght: VI height, Default:1080\n");
    printf("\t-d | --device_name set pcDeviceName, Default:rkispp_scale0, "
             "Option:[rkispp_scale0, rkispp_scale1, rkispp_scale2]\n");
    printf("\t-e | --encode: encode type, Default:h264, Value:h264, h265\n");
    printf("\t-b | --bitrate \n");
    printf("\t-c | --frame \n");
}

int main(int argc, char *argv[]) {
    DC_U32 u32Width = 1920;
    DC_U32 u32Height = 1080;
    DC_U32 bitrate = 5000000;
    DC_U32 frame = 30;
    DC_BOOL bMultictx = DC_FALSE;
    
    
    DC_CHAR *pDeviceName = "rkispp_scale0";
    
    DC_CHAR *pIqfilesPath = "/etc/iqfiles";
    CODEC_TYPE_E enCodecType = DC_CODEC_TYPE_H264;
    DC_CHAR *pCodecName = "H264";
    DC_S32 s32CamId = 0;

    int c;
    int ret = 0;
    while ((c = getopt_long(argc, argv, optstr, long_options, NULL)) != -1) {
        const char *tmp_optarg = optarg;
        switch (c) {
        case 'a':
            if (!optarg && NULL != argv[optind] && '-' != argv[optind][0]) {
                tmp_optarg = argv[optind++];
            }
            if (tmp_optarg) {
                pIqfilesPath = tmp_optarg;
            } else {
                pIqfilesPath = "/etc/iqfiles";
            }
            break;
        case 'w':
            u32Width = atoi(optarg);
            break;
        case 'h':
            u32Height = atoi(optarg);
            break;
        case 'd':
            pDeviceName = optarg;
            break;
        case 'b':
            bitrate = atoi(optarg);
            break;
        case 'c':
            frame = atoi(optarg);
            break;
        case 'e':
            if (!strcmp(optarg, "h264")) {
                enCodecType = DC_CODEC_TYPE_H264;
                pCodecName = "H264";
            } else if (!strcmp(optarg, "h265")) {
                enCodecType = DC_CODEC_TYPE_H265;
                pCodecName = "H265";
            } else {
                printf("ERROR: Invalid encoder type.\n");
                return 0;
            }
            break;
        case 'I':
            s32CamId = atoi(optarg);
            break;
        case 'M':
            if (atoi(optarg)) {
                bMultictx = DC_TRUE;
            }
            break;
        case '?':
        default:
            print_usage(argv[0]);
            return 0;
        }
    } 

    printf("#Device: %s\n", pDeviceName);
    printf("#CodecName:%s\n", pCodecName);
    printf("#Resolution: %dx%d\n", u32Width, u32Height);
    printf("#CameraIdx: %d\n\n", s32CamId);
    printf("#bMultictx: %d\n\n", bMultictx);
    printf("#Aiq xml dirpath: %s\n\n", pIqfilesPath);


    if (pIqfilesPath) {

        dc_aiq_working_mode_t hdr_mode = DC_AIQ_WORKING_MODE_NORMAL; 
        int fps = frame;

        SAMPLE_COMM_ISP_Init(s32CamId, hdr_mode, bMultictx, pIqfilesPath);
        SAMPLE_COMM_ISP_Run(s32CamId);
        SAMPLE_COMM_ISP_SetFrameRate(s32CamId, fps);
    }

    // init rtsp
    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");


 
    if (enCodecType == DC_CODEC_TYPE_H264) {
        rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    } else if (enCodecType == DC_CODEC_TYPE_H265) {
        rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
    } else {
        printf("not support other type\n");
        return -1;
    }
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime()); 

    DC_MPI_SYS_Init();

    VI_CHN_ATTR_S vi_chn_attr;
    vi_chn_attr.pcVideoNode = pDeviceName;
    vi_chn_attr.u32BufCnt = 3; 
    vi_chn_attr.u32Width = u32Width; 
    vi_chn_attr.u32Height = u32Height;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    ret = DC_MPI_VI_SetChnAttr(s32CamId, 0, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(s32CamId, 0);
    if (ret) {
        printf("ERROR: create VI[0] error! ret=%d\n", ret);
        return 0;
    }

    VENC_CHN_ATTR_S venc_chn_attr; 
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
    switch (enCodecType) {
    case DC_CODEC_TYPE_H265:
        venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H265; 
        venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = 30;
        //venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = u32Width * u32Height; 
        venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = bitrate; 
        // frame rate: in 30/1, out 30/1.
        venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = frame;
        venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = frame;
        break;
    case DC_CODEC_TYPE_H264:
    default:
        venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
        venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = 30;
        //venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = u32Width * u32Height;
        venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = bitrate;
        // frame rate: in 30/1, out 30/1.
        venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = frame;
        venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = frame;
        break;
    }
    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_NV12; 
    venc_chn_attr.stVencAttr.u32PicWidth = u32Width; 
    venc_chn_attr.stVencAttr.u32PicHeight = u32Height;
    venc_chn_attr.stVencAttr.u32VirWidth = u32Width;
    venc_chn_attr.stVencAttr.u32VirHeight = u32Height;
    venc_chn_attr.stVencAttr.u32Profile = 77;
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

    MPP_CHN_S stSrcChn;
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;
  
    MPP_CHN_S stDestChn;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("ERROR: Bind VI[0] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }


    printf("%s initial finish\n", __func__);
    signal(SIGINT, sigterm_handler);
    
    while (!quit) {
        usleep(500000);
    }

    printf("%s exit!\n", __func__);

    rtsp_del_demo(g_rtsplive);

    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;
  
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("ERROR: UnBind VI[0] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_VENC_DestroyChn(0);
    if (ret) {
        printf("ERROR: Destroy VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_VI_DisableChn(s32CamId, 0);
    if (ret) {
        printf("ERROR: Destroy VI[0] error! ret=%d\n", ret);
        return 0;
    }

    if (pIqfilesPath) {
        SAMPLE_COMM_ISP_Stop(s32CamId);
    }
    return 0;
}
