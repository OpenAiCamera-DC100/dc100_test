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


rockx_config_t rockx_configs;
rockx_object_array_t person_array;

rockx_handle_t person_det_handle;

int u32Width = 1920;
int u32Height = 1080;

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

static bool quit = false;
static void sigterm_handler(int sig) {
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

static void *PersonDetectionStream(void *data) {
    int ret;
    MEDIA_BUFFER scrBuf;
    cv::Mat *img = NULL;

    //create a person detection handle
    ret = rockx_create(&person_det_handle, ROCKX_MODULE_PERSON_DETECTION_V3, &rockx_configs, sizeof(rockx_config_t));
    if (ret != ROCKX_RET_SUCCESS) {
        printf("init rockx module ROCKX_MODULE_PERSON_DETCTION_V2 error %d\n", ret);
        return NULL;
    }

    rockx_image_t input_image;

    input_image.size = u32Width * u32Height * 3;
    input_image.width = u32Width;
    input_image.height = u32Height;
    input_image.pixel_format = ROCKX_PIXEL_FORMAT_RGB888;

    while(!quit) {
        scrBuf = DC_MPI_SYS_GetMediaBuffer(DC_ID_RGA, 0, -1); //RGB로 변환된 0번 카메라 버퍼 받아오기
        if (scrBuf == NULL) {
            printf("RGA buffer empty!\n");
            continue;
        }

        input_image.data = (uint8_t *)DC_MPI_MB_GetPtr(scrBuf);

        //detect
        ret = rockx_person_detect(person_det_handle, &input_image, &person_array, nullptr);
        if (ret != ROCKX_RET_SUCCESS) {
            printf("rockx_body_detect error %d\n", ret);
            return NULL;
        }
        img = new cv::Mat(cv::Size(u32Width, u32Height), CV_8UC3, (char *)DC_MPI_MB_GetPtr(scrBuf));
        for (int i = 0; i < person_array.count; i++) {
            if (person_array.object[i].score > 0.5) {
                cv::rectangle(*img, 
							cv::Point(person_array.object[i].box.left, person_array.object[i].box.top), 
							cv::Point(person_array.object[i].box.right, person_array.object[i].box.bottom), 
							cv::Scalar(0, 0, 255), 2);
                cv::putText(*img, std::to_string(person_array.object[i].score),
							cv::Point(person_array.object[i].box.left, person_array.object[i].box.top - 4),
							2, 1, cv::Scalar(255, 255, 255));
                printf("detect!\n");
            }
        }
        DC_MPI_SYS_SendMediaBuffer(DC_ID_RGA, 1, scrBuf);
        DC_MPI_MB_ReleaseBuffer(scrBuf);
    }
    rockx_destroy(person_det_handle);
    return NULL;
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

int main(int argc, char **argv) {
    int ret = 0;

    DC_CHAR *pIqfilesPath = (char *)"/etc/iqfiles";

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
    vi_chn_attr.u32Width = u32Width;
    vi_chn_attr.u32Height = u32Height;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;

    ret |= DC_MPI_VI_SetChnAttr(0, 0, &vi_chn_attr);
    ret |= DC_MPI_VI_EnableChn(0, 0);
    if (ret) {
        printf("Create VI[0] failed! ret=%d\n", ret);
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
    rga_chn_attr.stImgIn.u32Width = u32Width;       
    rga_chn_attr.stImgIn.u32Height = u32Height;     
    rga_chn_attr.stImgIn.u32HorStride = u32Width;   
    rga_chn_attr.stImgIn.u32VirStride = u32Height;  
    // Output image information 
    rga_chn_attr.stImgOut.u32X = 0;        
    rga_chn_attr.stImgOut.u32Y = 0;         
    rga_chn_attr.stImgOut.imgType = IMAGE_TYPE_RGB888; 
    rga_chn_attr.stImgOut.u32Width = u32Width;       
    rga_chn_attr.stImgOut.u32Height = u32Height;
    rga_chn_attr.stImgOut.u32HorStride = u32Width;
    rga_chn_attr.stImgOut.u32VirStride = u32Height;
    ret = DC_MPI_RGA_CreateChn(0, &rga_chn_attr);
    if (ret) {
        printf("Create RGA[0] failed! ret=%d\n", ret);
    }

    //0번 카메라 RGA채널 생성 (RTSP 스트리밍 위해 RGB888 -> NV12 변환) 
    memset(&rga_chn_attr, 0, sizeof(rga_chn_attr));
    rga_chn_attr.bEnBufPool = RK_TRUE;       
    rga_chn_attr.u16BufPoolCnt = 3;         
    rga_chn_attr.u16Rotaion = 0;            
    // input image information 
    rga_chn_attr.stImgIn.u32X = 0;           
    rga_chn_attr.stImgIn.u32Y = 0;           
    rga_chn_attr.stImgIn.imgType = IMAGE_TYPE_RGB888;   
    rga_chn_attr.stImgIn.u32Width = u32Width;       
    rga_chn_attr.stImgIn.u32Height = u32Height;     
    rga_chn_attr.stImgIn.u32HorStride = u32Width;   
    rga_chn_attr.stImgIn.u32VirStride = u32Height;  
    // Output image information 
    rga_chn_attr.stImgOut.u32X = 0;        
    rga_chn_attr.stImgOut.u32Y = 0;         
    rga_chn_attr.stImgOut.imgType = IMAGE_TYPE_NV12; 
    rga_chn_attr.stImgOut.u32Width = u32Width;       
    rga_chn_attr.stImgOut.u32Height = u32Height;
    rga_chn_attr.stImgOut.u32HorStride = u32Width;
    rga_chn_attr.stImgOut.u32VirStride = u32Height;
    ret = DC_MPI_RGA_CreateChn(1, &rga_chn_attr);
    if (ret) {
        printf("Create RGA[2] failed! ret=%d\n", ret);
    }

    //0번 카메라 VENC 채널
    VENC_CHN_ATTR_S venc_chn_attr;
	memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
	venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_NV12;
	venc_chn_attr.stVencAttr.u32PicWidth = u32Width;
	venc_chn_attr.stVencAttr.u32PicHeight = u32Height;
	venc_chn_attr.stVencAttr.u32VirWidth = u32Width;
	venc_chn_attr.stVencAttr.u32VirHeight = u32Height;
	venc_chn_attr.stVencAttr.u32Profile = 77;

    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
	venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265CBR;
	venc_chn_attr.stRcAttr.stH265Vbr.u32Gop = 30;
	venc_chn_attr.stRcAttr.stH265Vbr.u32MaxBitRate = u32Width * u32Height * 30 / 14;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.fr32DstFrameRateNum = 25;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateDen = 1;
	venc_chn_attr.stRcAttr.stH265Vbr.u32SrcFrameRateNum = 25;

	ret = DC_MPI_VENC_CreateChn(0, &venc_chn_attr);
    if (ret) {
        printf("ERROR: create VENC[0] error! ret=%d\n", ret);
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

    //RGA - VENC 바인드
    stSrcChn.enModId = DC_ID_RGA;
    stSrcChn.s32ChnId = 1;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind RGA[2]-VENC[0] failed! ret=%d\n", ret);
    }
    ret = DC_MPI_SYS_RegisterOutCb(&stDestChn, video_packet_cb); 
    if (ret) {
        printf("ERROR: register output callback for VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    //********************바인드 끝
    
    printf("%s initial finish\n", __func__);

    rockx_add_config(&rockx_configs, (char *)ROCKX_CONFIG_DATA_PATH, "/usr/share/rockx-data-rv1109/");

    pthread_t person_det_thread;
	pthread_create(&person_det_thread, NULL, PersonDetectionStream, NULL);
    
    signal(SIGINT, sigterm_handler);
    while (!quit) {
        usleep(500000);
    }

    rtsp_del_demo(g_rtsplive);
    pthread_join(person_det_thread, NULL);


    // unbind start
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32ChnId = 0;
    stDestChn.enModId = DC_ID_RGA;
    stDestChn.s32ChnId = 0;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Unbind VI[0] - RGA[0] failed! ret=%d\n", ret);
    }
    
    stSrcChn.enModId = DC_ID_RGA;
    stSrcChn.s32ChnId = 2;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32ChnId = 0;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Unbind RGA[2] -VENC[0] failed! ret=%d\n", ret);
    }
    
    stSrcChn.enModId = DC_ID_RGA;
    stSrcChn.s32ChnId = 3;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32ChnId = 1;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Unbind RGA[3] -VENC[1] failed! ret=%d\n", ret);
    }
    // unbind end

    // Destroy ch start
    DC_MPI_VENC_DestroyChn(0);
    DC_MPI_RGA_DestroyChn(1);
    DC_MPI_RGA_DestroyChn(0);
    DC_MPI_VI_DisableChn(0, 0);
    // Destroy ch end

    if (pIqfilesPath) {

        SAMPLE_COMM_ISP_Stop(0);

    }
}


