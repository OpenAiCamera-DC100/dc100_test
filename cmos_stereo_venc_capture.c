#include <getopt.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/time.h>

#include <rga/im2d.h>
#include <rga/rga.h>

#include "common/sample_common.h"
#include "dcmedia_api.h"
#include "librtsp/rtsp_demo.h"

#define CAM_NUM 2

static bool quit = false;
static char *g_save_path = "/tmp/disparity/";

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session_r;
static rtsp_session_handle g_rtsp_session_l;

void take_pictures_right_cb(MEDIA_BUFFER mb) {
    static DC_U32 jpeg_id = 0;
    printf("Get JPEG packet[%d]: ptr: %p, fd: %d, size: %zu, mode: %d, channel: %d, timestamp: %lld\n",
            jpeg_id, DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetFD(mb), DC_MPI_MB_GetSize(mb), DC_MPI_MB_GetModeID(mb),
            DC_MPI_MB_GetChannelID(mb), DC_MPI_MB_GetTimestamp(mb));
    
    char jpeg_path[64];
    sprintf(jpeg_path, "%sright%d.jpg", g_save_path, jpeg_id+1);
    FILE *file = fopen(jpeg_path, "w");
    if (file) {
        fwrite(DC_MPI_MB_GetPtr(mb), 1, DC_MPI_MB_GetSize(mb), file);
        fclose(file);
    }
    DC_MPI_MB_ReleaseBuffer(mb);
    jpeg_id++;
}

void take_pictures_left_cb(MEDIA_BUFFER mb) {
    static DC_U32 jpeg_id = 0;
    printf("Get JPEG packet[%d]: ptr: %p, fd: %d, size: %zu, mode: %d, channel: %d, timestamp: %lld\n",
            jpeg_id, DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetFD(mb), DC_MPI_MB_GetSize(mb), DC_MPI_MB_GetModeID(mb),
            DC_MPI_MB_GetChannelID(mb), DC_MPI_MB_GetTimestamp(mb));
    
    char jpeg_path[64];
    sprintf(jpeg_path, "%sleft%d.jpg", g_save_path, jpeg_id+1);
    FILE *file = fopen(jpeg_path, "w");
    if (file) {
        fwrite(DC_MPI_MB_GetPtr(mb), 1, DC_MPI_MB_GetSize(mb), file);
        fclose(file);
    }
    DC_MPI_MB_ReleaseBuffer(mb);
    jpeg_id++;
}

void video_packet_right_cb(MEDIA_BUFFER mb) { // Venc 데이터 정산, rtsp 전송 콜백
    static DC_S32 packet_cnt = 0;
    if (quit)
        return;

    //printf("####Get right-%d, size %zu\n", packet_cnt, DC_MPI_MB_GetSize(mb));

    if (g_rtsplive && g_rtsp_session_r) {
        rtsp_tx_video(g_rtsp_session_r, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb)); // RTSP 전송，수신 MB 는 데이터 가상 주소, 크기, 타임스탬프를 인코딩한 다음 전송합니다.
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

void video_packet_left_cb(MEDIA_BUFFER mb) { // Venc 데이터 정산, rtsp 전송 콜백
    static DC_S32 packet_cnt = 0;
    if (quit)
        return;

    //printf("#Get left-%d, size %zu\n", packet_cnt, DC_MPI_MB_GetSize(mb));

    if (g_rtsplive && g_rtsp_session_l) {
        rtsp_tx_video(g_rtsp_session_l, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb)); // RTSP 전송，수신 MB 는 데이터 가상 주소, 크기, 타임스탬프를 인코딩한 다음 전송합니다.
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

static void sigterm_handler(int sig) { 
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

int main(int argc, char *argv[]) {
    int ret = 0;
    int fps = 15;
    DC_U32 u32ScrWidth = 640; //카메라 입력 너비
    DC_U32 u32ScrHeight = 480; //카메라 입력 높이

    //IMAGE_TYPE_E enPixFmt = IMAGE_TYPE_NV12;  //입력 이미지 타입
    const DC_CHAR *pcVideoNode = "rkispp_scale0";  

    DC_BOOL bMultictx = DC_TRUE;

    char *iq_file_dir = "/etc/iqfiles/";
    
    signal(SIGINT, sigterm_handler);
    
    // ISP 이미지 센서 초기화
    if(iq_file_dir) {
        dc_aiq_working_mode_t hdr_mode = DC_AIQ_WORKING_MODE_NORMAL;
        for(int i = 0; i < CAM_NUM; i++){
            SAMPLE_COMM_ISP_Init(i, hdr_mode, bMultictx, iq_file_dir);
            SAMPLE_COMM_ISP_Run(i);
            SAMPLE_COMM_ISP_SetFrameRate(i, fps);
        }
    }

    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session_r = rtsp_new_session(g_rtsplive, "/live/right");
    g_rtsp_session_l = rtsp_new_session(g_rtsplive, "/live/left");
 
    rtsp_set_video(g_rtsp_session_r, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session_r, rtsp_get_reltime(), rtsp_get_ntptime()); //rtsp 현재 네트워크 시간 동기화
    rtsp_set_video(g_rtsp_session_l, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session_l, rtsp_get_reltime(), rtsp_get_ntptime()); //rtsp 현재 네트워크 시간 동기화

    //시스템 초기화
    ret = DC_MPI_SYS_Init();
    if(ret) {
        printf("Sys Init failed! ret = %d\n", ret);
        return -1;
    }

    //VI 채널 생성 및 활성화
    VI_CHN_ATTR_S vi_chn_attr;
    memset(&vi_chn_attr, 0, sizeof(vi_chn_attr));
    vi_chn_attr.pcVideoNode = pcVideoNode;
    vi_chn_attr.u32BufCnt = 3;
    vi_chn_attr.u32Width = u32ScrWidth;
    vi_chn_attr.u32Height = u32ScrHeight;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;

    for(int i = 0; i < CAM_NUM; i++) {
        ret = DC_MPI_VI_SetChnAttr(i, i, &vi_chn_attr);
        ret |= DC_MPI_VI_EnableChn(i, i);
        if (ret) {
            printf("Create VI[%d] failed! ret = %d\n", i, ret);
            return -1;
        }
    }

    printf("Create Venc(JPEG) start\n");
    //JPEG 인코더 채널 생성
    VENC_CHN_ATTR_S venc_chn_attr;
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_JPEG;
    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_NV12;
    venc_chn_attr.stVencAttr.u32PicWidth = u32ScrWidth;
    venc_chn_attr.stVencAttr.u32PicHeight = u32ScrHeight;
    venc_chn_attr.stVencAttr.u32VirWidth = u32ScrWidth;
    venc_chn_attr.stVencAttr.u32VirHeight = u32ScrHeight;
    //venc_chn_attr.stVencAttr.u32profile = 77;
    venc_chn_attr.stVencAttr.stAttrJpege.u32ZoomWidth = u32ScrWidth;
    venc_chn_attr.stVencAttr.stAttrJpege.u32ZoomHeight = u32ScrHeight;
    venc_chn_attr.stVencAttr.stAttrJpege.u32ZoomVirWidth = u32ScrWidth;
    venc_chn_attr.stVencAttr.stAttrJpege.u32ZoomVirHeight = u32ScrHeight;
    
    for(int i = 0; i < CAM_NUM; i++) {
        ret = DC_MPI_VENC_CreateChn(i, &venc_chn_attr);
        if (ret) {
            printf("Create VENC[%d](JPEG) failed! ret = %d\n", i, ret);
            return -1;
        }
    }

    //Venc 모듈 데이터 설정
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
    venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = 30;
    venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = u32ScrWidth * u32ScrHeight;
    // frame rate: in 30/1, out 30/1.
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = fps;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = fps;
    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_NV12;

    venc_chn_attr.stVencAttr.u32PicWidth = u32ScrWidth; 
    venc_chn_attr.stVencAttr.u32PicHeight = u32ScrHeight;
    venc_chn_attr.stVencAttr.u32VirWidth = u32ScrWidth;
    venc_chn_attr.stVencAttr.u32VirHeight = u32ScrHeight;
    venc_chn_attr.stVencAttr.u32Profile = 77;
    for(int i = 0; i < CAM_NUM; i++) {
        ret = DC_MPI_VENC_CreateChn(i + 2, &venc_chn_attr);
        if (ret) {
            printf("Create VENC[%d](RTSP) failed! ret = %d\n", i + 2, ret);
            return -1;
        }
    }

    printf("Create VENC ok\n");

    //JPEG 인코더 채널 절전모드로 전환
    VENC_RECV_PIC_PARAM_S stRecvParam;
    stRecvParam.s32RecvPicNum = 0;
    DC_MPI_VENC_StartRecvFrame(0, &stRecvParam);
    for(int i = 0; i < CAM_NUM; i++) {
        DC_MPI_VENC_StartRecvFrame(i, &stRecvParam);
    }

    MPP_CHN_S stEncChn;
    stEncChn.enModId = DC_ID_VENC;
    stEncChn.s32ChnId = 0;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, take_pictures_right_cb);
    if (ret) {
        printf("Register Output callback(take_pictures_right) failed! ret = %d\n", ret);
        return -1;
    }

    stEncChn.s32ChnId = 1;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, take_pictures_left_cb);
    if (ret) {
        printf("Register Output callback(take_pictures_left) failed! ret = %d\n", ret);
        return -1;
    }

    stEncChn.s32ChnId = 2;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_right_cb);
    if (ret) {
        printf("Register Output callback(rtsp_right) failed! ret = %d\n", ret);
        return -1;
    }

    stEncChn.s32ChnId = 3;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_left_cb);
    if (ret) {
        printf("Register Output callback(rtsp_left) failed! ret = %d\n", ret);
        return -1;
    }

    //bind
    MPP_CHN_S stSrcChn;
    MPP_CHN_S stDestChn;
    
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32ChnId = 0;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind VI[0] to VENC[0] failed! ret=%d\n", ret);
        return -1;
    }   

    stDestChn.s32ChnId = 2;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind VI[0] to VENC[2] failed! ret=%d\n", ret);
        return -1;
    } 

    stSrcChn.s32ChnId = 1;
    stDestChn.s32ChnId = 1;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind VI[1] to VENC[1] failed! ret=%d\n", ret);
        return -1;
    }  

    stDestChn.s32ChnId = 3;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("Bind VI[1] to VENC[3] failed! ret=%d\n", ret);
        return -1;
    } 
    //END bind

    char cmd[64];
    printf("#Usage: input 'quit' to exit programe!\n"
         "peress any other key to capture one picture to file\n");
    while(!quit) {
        fgets(cmd, sizeof(cmd), stdin);
        printf("#Input cmd: %s\n", cmd);
        if(strstr(cmd, "quit")) {
            printf("Get 'quit' cmd!\n");
            break;
        }

        stRecvParam.s32RecvPicNum = 1;
        for(int i = 0; i < CAM_NUM; i++) {
            ret = DC_MPI_VENC_StartRecvFrame(i, &stRecvParam);
            if (ret) {
                printf("DC_MPI_VENC_StartRecvFrame[%d] failed!\n", i);
                break;
            }
        }
        usleep(30000);
    }

    rtsp_del_demo(g_rtsplive);

    printf("%s exit!\n", __func__);
    stSrcChn.enModId = DC_ID_VI;
    stSrcChn.s32ChnId = 0;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32ChnId = 0;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);

    stDestChn.s32ChnId = 2;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);

    stSrcChn.s32ChnId = 1;
    stDestChn.s32ChnId = 1;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    
    stDestChn.s32ChnId = 3;
    DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);

   
    for(int i = 0; i < 4; i++) {
        DC_MPI_VENC_DestroyChn(i);
    }
    
    for(int i = 0; i < CAM_NUM; i++) {
        DC_MPI_VI_DisableChn(i, i);
    }

    for(int i = 0; i < CAM_NUM; i++) {
        SAMPLE_COMM_ISP_Stop(i);
    }

    return 0;
}
