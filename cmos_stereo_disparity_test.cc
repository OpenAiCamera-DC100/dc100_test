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
#include <vector>
#include <sys/stat.h>
#include <errno.h>

#include "common/sample_common.h"
#include "librtsp/rtsp_demo.h"
#include "rkmedia_api.h"
#include "rkmedia_venc.h"

#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"

#define RTSP
#define CAM_NUM 2
//#define SGBM
#define BM

using namespace std;
using namespace cv;

#ifdef RTSP
rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;
#endif //RTSP

RK_U32 u32Width = 640;
RK_U32 u32Height = 480;
int cap = 0;

static bool quit = false;
static void sigterm_handler(int sig) {
    fprintf(stderr, "signal %d\n", sig);
    quit = true;
}

static void *DisparityStream(void *data) {
    int ret;
    MEDIA_BUFFER scrBuf[2];
    MEDIA_BUFFER dstBuf[2];
    MEDIA_BUFFER disparity;
    cv::Mat* scr[2] = {NULL,};
    cv::Mat* dst[2] = {NULL,};
    cv::Mat xyz;
#ifdef BM
    cv::Mat dst8u[2];
    Ptr<StereoBM> bm = StereoBM::create(64,13);
#endif

#ifdef SGBM
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);
#endif

    int numberOfDisparities = 64;
    int sgbmWinSize = 13;
    int cn = 3;
    float disparity_multiplier = 1.0f;
    float val;
    char text[20];
    Point2d pointxy(320, 240);

    Vec3f  point3;

    Mat disp, disp8;
    Mat disp8_3c;

    MB_IMAGE_INFO_S dstImage;   //dstBuffer의 이미지 정보
    dstImage.u32Width = u32Width;   
    dstImage.u32Height = u32Height;
    dstImage.u32HorStride = u32Width;
    dstImage.u32VerStride = u32Height;
    dstImage.enImgType = IMAGE_TYPE_BGR888;   

    Size img_size = {u32Width, u32Height};

    Rect roi1, roi2;
    Mat Q;

    // reading intrinsic parameters
    FileStorage fs("intrinsics.yml", FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file intrinsics.yml\n");
        //return;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    fs.open("extrinsics.yml", FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file extrinsics.yml\n");
        //return;
    }

    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);

#ifdef BM   
/*
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(31);
    bm->setBlockSize(sgbmWinSize);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(10);
    bm->setUniquenessRatio(15);
    bm->setSpeckleWindowSize(100);
    bm->setSpeckleRange(32);
    bm->setDisp12MaxDiff(1);
*/
    bm->setROI1(roi1);
    bm->setROI2(roi2);
    bm->setPreFilterCap(63); //31
    bm->setPreFilterSize(7);
    bm->setPreFilterType(cv::StereoBM::PREFILTER_XSOBEL);
    bm->setBlockSize(sgbmWinSize);
    bm->setMinDisparity(0);
    bm->setNumDisparities(numberOfDisparities);
    bm->setTextureThreshold(30);  //10
    bm->setUniquenessRatio(10);  //15
    bm->setSpeckleWindowSize(72);  //100
    bm->setSpeckleRange(32);  //32
    bm->setDisp12MaxDiff(-1);  //1
#endif

#ifdef SGBM
    sgbm->setPreFilterCap(63);
    sgbm->setBlockSize(sgbmWinSize);
    sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
    sgbm->setMinDisparity(0);
    sgbm->setNumDisparities(numberOfDisparities);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(StereoSGBM::MODE_SGBM);
#endif



    while(!quit) {   
        scrBuf[0] = RK_MPI_SYS_GetMediaBuffer(RK_ID_RGA, 0, -1);  //VI(캠) 버퍼 가져오기.
        scrBuf[1] = RK_MPI_SYS_GetMediaBuffer(RK_ID_RGA, 1, -1);
        dstBuf[0] = RK_MPI_MB_CreateImageBuffer(&dstImage, RK_TRUE, MB_FLAG_NOCACHED);
        dstBuf[1] = RK_MPI_MB_CreateImageBuffer(&dstImage, RK_TRUE, MB_FLAG_NOCACHED);
        
        if (scrBuf[0] == NULL) {
            printf("RGA[0] buffer empty!\n");
            continue;
        }
        if (scrBuf[1] == NULL) {
            printf("RGA[1] buffer empty!\n");
            continue;
        }

        scr[0] = new Mat(Size(u32Width, u32Height), CV_8UC3, (char *)RK_MPI_MB_GetPtr(scrBuf[0])); 
        scr[1] = new Mat(Size(u32Width, u32Height), CV_8UC3, (char *)RK_MPI_MB_GetPtr(scrBuf[1]));
        dst[0] = new Mat(Size(u32Width, u32Height), CV_8UC3, (char *)RK_MPI_MB_GetPtr(dstBuf[0]));
        dst[1] = new Mat(Size(u32Width, u32Height), CV_8UC3, (char *)RK_MPI_MB_GetPtr(dstBuf[1]));


        //remap(*scr[0], *dst[0], map11, map12, INTER_LINEAR);
        //remap(*scr[1], *dst[1], map21, map22, INTER_LINEAR);
        if(cap == 2) {
            imwrite("/tmp/color_l.jpg", *scr[1]);
            imwrite("/tmp/color_r.jpg", *scr[0]);
            printf("left right capture ok\n");
            cap = 1;
        }

        remap(*scr[1], *dst[1], map11, map12, INTER_LINEAR);
        remap(*scr[0], *dst[0], map21, map22, INTER_LINEAR);

#ifdef BM
        //(*dst[1]).convertTo(dst8u[1], CV_8U);
        //(*dst[0]).convertTo(dst8u[0], CV_8U);

        cvtColor(*dst[1], *dst[1], COLOR_BGR2GRAY);
        cvtColor(*dst[0], *dst[0], COLOR_BGR2GRAY);

        bm->compute(*dst[1], *dst[0], disp);
#endif

#ifdef SGBM       
        sgbm->compute(*dst[1], *dst[0], disp);
#endif
        if (disp.type() == CV_16S) {
            //printf("16S\n");
            disparity_multiplier = 16.0f;
        }
        
        for (int i = 0; i < 2; i++) {
            delete scr[i];
            delete dst[i];
        }
        
        reprojectImageTo3D(disp, xyz, Q, true);
        xyz = xyz * 16;
        point3 = xyz.at<Vec3f>(pointxy);
        val = point3[0] * point3[0] + point3[1] * point3[1] + point3[2] * point3[2];
        val = sqrt(val);
        val = val / 10.0;
        //cout << "depth:" << val << "cm" << endl;

        disp.convertTo(disp8, CV_8U, 255.0 / 1200);
        applyColorMap(disp8, disp8_3c, COLORMAP_TURBO);
        
        

	    snprintf(text, sizeof(text), "%.1f", val);

        if(val <= 120) {
            circle(disp8_3c, pointxy, 4, Scalar(255, 51, 51), -1, 8, 0);
            putText(disp8_3c, text, pointxy, FONT_HERSHEY_DUPLEX, 1, Scalar(255, 51, 51),2);
        }
        else {
            circle(disp8_3c, pointxy, 4, Scalar(51, 51, 255), -1, 8, 0);
            putText(disp8_3c, text, pointxy, FONT_HERSHEY_DUPLEX, 1, Scalar(51, 51, 255),2);
        }

        if(cap == 1) {
            imwrite("/tmp/color_d.jpg", disp8_3c);
            printf("disparity map capture ok\n");
            //imwrite("/tmp/wb.jpg", disp8);
            //printf("2\n");
            cap = 0;
        }

        disparity = RK_MPI_MB_CreateImageBuffer(&dstImage, RK_TRUE, MB_FLAG_NOCACHED);
        dst[0] = new Mat(Size(u32Width, u32Height), CV_8UC3, (char *)RK_MPI_MB_GetPtr(disparity));
        disp8_3c.copyTo(*dst[0]);
        
        delete dst[0];      
        
        RK_MPI_SYS_SendMediaBuffer(RK_ID_RGA, 2, disparity);
        //RK_MPI_SYS_SendMediaBuffer(RK_ID_VENC, 0, disparity);
        //RK_MPI_SYS_SendMediaBuffer(RK_ID_VENC, 0, scrBuf[0]);
        
        for(int i = 0; i < CAM_NUM; i++) {
            RK_MPI_MB_ReleaseBuffer(scrBuf[i]);
            RK_MPI_MB_ReleaseBuffer(dstBuf[i]);
        }
        RK_MPI_MB_ReleaseBuffer(disparity);
    }
	return NULL;
}

void video_packet_cb(MEDIA_BUFFER mb) { // Venc 데이터 정산, rtsp 전송 콜백
    static RK_S32 packet_cnt = 0;
    if (quit)
        return;

    //printf("#Get packet-%d, size %zu\n", packet_cnt, RK_MPI_MB_GetSize(mb));

    if (g_rtsplive && g_rtsp_session) {
        rtsp_tx_video(g_rtsp_session, (uint8_t const*)RK_MPI_MB_GetPtr(mb), RK_MPI_MB_GetSize(mb),
                      RK_MPI_MB_GetTimestamp(mb)); // RTSP 전송，수신 MB 는 데이터 가상 주소, 크기, 타임스탬프를 인코딩한 다음 전송합니다.
        rtsp_do_event(g_rtsplive);
    }

    RK_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}


static RK_CHAR optstr[] = "?::a::w:h:c:e:d:I:M:";
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

static void print_usage(const RK_CHAR *name) {
    printf("usage example:\n");
#ifdef RKAIQ
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
#else
    printf("\t%s [-w 1920] "
             "[-h 1080]"
             "[-I 0] "
            "[-d rkispp_scale0] "
            "[-e 0] "
            "\n",
            name);
#endif
    printf("\t-I | --camid: camera ctx id, Default 0\n");
    printf("\t-w | --width: VI width, Default:1920\n");
    printf("\t-h | --heght: VI height, Default:1080\n");
    printf("\t-d | --device_name set pcDeviceName, Default:rkispp_scale0, "
             "Option:[rkispp_scale0, rkispp_scale1, rkispp_scale2]\n");
    printf("\t-e | --encode: encode type, Default:h264, Value:h264, h265\n");
}

int main(int argc, char *argv[]) {
   
    RK_U32 bitrate = 5000000;
    RK_U32 frame = 30;
    
    RK_CHAR *pDeviceName = (char *)"rkispp_scale0";
    
    RK_CHAR *pIqfilesPath = NULL;
    CODEC_TYPE_E enCodecType = RK_CODEC_TYPE_H264;
    RK_CHAR *pCodecName = (char *)"H265";
    RK_S32 s32CamId = 0;
#ifdef RKAIQ
    //RK_BOOL bMultictx = RK_FALSE;
    RK_BOOL bMultictx = RK_TRUE;
#endif
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
                pIqfilesPath = (char *)tmp_optarg;
            } else {
                pIqfilesPath = (char *)"/etc/iqfiles";
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
                enCodecType = RK_CODEC_TYPE_H264;
                pCodecName = (char *)"H264";
            } else if (!strcmp(optarg, "h265")) {
                enCodecType = RK_CODEC_TYPE_H265;
                pCodecName = (char *)"H265";
            } else {
                printf("ERROR: Invalid encoder type.\n");
                return 0;
            }
            break;
        case 'I':
            s32CamId = atoi(optarg);
            break;
#ifdef RKAIQ
        case 'M':
            if (atoi(optarg)) {
                bMultictx = RK_TRUE;
            }
            break;
#endif
        case '?':
        default:
            print_usage(argv[0]);
            return 0;
        }
    } 

    //printf("#Device: %s\n", pDeviceName);
    printf("#CodecName:%s\n", pCodecName);
    printf("#Resolution: %dx%d\n", u32Width, u32Height);
    printf("#CameraIdx: %d\n\n", s32CamId);
#ifdef RKAIQ
    printf("#bMultictx: %d\n\n", bMultictx);
    printf("#Aiq xml dirpath: %s\n\n", pIqfilesPath);
#endif

    if (pIqfilesPath) { // RKAIQ iqfile 경로
#ifdef RKAIQ //RKAIQ 초기화 RKMedia는 RKAIQ에 의존합니다. 이 초기화가 다른 프로세스에서 수행되면 다시 초기화 할 필요가 없습니다. 다시 초기화 할 경우 오류가 보고됩니다.
        rk_aiq_working_mode_t hdr_mode = RK_AIQ_WORKING_MODE_NORMAL; 
        int fps = frame;

        for(int i = 0; i < CAM_NUM; i++) {
            SAMPLE_COMM_ISP_Init(i, hdr_mode, bMultictx, pIqfilesPath);
            SAMPLE_COMM_ISP_Run(i);
            SAMPLE_COMM_ISP_SetFrameRate(i, fps);
        }
#endif
    }

#ifdef RTSP
    // init rtsp
    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session= rtsp_new_session(g_rtsplive, "/live/main_stream");
#endif //RTSP


#ifdef RTSP  
    if (enCodecType == RK_CODEC_TYPE_H264) { //RTSP 스트리밍 유형 선택
        rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    } else if (enCodecType == RK_CODEC_TYPE_H265) {
        rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H265, NULL, 0);
    } else {
        printf("not support other type\n");
        return -1;
    }
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime()); //rtsp 현재 네트워크 시간 동기화
#endif //RTSP

    RK_MPI_SYS_Init();//RKMedia MPI 시스템 초기화

    //VI 모듈 데이터 설정
    VI_CHN_ATTR_S vi_chn_attr;
    vi_chn_attr.pcVideoNode = pDeviceName;
    vi_chn_attr.u32BufCnt = 3; 
    vi_chn_attr.u32Width = u32Width; 
    vi_chn_attr.u32Height = u32Height;
    vi_chn_attr.enPixFmt = IMAGE_TYPE_NV12;
    vi_chn_attr.enBufType = VI_CHN_BUF_TYPE_MMAP;
    vi_chn_attr.enWorkMode = VI_WORK_MODE_NORMAL;

    for(int i = 0; i < CAM_NUM; i++) {
        ret = RK_MPI_VI_SetChnAttr(i, i, &vi_chn_attr);
        ret |= RK_MPI_VI_EnableChn(i, i);
        if (ret) {
            printf("ERROR: create VI[%d] error! ret=%d\n", i, ret);
            return 0;
        }
    }

    RGA_ATTR_S stRgaAttr;
    memset(&stRgaAttr, 0, sizeof(stRgaAttr));
    stRgaAttr.bEnBufPool = RK_TRUE;       //Enable buffer pool.
    stRgaAttr.u16BufPoolCnt = 3;          //Buffer pool count.
    stRgaAttr.u16Rotaion = 0;            //Rotation angle. Values : 0, 90, 180, 270.
    // input image information 
    stRgaAttr.stImgIn.u32X = 0;           //X-axis coordinate of camera.
    stRgaAttr.stImgIn.u32Y = 0;           //Y-axis coordinate of camera.
    stRgaAttr.stImgIn.imgType = IMAGE_TYPE_NV12;    //Image format type.
    stRgaAttr.stImgIn.u32Width = u32Width;       //The width of RGA.
    stRgaAttr.stImgIn.u32Height = u32Height;     //The height of RGA.
    stRgaAttr.stImgIn.u32HorStride = u32Width;   //Virture width.
    stRgaAttr.stImgIn.u32VirStride = u32Height;  //Virture height.
    // Output image information 
    stRgaAttr.stImgOut.u32X = 0;          //X-axis coordinate of RGA.
    stRgaAttr.stImgOut.u32Y = 0;          //Y-axis coordinate of RGA.

    stRgaAttr.stImgOut.imgType = IMAGE_TYPE_RGB888; //Image format type.

    stRgaAttr.stImgOut.u32Width = u32Width;       //The width of RGA.
    stRgaAttr.stImgOut.u32Height = u32Height;
    stRgaAttr.stImgOut.u32HorStride = u32Width;
    stRgaAttr.stImgOut.u32VirStride = u32Height;
    
    for(int i = 0; i < CAM_NUM; i++) {
        ret = RK_MPI_RGA_CreateChn(i, &stRgaAttr);
        if (ret) {
            printf("Create RGA[%d] failed! ret=%d\n", i, ret);
        }
    }

    memset(&stRgaAttr, 0, sizeof(stRgaAttr));
    stRgaAttr.bEnBufPool = RK_TRUE;       //Enable buffer pool.
    stRgaAttr.u16BufPoolCnt = 3;          //Buffer pool count.
    stRgaAttr.u16Rotaion = 0;            //Rotation angle. Values : 0, 90, 180, 270.
    // input image information 
    stRgaAttr.stImgIn.u32X = 0;           //X-axis coordinate of camera.
    stRgaAttr.stImgIn.u32Y = 0;           //Y-axis coordinate of camera.
    stRgaAttr.stImgIn.imgType = IMAGE_TYPE_BGR888;    //Image format type.
    stRgaAttr.stImgIn.u32Width = u32Width;       //The width of RGA.
    stRgaAttr.stImgIn.u32Height = u32Height;     //The height of RGA.
    stRgaAttr.stImgIn.u32HorStride = u32Width;   //Virture width.
    stRgaAttr.stImgIn.u32VirStride = u32Height;  //Virture height.
    // Output image information 
    stRgaAttr.stImgOut.u32X = 0;          //X-axis coordinate of RGA.
    stRgaAttr.stImgOut.u32Y = 0;          //Y-axis coordinate of RGA.
    stRgaAttr.stImgOut.imgType = IMAGE_TYPE_NV12; //Image format type.
    stRgaAttr.stImgOut.u32Width = u32Width;       //The width of RGA.
    stRgaAttr.stImgOut.u32Height = u32Height;
    stRgaAttr.stImgOut.u32HorStride = u32Width;
    stRgaAttr.stImgOut.u32VirStride = u32Height;
    
    ret = RK_MPI_RGA_CreateChn(2, &stRgaAttr);
    if (ret) {
        printf("Create RGA[2] failed! ret=%d\n", ret);
    }


    //Venc 모듈 데이터 설정
    VENC_CHN_ATTR_S venc_chn_attr; 
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
    switch (enCodecType) {
    case RK_CODEC_TYPE_H265:
        venc_chn_attr.stVencAttr.enType = RK_CODEC_TYPE_H265; 
        venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H265VBR;
        venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = 30;
        venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = u32Width * u32Height; 
        //venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = bitrate; 
        // frame rate: in 30/1, out 30/1.
        venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = frame;
        venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
        venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = frame;
        break;
    case RK_CODEC_TYPE_H264:
    default:
        venc_chn_attr.stVencAttr.enType = RK_CODEC_TYPE_H264;
        venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
        venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = 30;
        venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = u32Width * u32Height;
        //venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = bitrate;
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
    ret = RK_MPI_VENC_CreateChn(0, &venc_chn_attr);
    if (ret) {
        printf("ERROR: create VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    MPP_CHN_S stEncChn; 
    stEncChn.enModId = RK_ID_VENC;
    stEncChn.s32DevId = 0;
    stEncChn.s32ChnId = 0;
    ret = RK_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_cb); 
    if (ret) {
        printf("ERROR: register output callback for VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    // VI 
    MPP_CHN_S stSrcChn;
    stSrcChn.enModId = RK_ID_VI;

    MPP_CHN_S stDestChn;
    stDestChn.enModId = RK_ID_RGA;

    for(int i = 0; i < CAM_NUM; i++) {
        stSrcChn.s32ChnId = i;
        stDestChn.s32ChnId = i;
        
        ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);
        if (ret) {
            printf("ERROR: Bind VI[%d] and RGA[%d] error! ret=%d\n", i, i, ret);
            return 0;
        }    
    }
    
    stSrcChn.enModId = RK_ID_RGA;
    stSrcChn.s32ChnId = 2;
    stDestChn.enModId = RK_ID_VENC;
    stDestChn.s32ChnId = 0;

    ret = RK_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("ERROR: Bind RGA[2] and VENC[0] error! ret=%d\n", ret);
        return 0;
    } 
    
    
    pthread_t disparity_thread;
    pthread_create(&disparity_thread, NULL, DisparityStream, NULL);

    printf("%s initial finish\n", __func__);
    signal(SIGINT, sigterm_handler);
    
    char cmd[64];
    while (!quit) {
        fgets(cmd, sizeof(cmd), stdin);
        printf("#Input cmd: %s\n", cmd);
        if(strstr(cmd, "c")) {
            cap = 2;
        }

        if(strstr(cmd, "quit")) {
            printf("Get 'quit' cmd!\n");
            break;
        }

        usleep(500000);
    }

    printf("%s exit!\n", __func__);


#ifdef RTSP
    rtsp_del_demo(g_rtsplive);
#endif
    // unbind first
    stSrcChn.enModId = RK_ID_VI;
    stSrcChn.s32DevId = 0;
  
    stDestChn.enModId = RK_ID_RGA;
    stDestChn.s32DevId = 0;

    for(int i = 0; i < CAM_NUM; i++ ) {
        stSrcChn.s32ChnId = i;
        stDestChn.s32ChnId = i;
        ret = RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
        if (ret) {
            printf("ERROR: UnBind VI[%d] and RGA[%d] error! ret=%d\n", i, i, ret);
            return 0;
        }
    }
    
    stSrcChn.enModId = RK_ID_RGA;
    stSrcChn.s32ChnId = 2;
    stDestChn.enModId = RK_ID_VENC;
    stDestChn.s32ChnId = 0;
    
    ret = RK_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("ERROR: UnBind RGA[2] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }
    
    // destroy venc before vi
    ret = RK_MPI_VENC_DestroyChn(0);
    if (ret) {
        printf("ERROR: Destroy VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    // destroy vi
    for(int i = 0; i < CAM_NUM; i++) {
        ret = RK_MPI_VI_DisableChn(i, i);
        if (ret) {
            printf("ERROR: Destroy VI[%d] error! ret=%d\n", i, ret);
            return 0;
        }
    }

    if (pIqfilesPath) {
#ifdef RKAIQ
    SAMPLE_COMM_ISP_Stop(0);
    SAMPLE_COMM_ISP_Stop(1);
#endif
    }
    return 0;
}
