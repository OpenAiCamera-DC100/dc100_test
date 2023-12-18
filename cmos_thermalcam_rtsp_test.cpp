#include <stdio.h>
#include <signal.h>
#include <unistd.h>

#include "USBSDK.h"

#include "dcmedia_api.h"
#include "librtsp/rtsp_demo.h"

#include <opencv2/videoio/videoio.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;

MB_IMAGE_INFO_S dstImage;   //dstBuffer의 이미지 정보

unsigned short camera_data[2560 * 1024];
unsigned short temp_data_temp[1280 * 1024];
unsigned short temp_data[1280 * 1024];
float fTempValue = 10.0;

int iTempMeasType;
int iTEn;
bool bIfNewNios = false;

int iTempUnit = 0;//0:Celsius; 1:Kelvin; 2:Fahrenheit;
int iCoreType = 0;//1: LT Temperature measurement type     2??MicroIII Temperature measurement type    3??MicroIII Imaging

int width = 640;
int height = 512;

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

    printf("# CAM A: Get packet-%d, size %zu\n", packet_cnt, buffSize);

    if (g_rtsplive && g_rtsp_session) {
        rtsp_tx_video(g_rtsp_session, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb),
                      DC_MPI_MB_GetTimestamp(mb));
        rtsp_do_event(g_rtsplive);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

void VideoCB(unsigned char *pBuffer, int Width, int Height, void *pContext) {
    //Mat rgb;
    //Mat mat;
    Mat *scr = NULL;
    Mat *dst = NULL;


    //printf("W: %d  H: %d\n",Width, Height);

    //IplImage * pImg = NULL;
    MEDIA_BUFFER dstBuffer;
    //IplImage * pImgGray2 = NULL;
    dstBuffer = DC_MPI_MB_CreateImageBuffer(&dstImage, DC_TRUE, MB_FLAG_NOCACHED);

    //pImg = cvCreateImage(cvSize(Width, Height), 8, 2);
    //memcpy(pImg->imageData, pBuffer, Width*Height * 2);
    //pImgGray2 = cvCreateImage(cvGetSize(pImg), 8, 3);
    scr = new Mat(Size(Width, Height), CV_8UC2, pBuffer);  //YUYV
    dst = new Mat(Size(Width, Height), CV_8UC3, (char *)DC_MPI_MB_GetPtr(dstBuffer));

    cvtColor(*scr, *dst, CV_YUV2BGR_UYVY);
    //cvtColor(*scr, *dst, CV_YUV2BGR_YUYV);

    
    int iTemp = (int)temp_data[256 * Width + 320];
    float fK = iTemp / fTempValue;
    float fC = fK - 273.15;
    float fF = (fC * 9 / 5) + 32;
    
    char str[64];
    if (iTempUnit == 0) {
        sprintf(str, "%.2f", fC);
    }
    else if (iTempUnit == 1) {
        sprintf(str, "%.2f", fK);
    }
    else if (iTempUnit == 2) {
        sprintf(str, "%.2f", fF);
    }
    cv::putText(*dst, str, cv::Point(320, 256), CV_FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(255, 0, 0), 2);

    //mat = cvarrToMat(pImgGray2);
    delete scr;
    delete dst;
    DC_MPI_SYS_SendMediaBuffer(DC_ID_VENC, 0, dstBuffer);
    DC_MPI_MB_ReleaseBuffer(dstBuffer);
}

void TempCB(unsigned char *pBuffer, int Width, int Height, void* pContext) {
    memcpy(&camera_data[0], pBuffer, Width * Height * 2);
    for (int j = 0; j < Height; j++) {
        for (int i = 0; i < Width; i++) {
            //�ж��¶�����0~7300�����¶Ȼ��㹫ʽ������Ϊ���϶ȣ�(Value+7000)/30-273.2
            //7301~16383�� (Value-3300)/15-273.2
            if (iTEn&&bIfNewNios) {
                temp_data_temp[j * Width + i] = camera_data[j * Width + i];

                if (temp_data_temp[j * Width + i] > 7300) {  	//7301~16383�� (Value-3300)/15-273.2
                    fTempValue = 15.0;
                    temp_data_temp[j * Width + i] = camera_data[j * Width + i] - 3300;
                }
                else { //0~7300�����¶Ȼ��㹫ʽ������Ϊ���϶ȣ�(Value + 7000) / 30 - 273.2
                    fTempValue = 30.0;
                    temp_data_temp[j * Width + i] = camera_data[j * Width + i] + 7000;
                }
            }
            else {
                temp_data_temp[j * Width + i] = camera_data[j * Width + i];
                fTempValue = 10.0;
                
            }
        }
    }
    //printf("TempCB test\n");
    memcpy(&temp_data[0], temp_data_temp, Width * Height * 2);

}

int TempRead(IRNETHANDLE handle, unsigned char* p8value) {
    int ret;
    //prepare data
    unsigned char szCmd[0x09] = {0xAA, 0x05, 0x07, 0x71, 0x00, 0x00, 0x27, 0xEB, 0xAA};

    //send Request
    ret = WriteHandle(handle, (char*)szCmd, 0x09);
    if (ret != 0) {
        return ret;
    }

    usleep(60000);

    //receive data
    int recvCnt = 0;
    unsigned char recvBuf[MAX_LOCAL_BUF_LEN] = {0};
    ret = ReadHandle(handle, (char*)recvBuf, &recvCnt);
    if (ret != 0) {
        return ret;
    }

    //parse data
    if (recvCnt > 3 && 
        recvBuf[0] == 0x55 && 
        recvBuf[1] == recvCnt && 
        recvBuf[2] == 0x07 && 
        recvBuf[3] == (unsigned char)0x71 && 
        recvBuf[4] == 0x33)
    {
        *p8value = recvBuf[5];
        return 0;
    }
    else 
    {
        if (recvCnt > 3 &&
            recvBuf[0] == 0x55 &&
            recvBuf[1] == 0x05 &&
            recvBuf[2] == 0x07 &&
            recvBuf[3] == 0xF0 &&
            recvBuf[4] == 0x33)
        {
            *p8value = 2; //??��????55 05 07 F0 33 00 sum EB AA????????????????????
            return 0;
        }
        else
        {
            return -1;
        }
    }
}

int main(int argc, char **argv) {
    int ret;
    bool bret;

    unsigned char u8value = 0;
    unsigned char  p8value;

    unsigned char ucTemp = 0;

    int iPalette = 0;

    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");
    rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime()); 

    dstImage.u32Width = width;   
	dstImage.u32Height = height;
	dstImage.u32HorStride = width;
	dstImage.u32VerStride = height;
	dstImage.enImgType = IMAGE_TYPE_RGB888;   //openCV는 BGR 타잎 사용.
    
    DC_MPI_SYS_Init();
    //rga 채널 생성
    /*
    RGA_ATTR_S stRgaAttr;
    memset(&stRgaAttr, 0, sizeof(stRgaAttr));
    stRgaAttr.bEnBufPool = RK_TRUE;       //Enable buffer pool.
    stRgaAttr.u16BufPoolCnt = 3;          //Buffer pool count.
    stRgaAttr.u16Rotaion = 0;            //Rotation angle. Values : 0, 90, 180, 270.
    // input image information 
    stRgaAttr.stImgIn.u32X = 0;           //X-axis coordinate of camera.
    stRgaAttr.stImgIn.u32Y = 0;           //Y-axis coordinate of camera.
    stRgaAttr.stImgIn.imgType = IMAGE_TYPE_RGB888;    //Image format type.
    stRgaAttr.stImgIn.u32Width = width;       //The width of RGA.
    stRgaAttr.stImgIn.u32Height = height;     //The height of RGA.
    stRgaAttr.stImgIn.u32HorStride = width;   //Virture width.
    stRgaAttr.stImgIn.u32VirStride = height;  //Virture height.
    // Output image information 
    stRgaAttr.stImgOut.u32X = 0;          //X-axis coordinate of RGA.
    stRgaAttr.stImgOut.u32Y = 0;          //Y-axis coordinate of RGA.
    stRgaAttr.stImgOut.imgType = IMAGE_TYPE_RGB888; //Image format type.
    stRgaAttr.stImgOut.u32Width = width;       //The width of RGA.
    stRgaAttr.stImgOut.u32Height = height;
    stRgaAttr.stImgOut.u32HorStride = width;
    stRgaAttr.stImgOut.u32VirStride = height;
    ret = DC_MPI_RGA_CreateChn(0, &stRgaAttr);
    if (ret) {
        printf("Create RGA[0] failed! ret=%d\n", ret);
    }
    */
    VENC_CHN_ATTR_S venc_chn_attr; 
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));

    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264;
    venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    venc_chn_attr.stRcAttr.stH264Cbr.u32Gop = 30;
    //venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = u32Width * u32Height;
    venc_chn_attr.stRcAttr.stH264Cbr.u32BitRate = 500000;
    // frame rate: in 30/1, out 30/1.
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.fr32DstFrameRateNum = 30;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH264Cbr.u32SrcFrameRateNum = 30;

    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_RGB888; 
    venc_chn_attr.stVencAttr.u32PicWidth = width; 
    venc_chn_attr.stVencAttr.u32PicHeight = height;
    venc_chn_attr.stVencAttr.u32VirWidth = width;
    venc_chn_attr.stVencAttr.u32VirHeight = height;
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
    /*
    MPP_CHN_S stSrcChn;
    stSrcChn.enModId = DC_ID_RGA;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;
  
    MPP_CHN_S stDestChn;
    stDestChn.enModId = DC_ID_VENC;
    stDestChn.s32DevId = 0;
    stDestChn.s32ChnId = 0;
    ret = DC_MPI_SYS_Bind(&stSrcChn, &stDestChn);
    if (ret) {
        printf("ERROR: Bind RGA[0] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }
    */
    IRNETHANDLE handle;
    DeviceLst devList;

    handle = sdk_create();
    
    ret = sdk_loginDevice(handle);
    if(ret < 0 ) {
        printf("sdk init fail!!\n");
        return -1;
    }

    ret = SearchDevice(handle, devList);
    if(ret < 0 ) {
        printf("device serch fail!!\n");
        return -1;
    }
    printf("==========  Device List  ==========\n");
    printf("Number of serial ports : %d\n", devList.iComCount);
    printf("Device count : %d\n", devList.iNumber);
    for(int i = 0; i < MAX_DEVICE_NUM; i++) {
        printf("Device ID : %d\n", devList.DevInfo[i].id);
    }
    for(int i = 0; i < MAX_DEVICE_NUM; i++) {
        printf("Device name : %s\n", devList.DevInfo[i].cName);
    }
    for(int i = 0; i < MAX_DEVICE_NUM; i++) {
        printf("Com name : %s\n", devList.ComNameInfo[i].cComPort);
    }
    printf("==========  ===========  ==========\n");

    SetVideoCallBack(handle, VideoCB, NULL);
    SetTempCallBack(handle, TempCB, NULL);

    bret = OpenDevice(handle, 1, 0);
    if (bret == false) {
        printf("open device fail!!\n");
        return -1;
    }

    iCoreType = CoreType(handle);
    iTempMeasType = TempMeasureType(handle);
    printf("iCoreType: %d, iTempMeasType: %d\n", iCoreType, iTempMeasType);

    for (int i = 0; i < 2; i++) {
        ret = TempRead(handle, &p8value);
        if (ret == 0) {
            break;       
        }
    }
    iTEn = p8value;
    printf("iTEn: %d\n", iTEn);
    if (iTEn == 2) {
        bIfNewNios = false;
    }
    else {
        bIfNewNios = true;
    }
    //온도 단위 확인
    for (int i = 0; i < 10; i++) {
        ret = sdk_read_temp_unit(handle, &ucTemp);
        if (ret == 0){
            break;
        }
    }
    iTempUnit = ucTemp;  //0 섭씨, 1 켈빈, 2 화씨
    printf("Temp unit: %d\n", iTempUnit);

    for (int i = 0; i < 10; i++)
    {
        ret = sdk_get_color_plate(handle, iCoreType, &iPalette);
        if (ret == 0)
        {
            break;
        }
    }
    printf("Color plate: %d\n", iPalette);

    signal(SIGINT, sigterm_handler);
    while(!quit) {

        usleep(30000);
    }

    rtsp_del_demo(g_rtsplive);

    //DC_MPI_SYS_UnBind(&stSrcChn, &stDestChn);
    DC_MPI_VENC_DestroyChn(0);
    //DC_MPI_RGA_DestroyChn(0);
    

    CloseDevice(handle);

    ReleaseSDK(handle); 

    return 0;
}