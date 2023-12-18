//Note:DCAM550 has no RGB frame and no use DCAM_305 in Vzense_dcamtype.h

#include <iostream>
#include <fstream>
#include <unistd.h>

#include <opencv2/opencv.hpp>

#include "Vzense_api_550.h"
#include <thread>
#include <sys/timeb.h>

#include "librtsp/rtsp_demo.h"
#include "dcmedia_api.h"
 
#define FPS_LEN 100
using namespace std;
using namespace cv;

struct RGB {
	unsigned char Red;
	unsigned char Green;
	unsigned char Blue;
};

#ifdef FPS

long delayT = 0;

int countof_loop_tof = 0;
long tatoldelay_tof = 0;
int fps_tof = 0;

int countof_loop_ir = 0;
long tatoldelay_ir = 0;
int fps_ir = 0;

int countof_loop_wdr1 = 0;
long tatoldelay_wdr1 = 0;
int fps_wdr1 = 0;

int countof_loop_wdr2 = 0;
long tatoldelay_wdr2 = 0;
int fps_wdr2 = 0;

int countof_loop_wdr3 = 0;
long tatoldelay_wdr3 = 0;
int fps_wdr3 = 0;

#endif

char cmd[16];
char mode;
bool quit = false;
bool modeChange = false;
bool cap = false;

rtsp_demo_handle g_rtsplive = NULL;
static rtsp_session_handle g_rtsp_session;

MB_IMAGE_INFO_S dstImage;   //dstBuffer의 이미지 정보
void HotPlugStateCallback(const PsDeviceInfo* pInfo, int params);
/*
//배경색 변환 함수
void ChangeSpecificPixels(Mat* pInImage, Mat* pOutImage, const RGB& before, const RGB& after) {
	for(int y = 0; y < pInImage->rows; ++y) {
		for(int x = 0; 0 < pInImage->cols; ++x) {
			if(pInImage->at<Vec3b>(y, x) == Vec3b(before.Blue, before.Green, before.Red)) {
				pOutImage->at<Vec3b>(y, x)[0] = after.Blue;
				pOutImage->at<Vec3b>(y, x)[0] = after.Green;
				pOutImage->at<Vec3b>(y, x)[0] = after.Red;
			}
		}
	}
}
*/

static void *GetCh(void *data) {
    cout << "\n--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "Press following key to set corresponding feature:" << endl;
	cout << "0/1/2...: Change depth range Near/Middle/Far..." << endl;
	cout << "P/p: Save point cloud data into PointCloud.txt in current directory" << endl;
	cout << "T/t: Change background filter threshold value" << endl;
	cout << "M/m: Change data mode: input corresponding index in terminal:" << endl;
	cout << "                    0: Output Depth in 30 fps" << endl;
	cout << "                    1: Output IR in 30 fps" << endl;
	cout << "                    2: Output Depth and IR in 30 fps" << endl;
	cout << "                    3: Output WDR_Depth in 30 fps" << endl;
	cout << "C/c: Enable or disable the ConfidenceFilter in DataMode(DepthAndIR_30) " << endl;
	cout << "F/f: set the ConfidenceFilter Threshold in DataMode(DepthAndIR_30)" << endl;
	cout << "Esc: Program quit " << endl;
	cout << "--------------------------------------------------------------------" << endl;
	cout << "--------------------------------------------------------------------\n" << endl;

    while(!quit) {
        fgets(cmd, sizeof(cmd), stdin);
        printf("#Input cmd: %s\n", cmd);
        if(strstr(cmd, "q")) {
            printf("Get 'quit' cmd!\n");
            quit = true;
        }

        if(strstr(cmd, "0")){
            modeChange = true;
			mode = '0';
        }
		else if(strstr(cmd, "1")){
            modeChange = true;
			mode = '1';
        }
		else if(strstr(cmd, "2")){
            modeChange = true;
			mode = '2';
        }
		else if(strstr(cmd, "3")){
            modeChange = true;
			mode = '3';
        }
		else if(strstr(cmd, "4")){
            //modeChange = true;
			mode = '4';
        }
		else if(strstr(cmd, "5")){
            //modeChange = true;
			mode = '5';
        }
		else if(strstr(cmd, "6")){
            //modeChange = true;
			mode = '6';
        }
		else if(strstr(cmd, "8")){
            //modeChange = true;
			mode = '8';
        }
		else if(strstr(cmd, "p")){
            modeChange = true;
			mode = 'p';
        }
		else if(strstr(cmd, "i")){
            modeChange = true;
			mode = 'i';
        }
		else if(strstr(cmd, "d")){
            modeChange = true;
			mode = 'd';
        }
        else if(strstr(cmd, "c")){
            cap = true;
        }
		else {

		}
        
        usleep(30000);

    }
}

void video_packet_cb(MEDIA_BUFFER mb) { // Venc 데이터 정산, rtsp 전송 콜백
    int ret;
    static DC_S32 packet_cnt = 0;
    if (quit)
        return;

    //printf("#Get packet-%d, size %zu\n", packet_cnt, DC_MPI_MB_GetSize(mb));

    if (g_rtsplive && g_rtsp_session) {
        //printf("1\n");
        ret = rtsp_tx_video(g_rtsp_session, (uint8_t const*)DC_MPI_MB_GetPtr(mb), DC_MPI_MB_GetSize(mb), DC_MPI_MB_GetTimestamp(mb)); // RTSP 전송，수신 MB 는 데이터 가상 주소, 크기, 타임스탬프를 인코딩한 다음 전송합니다.
        //printf("1: %d  ", ret);
        ret = rtsp_do_event(g_rtsplive);
        //printf("2: %d\n", ret);
    }

    DC_MPI_MB_ReleaseBuffer(mb);
    packet_cnt++;
}

static void Opencv_Depth(uint32_t slope, int height, int width, uint8_t*pData, cv::Mat& dispImg)
{
	static int wp = width / 2, hp = height / 2;
	
	if(mode == '8') {
		hp -= 5;
		mode = '0';
		//modeChange = false;
	}
	else if(mode == '5') {
		hp += 5;
		mode = '0';
		//modeChange = false;
	}
	else if(mode == '4') {
		wp -= 5;
		mode = '0';
		//modeChange = false;
	}	
	else if(mode == '6') {
		wp += 5;
		mode = '0';
		//modeChange = false;
	}		

	dispImg = cv::Mat(height, width, CV_16UC1, pData);
	Point2d pointxy(wp, hp);
	int val = dispImg.at<ushort>(pointxy);
	char text[20];

	snprintf(text, sizeof(text), "%d", val);

	dispImg.convertTo(dispImg, CV_8U, 255.0 / slope);
	applyColorMap(dispImg, dispImg, cv::COLORMAP_RAINBOW);
    //printf("rainbow img : %d\n", dispImg.channels());
	int color;
	if (val > 2500)
		color = 0;
	else
		color = 4096;
	//circle(dispImg, pointxy, 4, Scalar(color, color, color), -1, 8, 0);
	//putText(dispImg, text, pointxy, FONT_HERSHEY_DUPLEX, 2, Scalar(color, color, color));
}


int main(int argc, char *argv[])
{
	PsReturnStatus status;
	uint32_t deviceIndex = 0;
	uint32_t deviceCount = 0;
	uint32_t slope = 1450;
	uint32_t wdrSlope = 4400;
	uint32_t wdrRange1Slope = 1450;
	uint32_t wdrRange2Slope = 4400;
	uint32_t wdrRange3Slope = 6000;

	constexpr RGB before{255, 0, 255};
	constexpr RGB after{0, 0, 0};

    Mat *dst = NULL;
    MEDIA_BUFFER dstBuffer;

    int ret;

	PsDepthRange depthRange = PsNearRange;  //깊이 맵 거리 설정 모드.
	PsDataMode dataMode = PsDepth_30;   //데이터 모드. 깊이 맵, IR카메라, 프레임 설정.
	PsDataMode t_datamode = PsDepth_30;
	PsWDROutputMode wdrMode = { PsWDRTotalRange_Two, PsNearRange, 1, PsFarRange, 1, PsUnknown, 1 };   //카메라 매개변수.
	bool f_bWDRMode = false;
	bool bWDRStyle = true;
	bool f_bConfidence = true;

    dstImage.u32Width = 640;   
	dstImage.u32Height = 480;
	dstImage.u32HorStride = 640;
	dstImage.u32VerStride = 480;
	dstImage.enImgType = IMAGE_TYPE_RGB888;   //openCV는 BGR 타잎 사용.

    // init rtsp
    g_rtsplive = create_rtsp_demo(554);
    g_rtsp_session = rtsp_new_session(g_rtsplive, "/live/main_stream");  
    rtsp_set_video(g_rtsp_session, RTSP_CODEC_ID_VIDEO_H264, NULL, 0);  
    rtsp_sync_video_ts(g_rtsp_session, rtsp_get_reltime(), rtsp_get_ntptime()); //rtsp 현재 네트워크 시간 동기화

    DC_MPI_SYS_Init();

    //rga 채널 생성
    RGA_ATTR_S stRgaAttr;
    memset(&stRgaAttr, 0, sizeof(stRgaAttr));
    stRgaAttr.bEnBufPool = RK_TRUE;       //Enable buffer pool.
    stRgaAttr.u16BufPoolCnt = 3;          //Buffer pool count.
    stRgaAttr.u16Rotaion = 0;            //Rotation angle. Values : 0, 90, 180, 270.
    // input image information 
    stRgaAttr.stImgIn.u32X = 0;           //X-axis coordinate of camera.
    stRgaAttr.stImgIn.u32Y = 0;           //Y-axis coordinate of camera.
    stRgaAttr.stImgIn.imgType = IMAGE_TYPE_RGB888;    //Image format type.
    stRgaAttr.stImgIn.u32Width = 640;       //The width of RGA.
    stRgaAttr.stImgIn.u32Height = 480;     //The height of RGA.
    stRgaAttr.stImgIn.u32HorStride = 640;   //Virture width.
    stRgaAttr.stImgIn.u32VirStride = 480;  //Virture height.
    // Output image information 
    stRgaAttr.stImgOut.u32X = 0;          //X-axis coordinate of RGA.
    stRgaAttr.stImgOut.u32Y = 0;          //Y-axis coordinate of RGA.
    stRgaAttr.stImgOut.imgType = IMAGE_TYPE_NV12; //Image format type.
    stRgaAttr.stImgOut.u32Width = 640;       //The width of RGA.
    stRgaAttr.stImgOut.u32Height = 480;
    stRgaAttr.stImgOut.u32HorStride = 640;
    stRgaAttr.stImgOut.u32VirStride = 480;
    ret = DC_MPI_RGA_CreateChn(0, &stRgaAttr);
    if (ret) {
        printf("Create RGA[0] failed! ret=%d\n", ret);
    }

    //Venc 모듈 데이터 설정
    VENC_CHN_ATTR_S venc_chn_attr; 
    memset(&venc_chn_attr, 0, sizeof(venc_chn_attr));
    venc_chn_attr.stVencAttr.enType = DC_CODEC_TYPE_H264; 
    venc_chn_attr.stRcAttr.enRcMode = VENC_RC_MODE_H264CBR;
    venc_chn_attr.stRcAttr.stH265Cbr.u32Gop = 30;  //30
    venc_chn_attr.stRcAttr.stH265Cbr.u32BitRate = 1920 * 1200; 
    // frame rate: in 30/1, out 30/1.
    venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH265Cbr.fr32DstFrameRateNum = 30;
    venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateDen = 1;
    venc_chn_attr.stRcAttr.stH265Cbr.u32SrcFrameRateNum = 30;
 
    venc_chn_attr.stVencAttr.imageType = IMAGE_TYPE_NV12; 
    venc_chn_attr.stVencAttr.u32PicWidth = 640; 
    venc_chn_attr.stVencAttr.u32PicHeight = 480;
    venc_chn_attr.stVencAttr.u32VirWidth = 640;
    venc_chn_attr.stVencAttr.u32VirHeight = 480;
    venc_chn_attr.stVencAttr.u32Profile = 77;
    ret = DC_MPI_VENC_CreateChn(0, &venc_chn_attr);
    if (ret) {
        printf("ERROR: create VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    MPP_CHN_S stSrcChn;
    stSrcChn.enModId = DC_ID_RGA;
    stSrcChn.s32DevId = 0;
    stSrcChn.s32ChnId = 0;

    MPP_CHN_S stEncChn; 
    stEncChn.enModId = DC_ID_VENC;
    stEncChn.s32DevId = 0;
    stEncChn.s32ChnId = 0;
    ret = DC_MPI_SYS_RegisterOutCb(&stEncChn, video_packet_cb); 
    if (ret) {
        printf("ERROR: register output callback for VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_SYS_Bind(&stSrcChn, &stEncChn);
    if (ret) {
        printf("ERROR: Bind VI[0] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }

	status = Ps2_Initialize();  //초기화.
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsInitialize failed!" << endl;
		system("pause");
		return -1;
	}

GET:
	status = Ps2_GetDeviceCount(&deviceCount);   //연결된 장치 수 가져오기.
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "PsGetDeviceCount failed! make sure the DCAM is connected" << endl;
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}
	cout << "Get device count: " << deviceCount << endl;
	if (0 == deviceCount)
	{
		this_thread::sleep_for(chrono::seconds(1));
		goto GET;
	}
    Ps2_SetHotPlugStatusCallback(HotPlugStateCallback);

    //여기까지는 디바이스 연결관련

	PsDeviceInfo* pDeviceListInfo = new PsDeviceInfo[deviceCount];
	status = Ps2_GetDeviceListInfo(pDeviceListInfo, deviceCount);
	PsDeviceHandle deviceHandle = 0;
	status = Ps2_OpenDevice(pDeviceListInfo->uri, &deviceHandle);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "OpenDevice failed!" << endl;
		system("pause");
		return -1;
	}
	uint32_t sessionIndex = 0;

	status = Ps2_StartStream(deviceHandle, sessionIndex);
	if (status != PsReturnStatus::PsRetOK)
	{
		cout << "StartStream failed!" << endl;
		system("pause");
		return -1;
	}

	PsCameraParameters cameraParameters;
	status = Ps2_GetCameraParameters(deviceHandle, sessionIndex, PsDepthSensor, &cameraParameters);

	cout << "Get PsGetCameraParameters status: " << status << endl;
	cout << "Depth Camera Intinsic: " << endl;
	cout << "Fx: " << cameraParameters.fx << endl;
	cout << "Cx: " << cameraParameters.cx << endl;
	cout << "Fy: " << cameraParameters.fy << endl;
	cout << "Cy: " << cameraParameters.cy << endl;
	cout << "Depth Distortion Coefficient: " << endl;
	cout << "K1: " << cameraParameters.k1 << endl;
	cout << "K2: " << cameraParameters.k2 << endl;
	cout << "P1: " << cameraParameters.p1 << endl;
	cout << "P2: " << cameraParameters.p2 << endl;
	cout << "K3: " << cameraParameters.k3 << endl;
	cout << "K4: " << cameraParameters.k4 << endl;
	cout << "K5: " << cameraParameters.k5 << endl;
	cout << "K6: " << cameraParameters.k6 << endl;

	//Get MeasuringRange
	PsMeasuringRange measuringrange = { 0 };

	status = Ps2_GetDataMode(deviceHandle, sessionIndex, &dataMode);
	if (status != PsReturnStatus::PsRetOK)
		cout << "Ps2_GetDataMode failed!" << endl;
	else
		cout << "Get Ps2_GetDataMode : " << dataMode << endl;
	

	status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depthRange);		
    if (status != PsReturnStatus::PsRetOK)
		cout << "Ps2_GetDepthRange failed!" << endl;
	else
		cout << "Get Depth Range " << depthRange << endl;

	status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depthRange, &measuringrange);
	if (status != PsReturnStatus::PsRetOK)
		cout << "Ps2_GetMeasuringRange failed!" << endl;
	else
	{
		slope = measuringrange.effectDepthMaxNear;
		cout << "slope  ==  " << slope << endl;
	}
	
	cv::Mat imageMat;

	ofstream PointCloudWriter;
	PsDepthVector3 DepthVector = { 0, 0, 0 };
	PsVector3f WorldVector = { 0.0f }; 

	bool f_bPointClound = false;

	PsDepthRangeList rangelist = { 0 };
	int len = sizeof(rangelist);
	status = Ps2_GetProperty(deviceHandle, sessionIndex, PsPropertyDepthRangeList, &rangelist, &len);

	if (status == PsReturnStatus::PsRetOK&&rangelist.count > 0)
	{
		cout << "Available Range List: ";
		for (int i = 0; i < rangelist.count-1; i++)
		{
			cout << (int)rangelist.depthrangelist[i] <<",";
		}
		cout << (int)rangelist.depthrangelist[rangelist.count - 1] << endl;
	}

    pthread_t main_stream_thread;
    pthread_create(&main_stream_thread, NULL, GetCh, NULL);


	while (!quit)
	{      
        //PsFrame = 이미지 정보
		PsFrame depthFrame = { 0 };
		PsFrame irFrame = { 0 };
		PsFrame wdrDepthFrame = { 0 };

		// Read one frame before call PsGetFrame
		PsFrameReady frameReady = { 0 };
		status = Ps2_ReadNextFrame(deviceHandle, sessionIndex, &frameReady);
		
#ifdef FPS
		struct  timeb   stTimeb;
		ftime(&stTimeb);
		long dwEnd = stTimeb.millitm;
		long timedelay = dwEnd - delayT;
		delayT = dwEnd;
		if (timedelay < 0) {
			timedelay += 1000;
		}
		tatoldelay_tof += timedelay;
		tatoldelay_ir += timedelay;
		tatoldelay_wdr1 += timedelay;
		tatoldelay_wdr2 += timedelay;
		tatoldelay_wdr3 += timedelay;

#endif
		//Get depth frame, depth frame only output in following data mode
		if (1 == frameReady.depth)
		{
			status = Ps2_GetFrame(deviceHandle, sessionIndex, PsDepthFrame, &depthFrame);

			if (depthFrame.pFrameData != NULL)
			{   
				if (f_bPointClound)
				{
					PointCloudWriter.open("PointCloud.txt");
					PsFrame &srcFrame = depthFrame;
					const int len = srcFrame.width * srcFrame.height;
					PsVector3f* worldV = new PsVector3f[len];

					Ps2_ConvertDepthFrameToWorldVector(deviceHandle, sessionIndex, srcFrame, worldV); //Convert Depth frame to World vectors.

					for (int i = 0; i < len; i++)
					{
						if (worldV[i].z == 0 || worldV[i].z == 0xFFFF)
							continue; //discard zero points
						PointCloudWriter << worldV[i].x << "\t" << worldV[i].y << "\t" << worldV[i].z << std::endl;
					}
					delete[] worldV;
					worldV = NULL;
					std::cout << "Save point cloud successful in PointCloud.txt" << std::endl;
					PointCloudWriter.close();
					f_bPointClound = false;
				}
				//Display the Depth Image
				if (dataMode == PsDepth_30)  //f_bWDRMode&&dataMode == PsWDR_Depth
				{
#ifdef FPS
					countof_loop_tof++;
					if (countof_loop_tof >= FPS_LEN)
					{
						fps_tof = 1000 * FPS_LEN / tatoldelay_tof;
						//cout << fps_tof<<endl;
						countof_loop_tof = 0;
						tatoldelay_tof = 0;
					}
#endif                  
					Opencv_Depth(slope, depthFrame.height, depthFrame.width, depthFrame.pFrameData, imageMat);

#ifdef FPS
					if (fps_tof != 0)
					{
						char fps[20];
						snprintf(fps, sizeof(fps), "FPS: %d", fps_tof);
						putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(0, 0, 0));
					}
#endif
                    
                    if(cap == true) {
                        imwrite("/tmp/tof.jpg", imageMat);
                        printf("captOK!\n");
                        cap = false;
                    }
                    //cvtColor(imageMat, imageMat, COLOR_RGB2YUV_I420);
					//cvtColor(imageMat, imageMat, CV_HSV2RGB);
	                //ChangeSpecificPixels(&imageMat, &imageMat, before, after);
                    dstBuffer = DC_MPI_MB_CreateImageBuffer(&dstImage, DC_TRUE, MB_FLAG_NOCACHED);
                    dst = new Mat(Size(640, 480), CV_8UC3, (char *)DC_MPI_MB_GetPtr(dstBuffer));
                    //dstBuffer = (char *)DC_MPI_MB_GetPtr(&imageMat);
                    imageMat.copyTo(*dst);

					

                    DC_MPI_SYS_SendMediaBuffer(DC_ID_RGA, 0, dstBuffer);
                    delete dst;

                    DC_MPI_MB_ReleaseBuffer(dstBuffer);			
				}
					
				
	
			}
			else
			{
				cout << "Ps2_GetFrame PsDepthFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}

        if (1 == frameReady.ir)
		{
			//printf("ir?\n");
			status = Ps2_GetFrame(deviceHandle, sessionIndex, PsIRFrame, &irFrame);

            if (irFrame.pFrameData != NULL)
			{
#ifdef FPS	 
				countof_loop_ir++;
				if (countof_loop_ir >= FPS_LEN)
				{
					fps_ir = 1000 * FPS_LEN / tatoldelay_ir;
					//cout << fps_tof << endl;
					countof_loop_ir = 0;
					tatoldelay_ir = 0;
				}		 
#endif
				//Display the IR Image
				imageMat = cv::Mat(irFrame.height, irFrame.width, CV_16UC1, irFrame.pFrameData);

				// Convert 16bit IR pixel (max pixel value is 3840) to 8bit for display
				imageMat.convertTo(imageMat, CV_8U, 255.0 / 3840);
				cvtColor(imageMat, imageMat, COLOR_GRAY2RGB);  //8bit -> 8bit 3ch 로 변환.
                //cvtColor(imageMat, imageMat, COLOR_RGB2YUV_I420);
				//imageMat.convertTo(imageMat, CV_8UC3);
#ifdef FPS
				if (fps_ir != 0)
				{
					char fps[20];
					snprintf(fps, sizeof(fps), "FPS: %d", fps_ir);
					putText(imageMat, fps, Point2d(10, 20), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 255, 255));
				}
#endif
                if(cap == true) {
                    imwrite("/tmp/tof.jpg", imageMat);
                    printf("captOK!\n");
                    cap = false;
                }
                dstBuffer = DC_MPI_MB_CreateImageBuffer(&dstImage, DC_TRUE, MB_FLAG_NOCACHED);
                dst = new Mat(Size(640, 480), CV_8UC3, (char *)DC_MPI_MB_GetPtr(dstBuffer));
                //dstBuffer = (char *)DC_MPI_MB_GetPtr(&imageMat);
                imageMat.copyTo(*dst);

				

                DC_MPI_SYS_SendMediaBuffer(DC_ID_RGA, 0, dstBuffer);
                delete dst;

                DC_MPI_MB_ReleaseBuffer(dstBuffer);
			}
			else
			{
				cout << "Ps2_GetFrame PsIRFrame status:" << status << " pFrameData is NULL " << endl;
			}
		}
        
        if(modeChange) {
			//printf("mode debug!!!\n");
			
            modeChange = false;
            switch(mode) {
                case '0':
					printf("0\n");
                    depthRange = PsNearRange;
				    slope = 1450;
                    break;
                case '1':
                    printf("1\n");
					depthRange = PsMidRange;
				    slope = 3000;
                    break;
                case '2':
				    printf("2\n");
					depthRange = PsFarRange;
				    slope = 4400;
				    break;
				case '3':
				    printf("3\n");
					depthRange = PsXFarRange;
				    slope = 6000;
				    break;
				case 'p':
				    printf("p\n");
					f_bPointClound = true;
				    break;
				case 'i':
				    printf("i\n");
					t_datamode = PsIR_30;					
				    break;
				case 'd':
				    printf("d\n");
					t_datamode = PsDepth_30;
				    break;
                default:
				    cout << "Unsupported Range!" << endl;
				    break;
            }

			if((mode == '0') || (mode == '1') || (mode == '2') || (mode == '3')) {
            	status = Ps2_SetDepthRange(deviceHandle, sessionIndex, depthRange);
				if (depthRange == PsNearRange)
					cout << "Set depth range to Near," << " status: " << status << endl;
				if (depthRange == PsMidRange)
					cout << "Set depth range to Mid," << " status: " << status << endl;
				if (depthRange == PsFarRange)
					cout << "Set depth range to Far," << " status: " << status << endl;
				if (depthRange == PsXFarRange)
					cout << "Set depth range to XFar," << " status: " << status << endl;
            	if (status != PsRetOK)
				{
					cout << "Set depth range failed! " << endl;
				}

	            status = Ps2_GetDepthRange(deviceHandle, sessionIndex, &depthRange);
				cout << "Get depth range," << " depthRange: " << depthRange << endl;
				if (status != PsRetOK)
				{
					cout << "Get depth range failed! " << endl;
				}
				else
				{
					status = Ps2_GetMeasuringRange(deviceHandle, sessionIndex, depthRange, &measuringrange);
					if (status != PsReturnStatus::PsRetOK)
						cout << "Ps2_GetMeasuringRange failed!" << endl;
					else
					{
						switch (depthRange)
						{
						case PsNearRange:
							slope = measuringrange.effectDepthMaxNear;
							break;

						case PsMidRange:
							slope = measuringrange.effectDepthMaxMid;
							break;

						case PsFarRange:
							slope = measuringrange.effectDepthMaxFar;
							break;
						
						case PsXFarRange:
							slope = measuringrange.effectDepthMaxFar;
							break;
						default:
							break;
						}
						cout << "slope  ==  " << slope << endl;
					}
				}
			}
			if((mode == 'i') || (mode == 'd')) {

				status = Ps2_SetDataMode(deviceHandle, sessionIndex, (PsDataMode)t_datamode);
				if (status != PsRetOK)
				{
					cout << "Ps2_SetDataMode  status" << status << endl;
				}		
			}
        }
	}
    pthread_join(main_stream_thread, NULL);
   
    // destroy venc before vi
    rtsp_del_demo(g_rtsplive);

    ret = DC_MPI_SYS_UnBind(&stSrcChn, &stEncChn);
    if (ret) {
        printf("ERROR: UnBind VI[0] and VENC[0] error! ret=%d\n", ret);
        return 0;
    }
    
    ret = DC_MPI_VENC_DestroyChn(0);
    if (ret) {
        printf("ERROR: Destroy VENC[0] error! ret=%d\n", ret);
        return 0;
    }

    ret = DC_MPI_RGA_DestroyChn(0);
    if (ret) {
        printf("ERROR: Destroy VENC[0] error! ret=%d\n", ret);
        return 0;
    }
	
    status = Ps2_CloseDevice(&deviceHandle);
	cout << "CloseDevice status: " << status << endl;

	status = Ps2_Shutdown();
	cout << "Shutdown status: " << status << endl;
	return 0;
}

void HotPlugStateCallback(const PsDeviceInfo* pInfo, int status)
{
	cout << "uri " << status << "  " << pInfo->uri << "    " << (status == 0 ? "add" : "remove") << endl;
	cout << "alia " << status << "  " << pInfo->alias << "    " << (status == 0 ? "add" : "remove") << endl;
 }
