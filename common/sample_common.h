#include <assert.h>
#include <fcntl.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

#include "dcmedia_api.h"

#include "dcaiq_api.h"

/*
 * stream on:
 * 1) ISP Init
 * 2) ISP Run
 * 3) VI enable and stream on
 *
 * stream off:
 * 4) VI disable
 * 5) ISP Stop
 */
/*
typedef enum {
 DC_AIQ_WORKING_MODE_NORMAL,
 DC_AIQ_WORKING_MODE_ISP_HDR2    = 0x10,
 DC_AIQ_WORKING_MODE_ISP_HDR3    = 0x20,
 //DC_AIQ_WORKING_MODE_SENSOR_HDR = 10, // sensor built-in hdr mode
} DC_aiq_working_mode_t;
*/
DC_S32 SAMPLE_COMM_ISP_Init(DC_S32 CamId, dc_aiq_working_mode_t WDRMode,
                            DC_BOOL MultiCam, const char *iq_file_dir);
DC_S32 SAMPLE_COMM_ISP_UpdateIq(DC_S32 CamId, char *iqfile);
DC_S32 SAMPLE_COMM_ISP_Stop(DC_S32 CamId);
DC_S32 SAMPLE_COMM_ISP_SetFecEn(DC_S32 CamId, DC_BOOL bFECEnable);
DC_S32 SAMPLE_COMM_ISP_Run(DC_S32 CamId); // isp stop before vi streamoff
DC_S32 SAMPLE_COMM_ISP_DumpExpInfo(DC_S32 CamId, dc_aiq_working_mode_t WDRMode);
DC_S32 SAMPLE_COMM_ISP_SetFrameRate(DC_S32 CamId, DC_U32 uFps);
DC_S32 SAMPLE_COMM_ISP_SetLDCHLevel(DC_S32 CamId, DC_U32 level);
DC_S32 SAMPLE_COMM_ISP_SetWDRModeDyn(DC_S32 CamId,
                                     dc_aiq_working_mode_t WDRMode);
DC_S32 SAMPLE_COMM_ISP_SET_Brightness(DC_S32 CamId, DC_U32 value);
DC_S32 SAMPLE_COMM_ISP_SET_Contrast(DC_S32 CamId, DC_U32 value);
DC_S32 SAMPLE_COMM_ISP_SET_Saturation(DC_S32 CamId, DC_U32 value);
DC_S32 SAMPLE_COMM_ISP_SET_Sharpness(DC_S32 CamId, DC_U32 value);
DC_S32 SAMPLE_COMM_ISP_SET_ManualExposureAutoGain(DC_S32 CamId,
                                                  DC_U32 u32Shutter);
DC_S32 SAMPLE_COMM_ISP_SET_ManualExposureManualGain(DC_S32 CamId,
                                                    DC_U32 u32Shutter,
                                                    DC_U32 u32Gain);
DC_S32 SAMPLE_COMM_ISP_SET_AutoExposure(DC_S32 CamId);
DC_S32 SAMPLE_COMM_ISP_SET_Exposure(DC_S32 CamId, DC_BOOL bIsAutoExposure,
                                    DC_U32 bIsAGC, DC_U32 u32ElectronicShutter,
                                    DC_U32 u32Agc);
DC_S32 SAMPLE_COMM_ISP_SET_BackLight(DC_S32 CamId, DC_BOOL bEnable,
                                     DC_U32 u32Strength);
DC_S32 SAMPLE_COMM_ISP_SET_LightInhibition(DC_S32 CamId, DC_BOOL bEnable,
                                           DC_U8 u8Strength, DC_U8 u8Level);
DC_S32 SAMPLE_COMM_ISP_SET_CPSL_CFG(DC_S32 CamId, dc_aiq_cpsl_cfg_t *cpsl);
DC_S32 SAMPLE_COMM_ISP_SET_OpenColorCloseLed(DC_S32 CamId);
DC_S32 SAMPLE_COMM_ISP_SET_GrayOpenLed(DC_S32 CamId, DC_U8 u8Strength);
DC_S32 SAMPLE_COMMON_ISP_SET_AutoWhiteBalance(DC_S32 CamId);
DC_S32 SAMPLE_COMMON_ISP_SET_ManualWhiteBalance(DC_S32 CamId, DC_U32 u32RGain,
                                                DC_U32 u32GGain,
                                                DC_U32 u32BGain);
DC_S32 SAMPLE_COMMON_ISP_GET_WhiteBalanceGain(DC_S32 CamId,
                                              dc_aiq_wb_gain_t *gain);
DC_S32 SAMPLE_COMMON_ISP_SET_DNRStrength(DC_S32 CamId, DC_U32 u32Mode,
                                         DC_U32 u322DValue, DC_U32 u323Dvalue);
DC_S32 SAMPLE_COMMON_ISP_GET_DNRStrength(DC_S32 CamId, DC_U32 *u322DValue,
                                         DC_U32 *u323Dvalue);

DC_S32 SAMPLE_COMMON_ISP_SET_Flicker(DC_S32 CamId, DC_U8 u32Flicker);
DC_S32 SAMPLE_COMM_ISP_SET_HDR(DC_S32 CamId, dc_aiq_working_mode_t mode);
DC_S32 SAMPLE_COMM_ISP_SET_DefogEnable(DC_S32 CamId, DC_U32 u32Mode);
DC_S32 SAMPLE_COMM_ISP_SET_DefogStrength(DC_S32 CamId, DC_U32 u32Mode,
                                         DC_U32 u32Value);
DC_S32 SAMPLE_COMM_ISP_SET_Correction(DC_S32 CamId, DC_U32 u32Mode,
                                      DC_U32 u32Value);
DC_S32 SAMPLE_COMM_ISP_SET_mirror(DC_S32 CamId, DC_U32 u32Value);
DC_S32 SAMPLE_COMM_ISP_SET_BypassStreamRotation(DC_S32 CamId,
                                                DC_S32 S32Rotation);
DC_S32 SAMPLE_COMM_ISP_SET_Crop(DC_S32 CamId, dc_aiq_rect_t rect);
