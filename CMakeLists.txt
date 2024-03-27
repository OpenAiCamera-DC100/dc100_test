CMAKE_MINIMUM_REQUIRED(VERSION 2.8.0 FATAL_ERROR)
SET(CMAKE_CXX_STANDARD 11)
PROJECT(cmosmedia_media_test)

SET(CMAKE_C_COMPILER "${PROJECT_SOURCE_DIR}/../../../../../prebuilts/gcc/linux-x86/arm/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-gcc")
SET(CMAKE_CXX_COMPILER "${PROJECT_SOURCE_DIR}/../../../../../prebuilts/gcc/linux-x86/arm/gcc-linaro-6.3.1-2017.05-x86_64_arm-linux-gnueabihf/bin/arm-linux-gnueabihf-g++")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

#include(FindPkgConfig)
#pkg_check_modules (JSON-C REQUIRED IMPORTED_TARGET json-c)

include_directories(
  ${CMAKE_SOURCE_DIR}/common
  ${CMAKE_SOURCE_DIR}/librtsp
  ${CMAKE_SOURCE_DIR}/yolov5
  ${CMAKE_SOURCE_DIR}/tracker/bytetrack/include
)

set(YOLOV5_SOURCE
  yolov5/YOLOv562Detector.cpp
)

set(TRACKER_SOURCE
  tracker/bytetrack/src/BytekalmanFilter.cpp
  tracker/bytetrack/src/BYTETracker.cpp
  tracker/bytetrack/src/lapjv.cpp
  tracker/bytetrack/src/STrack.cpp
  tracker/bytetrack/src/utils.cpp
)

set(CC_DCAIQ_SOURCE
  common/sample_common_isp.cpp
  #common/sample_fake_isp.cpp
)

set(C_DCAIQ_SOURCE
  common/sample_common_isp.c
  #common/sample_fake_isp.c
)

set(OPENCV_LIBS
    opencv_flann
    opencv_videoio
    opencv_core
    opencv_highgui
    opencv_imgcodecs
    opencv_imgproc
    opencv_calib3d
)

set(ROCKCHIP_RTSP_LIB
    ${CMAKE_SOURCE_DIR}/librtsp/librtsp.a
)

install(FILES ${ROCKCHIP_RTSP_LIB} DESTINATION lib)

#--------------------------
# cmos_vi_venc_rtsp_test.cc
#--------------------------
#add_executable(cmos_vi_venc_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_vi_venc_rtsp_test.cc ${CC_DCAIQ_SOURCE})
#target_link_libraries(cmos_vi_venc_rtsp_test dcmedia dcaiq pthread ${ROCKCHIP_RTSP_LIB})
#install(TARGETS cmos_vi_venc_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_get_lux.c
#--------------------------
add_executable(cmos_get_lux ${CMAKE_SOURCE_DIR}/cmos_get_lux.c)
install(TARGETS cmos_get_lux RUNTIME DESTINATION "bin")

#--------------------------
# cmos_netserver_test.c
#--------------------------
add_executable(cmos_netserver_test ${CMAKE_SOURCE_DIR}/cmos_netserver_test.c)
target_link_libraries(cmos_netserver_test pthread json-c)
install(TARGETS cmos_netserver_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_rightcam_rtsp_test.c
#--------------------------
add_executable(cmos_rightcam_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_rightcam_rtsp_test.c ${C_DCAIQ_SOURCE})
target_link_libraries(cmos_rightcam_rtsp_test dcmedia dcaiq_c pthread ${ROCKCHIP_RTSP_LIB})
install(TARGETS cmos_rightcam_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_leftcam_rtsp_test.c
#--------------------------
add_executable(cmos_leftcam_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_leftcam_rtsp_test.c ${C_DCAIQ_SOURCE})
target_link_libraries(cmos_leftcam_rtsp_test dcmedia dcaiq_c pthread ${ROCKCHIP_RTSP_LIB})
install(TARGETS cmos_leftcam_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_person_detection_test.c
#--------------------------
#add_executable(cmos_person_detection_test ${CMAKE_SOURCE_DIR}/cmos_person_detection_test.c ${C_DCAIQ_SOURCE})
#target_link_libraries(cmos_person_detection_test ${ROCKCHIP_RTSP_LIB} m dcmedia dcaiq_c pthread rga rknn_api)
#target_include_directories(cmos_person_detection_test PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SYSROOT}/usr/include/rknn)
#install(TARGETS cmos_person_detection_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_yolo_test.cc
#--------------------------
add_executable(cmos_yolo_test ${CMAKE_SOURCE_DIR}/cmos_yolo_test.cc ${CC_DCAIQ_SOURCE} ${YOLOV5_SOURCE})
target_link_libraries(cmos_yolo_test ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS} m dcmedia dcaiq pthread rga rknn_api Eigen3::Eigen)
target_include_directories(cmos_yolo_test PRIVATE ${CMAKE_SOURCE_DIR}/include ${CMAKE_SYSROOT}/usr/include/rknn)
install(TARGETS cmos_yolo_test RUNTIME DESTINATION "bin")
install(FILES ${CMAKE_SOURCE_DIR}/yolov5/yolov5s_v6.2_output3_4.rknn DESTINATION /usr/share)
install(FILES ${CMAKE_SOURCE_DIR}/yolov5/coco_80_labels_list.txt DESTINATION /usr/share)
#install(FILES ${CMAKE_SOURCE_DIR}/yolov5/yolov5s_relu_rv1109_rv1126_out_opt.rknn DESTINATION /oem/usr/share)
#install(FILES ${CMAKE_SOURCE_DIR}/yolov5/coco_80_labels_list.txt DESTINATION /oem/usr/share)

#--------------------------
# cmos_thermalcam_rtsp_test.cpp
#--------------------------
add_executable(cmos_thermalcam_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_thermalcam_rtsp_test.cpp)
target_link_libraries(cmos_thermalcam_rtsp_test dcmedia pthread USBSDK ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS})
install(TARGETS cmos_thermalcam_rtsp_test RUNTIME DESTINATION "bin")


#--------------------------
# cmos_tof_rtsp_test.cpp
#--------------------------
add_executable(cmos_tof_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_tof_rtsp_test.cpp)
target_link_libraries(cmos_tof_rtsp_test dcmedia pthread vzense_api ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS})
install(TARGETS cmos_tof_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_vi_rockx_face_detect_rtsp_test.cc
#--------------------------
add_executable(cmos_vi_rockx_face_detect_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_vi_rockx_face_detect_rtsp_test.cc  ${CC_DCAIQ_SOURCE})
target_link_libraries(cmos_vi_rockx_face_detect_rtsp_test dcmedia dcaiq pthread ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS} rockx)
install(TARGETS cmos_vi_rockx_face_detect_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_peoplecount_rtsp_test.cc
#--------------------------
add_executable(cmos_peoplecount_rtsp_test ${CMAKE_SOURCE_DIR}/cmos_peoplecount_rtsp_test.cc  ${CC_DCAIQ_SOURCE})
target_link_libraries(cmos_peoplecount_rtsp_test dcmedia dcaiq pthread ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS} rockx)
install(TARGETS cmos_peoplecount_rtsp_test RUNTIME DESTINATION "bin")

#--------------------------
# cmos_stereo_disparity_test.cc
#--------------------------
#add_executable(cmos_stereo_disparity_test ${CMAKE_SOURCE_DIR}/cmos_stereo_disparity_test.cc  ${CC_DCAIQ_SOURCE})
#target_link_libraries(cmos_stereo_disparity_test ${COMMON_DEPENDENT_LIBS} ${ROCKCHIP_RTSP_LIB} ${OPENCV_LIBS} rga rkaiq)
#install(TARGETS cmos_stereo_disparity_test RUNTIME DESTINATION "bin")
