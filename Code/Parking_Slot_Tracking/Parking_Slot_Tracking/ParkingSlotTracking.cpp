// ParkingSlotTracking.cpp
//
// 2017.11.27: Seokwon Kim, release version 1.0 for parking slot tracking

// Include
#include "ParkingSlotTracking.h"
#include "opencv\highgui.h"

#define _USE_MATH_DEFINES
#include <math.h>

CParkingSlotTracking::CParkingSlotTracking(void)
	:m_eErrorCode(ERR_NONE),
	templateflag(false),
	XPixelMeter(0,)

{
	Init();
}

CParkingSlotTracking::~CParkingSlotTracking()
{
}

bool CParkingSlotTracking::Init()
{
	XPixelMeter = 0.02469961956;
}

void CParkingSlotTracking::Run(bool trackingFlag, cv::Mat img_src, std::vector<cv::Point>pt_list, double yaw_rate, double speed, double dt)
{
	if (templateflag == false){

		templateflag = true;
	}
	else{

	}
}

void CParkingSlotTracking::Terminate()
{
}

// ===========================================================================
// Internal functions
// ---------------------------------------------------------------------------
// Initialization
bool predictPosition(std::vector<cv::Point>pt_list, std::vector<cv::Point>& pt_list_dst){

}
bool estimateSpace(std::vector<cv::Point> pt_list, std::vector<cv::Point> pt_list_dst){

}
bool CParkingSlotTracking::cropImage(cv::Mat img_src, cv::Mat& img_dst, std::vector<cv::Point> pt_list){
	
}

bool CParkingSlotTracking::calculateDT(cv::Mat img_cropped, std::vector<cv::Mat>& img_dst){

}

// ===========================================================================


// ===========================================================================
// External interfaces
// ---------------------------------------------------------------------------

// ===========================================================================