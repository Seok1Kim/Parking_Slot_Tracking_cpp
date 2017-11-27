// ParkingSlotTracking.h
#include "opencv2\opencv.hpp"

#pragma once

typedef enum { ERR_NONE = 0, ERR_INIT, ERR_INPUT, ERR_PREDICT, ERR_MEAS_UPT } eOGF_Error_type;

class CParkingSlotTracking
{
	// ===========================================================================
	// Process functions
	// ---------------------------------------------------------------------------
public:
	// Constructor & Destructor
	explicit CParkingSlotTracking();
	virtual ~CParkingSlotTracking();

public:
	// Task
	bool Init(); // initialization
	void Run(bool trackingFlag, cv::Mat img_src, std::vector<cv::Point>pt_list, double yaw_rate, double speed, double dt);
	void Terminate(); // termination
	// ===========================================================================


private:
	// ===========================================================================
	// Configuration Parameters
	// ---------------------------------------------------------------------------
	// Constant
	eOGF_Error_type				m_eErrorCode;
	bool						templateflag;
	double						XPixelMeter;
	// ===========================================================================

private:
	// ===========================================================================
	// Internal functions
	// ---------------------------------------------------------------------------
	bool predictPosition(std::vector<cv::Point>pt_list, std::vector<cv::Point>& pt_list_dst);
	bool estimateSpace(std::vector<cv::Point> pt_list, std::vector<cv::Point> pt_list_dst);
	bool CParkingSlotTracking::cropImage(cv::Mat img_src, cv::Mat& img_dst, std::vector<cv::Point> pt_list);
	bool CParkingSlotTracking::calculateDT(cv::Mat img_cropped, std::vector<cv::Mat>& img_dst);
	// ===========================================================================

public:
	// ===========================================================================
	// External interfaces
	// ---------------------------------------------------------------------------

	// ===========================================================================
};