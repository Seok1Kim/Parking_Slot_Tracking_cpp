// ParkingSlotTracking.h
#include "opencv2\opencv.hpp"

#pragma once

typedef enum { ERR_NONE = 0, ERR_INIT, ERR_ESTIMATE, ERR_CROP,  ERR_CALDT, ERR_PREDICT} eOGF_Error_type;

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
	void Init(); // initialization
	void Run(bool trackingFlag, cv::Mat img_src, std::vector<cv::Point2d>pt_list, double yaw_rate, double speed, double dt, std::vector<cv::Point2d>& pt_list_filtered);
	bool CParkingSlotTracking::predictPosition(std::vector<cv::Point2d>pt_list, double yaw_rate, double speed, double dt, std::vector<cv::Point2d>& pt_list_dst);
	void Terminate(); // termination
	// ===========================================================================


private:
	// ===========================================================================
	// Configuration Parameters
	// ---------------------------------------------------------------------------
	// Constant
	eOGF_Error_type					m_eErrorCode;
	bool							templateflag;
	double							m_XPixelMeter;
	std::vector<cv::Point2d>			m_pt_list_predicted;
	std::vector<cv::Point2d>			m_pt_list_estimated;
	std::vector<cv::Point2d>			m_pt_list_corrected;
	std::vector<cv::Point2d>			m_pt_list_tracking;
	cv::Mat							img_cropped_template;
	std::vector<cv::Mat>			img_DT_template;
	double							m_VehicleCenterX;
	double							m_VehicleCenterY;
	int								m_CropImageSizeX;
	int								m_CropImageSizeY;
	double							m_ResizeScale;
	int								m_ThresholdImage;
	int								m_disturbanceX;
	int								m_disturbanceY;
	int								m_disturbanceANG;

	// ===========================================================================

private:
	// ===========================================================================
	// Internal functions
	// ---------------------------------------------------------------------------
	
	bool CParkingSlotTracking::estimateSpace(std::vector<cv::Point2d> pt_list, std::vector<cv::Point2d>& pt_list_estimated);
	bool CParkingSlotTracking::cropImage(cv::Mat img_src_ss, cv::Mat& img_cropped, std::vector<cv::Point2d> pt_list_estimated);
	bool CParkingSlotTracking::calculateDT(cv::Mat img_cropped, std::vector<cv::Mat>& img_DT);
	bool CParkingSlotTracking::getDisturbancePoints(std::vector<cv::Point2d> pt_list_predicted, std::vector<cv::Point2d>& pt_list_disturbance, int x_disturbance, int y_disturbance, int ang_disturbance);
	bool CParkingSlotTracking::correctPosition(std::vector<cv::Point2d> pt_list_src, std::vector<cv::Point2d> pt_list_predicted, cv::Mat img_src_ss, std::vector<cv::Mat> img_template_DT, std::vector<cv::Point2d>& pt_list_corrected);
	bool CParkingSlotTracking::KalmanFiltering(std::vector<cv::Point2d> pt_list_corrected, std::vector<cv::Point2d>& pt_list_filtered);

	// ===========================================================================

public:
	// ===========================================================================
	// External interfaces
	// ---------------------------------------------------------------------------
	std::vector<cv::Point2d> getCornerpoint(void);
	void asdt();
	// ===========================================================================
};