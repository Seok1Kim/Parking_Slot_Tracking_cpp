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
	templateflag(true),
	m_XPixelMeter(0.),
	m_CropImageSizeX(0),
	m_CropImageSizeY(0),
	m_ResizeScale(0.3),
	m_ThresholdImage(245),
	m_disturbanceX(2),
	m_disturbanceY(2),
	m_disturbanceANG(1)

{
	Init();
}

CParkingSlotTracking::~CParkingSlotTracking()
{
}

void CParkingSlotTracking::Init()
{
	m_XPixelMeter = 0.02469961956;

	m_VehicleCenterX = 245.;
	m_VehicleCenterY = 118.;

	m_F_matrix = 1. * cv::Mat::eye(4, 4, CV_64F);
	m_H_matrix = 1. * cv::Mat::eye(4, 4, CV_64F);
	m_Q_matrix = 1e-6 * cv::Mat::eye(4, 4, CV_64F);
	m_R_matrix = 1e-4 * cv::Mat::eye(4, 4, CV_64F);
	m_P_matrix = 1. * cv::Mat::eye(4, 4, CV_64F);
}

void CParkingSlotTracking::Run(bool trackingFlag, bool filterFlag, cv::Mat img_src, std::vector<cv::Point2d>pt_list, double yaw_rate, double speed, double dt)
{
	if (trackingFlag == true){
		if (templateflag == true){
			if (!estimateSpace(pt_list, m_pt_list_estimated)) m_eErrorCode = ERR_ESTIMATE;
			else if (!cropImage(img_src, img_cropped_template, m_pt_list_estimated)) m_eErrorCode = ERR_CROP;
			else if (!calculateDT(img_cropped_template, img_DT_template)) m_eErrorCode = ERR_CALDT;

			m_pt_list_tracking.clear();
			m_pt_list_tracking.push_back(pt_list[0]);
			m_pt_list_tracking.push_back(pt_list[1]);

			templateflag = false;
		}
		else{
			if (!predictPosition(m_pt_list_tracking, yaw_rate, speed, dt, m_pt_list_predicted)) m_eErrorCode = ERR_PREDICT;
			else if (!estimateSpace(m_pt_list_predicted, m_pt_list_estimated)) m_eErrorCode = ERR_ESTIMATE;
			if (!correctPosition(m_pt_list_estimated, m_pt_list_predicted, img_src, img_DT_template, m_pt_list_corrected)){
				m_pt_list_tracking.clear();
				m_pt_list_tracking.push_back(m_pt_list_predicted[0]);
				m_pt_list_tracking.push_back(m_pt_list_predicted[1]);
			}
			else{
				if (filterFlag == true){
					KalmanFiltering(m_pt_list_predicted, m_pt_list_corrected, m_pt_list_filtered);
					m_pt_list_tracking.clear();
					m_pt_list_tracking.push_back(m_pt_list_filtered[0]);
					m_pt_list_tracking.push_back(m_pt_list_filtered[1]);
				}
				else{
					m_pt_list_tracking.clear();
					m_pt_list_tracking.push_back(m_pt_list_corrected[0]);
					m_pt_list_tracking.push_back(m_pt_list_corrected[1]);
				}
			}
		}
	}
	else{
		if (!predictPosition(m_pt_list_tracking, yaw_rate, speed, dt, m_pt_list_predicted)){
			m_pt_list_tracking.clear();
			m_pt_list_tracking.push_back(m_pt_list_predicted[0]);
			m_pt_list_tracking.push_back(m_pt_list_predicted[1]);
		}
	}
}

void CParkingSlotTracking::Terminate()
{
}

// ===========================================================================
// Internal functions
// ---------------------------------------------------------------------------
// Initialization
bool CParkingSlotTracking::predictPosition(std::vector<cv::Point2d>pt_list, double yaw_rate, double speed, double dt, std::vector<cv::Point2d>& pt_list_dst){

	if (m_eErrorCode != ERR_NONE) return false;

	double delta_distance = speed * dt / m_XPixelMeter;
	double delta_yaw = yaw_rate * dt / 180. * M_PI;

	pt_list_dst.clear();
	for (int idx_points = 0; idx_points < pt_list.size(); idx_points++){
		// translation
		double moved_x = (double)pt_list[idx_points].x + delta_distance;

		// rotation & translation
		double tmp_x = moved_x - m_VehicleCenterX;
		double tmp_y = pt_list[idx_points].y - m_VehicleCenterY;

		double rot_x = cos(delta_yaw) * tmp_x - sin(delta_yaw) * tmp_y + m_VehicleCenterX;
		double rot_y = sin(delta_yaw) * tmp_x + cos(delta_yaw) * tmp_y + m_VehicleCenterY;

		cv::Point2d dst_points;
		dst_points.x = rot_x;
		dst_points.y = rot_y;

		pt_list_dst.push_back(dst_points);
	}
	return true;
}
bool CParkingSlotTracking::estimateSpace(std::vector<cv::Point2d> pt_list, std::vector<cv::Point2d>& pt_list_estimated){
	if (pt_list.size() < 2){
		return false;
	}
	
	double ang = 90. / 180. * M_PI;
	double areaboundary = 25.;

	double dx = (double)pt_list[1].x - (double)pt_list[0].x;
	double dy = (double)pt_list[1].y - (double)pt_list[0].y;

	double len_points = sqrt(dx * dx + dy * dy);

	double unit_x = dx / len_points;
	double unit_y = dy / len_points;

	double rot_x = cos(ang) * unit_x - sin(ang) * unit_y;
	double rot_y = sin(ang) * unit_x + cos(ang) * unit_y;

	cv::Point2d tmp_pt_list1, tmp_pt_list2;
	tmp_pt_list1.x = pt_list[0].x - areaboundary * unit_x;
	tmp_pt_list1.y = pt_list[0].y - areaboundary * unit_y;
	tmp_pt_list2.x = pt_list[1].x + areaboundary * unit_x;
	tmp_pt_list2.y = pt_list[1].y + areaboundary * unit_y;

	pt_list_estimated.clear();
	cv::Point2d tmp_pt_list3, tmp_pt_list4, tmp_pt_list5, tmp_pt_list6;
	tmp_pt_list3.x = tmp_pt_list1.x - areaboundary * rot_x;
	tmp_pt_list3.y = tmp_pt_list1.y - areaboundary * rot_y;
	pt_list_estimated.push_back(tmp_pt_list3);
	tmp_pt_list4.x = tmp_pt_list2.x - areaboundary * rot_x;
	tmp_pt_list4.y = tmp_pt_list2.y - areaboundary * rot_y;
	pt_list_estimated.push_back(tmp_pt_list4);
	tmp_pt_list5.x = tmp_pt_list2.x + areaboundary * rot_x;
	tmp_pt_list5.y = tmp_pt_list2.y + areaboundary * rot_y;
	pt_list_estimated.push_back(tmp_pt_list5);
	tmp_pt_list6.x = tmp_pt_list1.x + areaboundary * rot_x;
	tmp_pt_list6.y = tmp_pt_list1.y + areaboundary * rot_y;
	pt_list_estimated.push_back(tmp_pt_list6);

	return true;
}
bool CParkingSlotTracking::cropImage(cv::Mat img_src_ss, cv::Mat& img_cropped, std::vector<cv::Point2d> pt_list_estimated){
	if (m_eErrorCode != ERR_NONE) return false;

	if (templateflag == true){/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		m_CropImageSizeX = (int)(sqrt((pt_list_estimated[1].x - pt_list_estimated[0].x) * (pt_list_estimated[1].x - pt_list_estimated[0].x) +
							(pt_list_estimated[1].y - pt_list_estimated[0].y) * (pt_list_estimated[1].y - pt_list_estimated[0].y)));
		m_CropImageSizeY = (int)(sqrt((pt_list_estimated[2].x - pt_list_estimated[1].x) * (pt_list_estimated[2].x - pt_list_estimated[1].x) +
							(pt_list_estimated[2].y - pt_list_estimated[1].y) * (pt_list_estimated[2].y - pt_list_estimated[1].y)));

		cv::Point2f pt_i[4];
		pt_i[0] = cv::Point2f(pt_list_estimated[0].x, pt_list_estimated[0].y);
		pt_i[1] = cv::Point2f(pt_list_estimated[1].x, pt_list_estimated[1].y);
		pt_i[2] = cv::Point2f(pt_list_estimated[2].x, pt_list_estimated[2].y);
		pt_i[3] = cv::Point2f(pt_list_estimated[3].x, pt_list_estimated[3].y);

		cv::Point2f pt_b[4];
		pt_b[0] = cv::Point2f(m_CropImageSizeX,		m_CropImageSizeY);
		pt_b[1] = cv::Point2f(0,					m_CropImageSizeY);
		pt_b[2] = cv::Point2f(0,					0				);
		pt_b[3] = cv::Point2f(m_CropImageSizeX,     0				);
		
		cv::Mat Trans_mat = cv::getPerspectiveTransform(pt_i, pt_b);

		cv::Mat img_perspective;
		cv::warpPerspective(img_src_ss, img_perspective, Trans_mat, cv::Size(m_CropImageSizeX, m_CropImageSizeY));

		cv::resize(img_perspective, img_perspective, cv::Size(), m_ResizeScale, m_ResizeScale);

		cv::Mat mask_thresholding;
		cv::inRange(img_perspective, cv::Scalar(m_ThresholdImage), cv::Scalar(255), mask_thresholding);

		img_perspective.copyTo(img_cropped, mask_thresholding); 
		
		cv::imshow("test", img_perspective);
		cv::waitKey(1);
	}
	else{
		cv::Point2f pt_i[4];
		pt_i[0] = cv::Point2f(pt_list_estimated[0].x, pt_list_estimated[0].y);
		pt_i[1] = cv::Point2f(pt_list_estimated[1].x, pt_list_estimated[1].y);
		pt_i[2] = cv::Point2f(pt_list_estimated[2].x, pt_list_estimated[2].y);
		pt_i[3] = cv::Point2f(pt_list_estimated[3].x, pt_list_estimated[3].y);

		cv::Point2f pt_b[4];
		pt_b[0] = cv::Point2f(m_CropImageSizeX, m_CropImageSizeY);
		pt_b[1] = cv::Point2f(0, m_CropImageSizeY);
		pt_b[2] = cv::Point2f(0, 0);
		pt_b[3] = cv::Point2f(m_CropImageSizeX, 0);

		cv::Mat Trans_mat = cv::getPerspectiveTransform(pt_i, pt_b);

		cv::Mat img_perspective;
		cv::warpPerspective(img_src_ss, img_perspective, Trans_mat, cv::Size(m_CropImageSizeX, m_CropImageSizeY));

		cv::resize(img_perspective, img_perspective, cv::Size(), m_ResizeScale, m_ResizeScale);

		cv::Mat mask_thresholding;
		cv::inRange(img_perspective, cv::Scalar(m_ThresholdImage), cv::Scalar(255), mask_thresholding);

		img_perspective.copyTo(img_cropped, mask_thresholding);
		
		m_debug = cv::Scalar(0.0);
		img_cropped.copyTo(m_debug);
	}

	return true;
}

bool CParkingSlotTracking::calculateDT(cv::Mat img_cropped, std::vector<cv::Mat>& img_DT){
	if (m_eErrorCode != ERR_NONE) return false;

	cv::Mat tmp_img_src;
	cv::Mat tmp_img_src_inv;
	img_cropped.copyTo(tmp_img_src);
	img_cropped.copyTo(tmp_img_src_inv);

	cv::threshold(tmp_img_src, tmp_img_src, m_ThresholdImage, 255, CV_THRESH_BINARY);
	cv::threshold(tmp_img_src_inv, tmp_img_src_inv, m_ThresholdImage, 255, CV_THRESH_BINARY_INV);
	
	cv::Mat img_sobel_xpos, img_sobel_ypos, img_sobel_xneg, img_sobel_yneg;
	cv::Sobel(tmp_img_src, img_sobel_xpos, CV_8U, 1, 0, 1);
	cv::Sobel(tmp_img_src, img_sobel_ypos, CV_8U, 0, 1, 1);
	cv::Sobel(tmp_img_src_inv, img_sobel_xneg, CV_8U, 1, 0, 1);
	cv::Sobel(tmp_img_src_inv, img_sobel_yneg, CV_8U, 0, 1, 1);

	cv::threshold(img_sobel_xpos, img_sobel_xpos, m_ThresholdImage, 255, CV_THRESH_BINARY_INV);
	cv::threshold(img_sobel_ypos, img_sobel_ypos, m_ThresholdImage, 255, CV_THRESH_BINARY_INV);
	cv::threshold(img_sobel_xneg, img_sobel_xneg, m_ThresholdImage, 255, CV_THRESH_BINARY_INV);
	cv::threshold(img_sobel_yneg, img_sobel_yneg, m_ThresholdImage, 255, CV_THRESH_BINARY_INV);

	cv::Mat img_DT_xpos, img_DT_ypos, img_DT_xneg, img_DT_yneg;
	cv::distanceTransform(img_sobel_xpos, img_DT_xpos, CV_DIST_L2, 3);
	cv::distanceTransform(img_sobel_ypos, img_DT_ypos, CV_DIST_L2, 3);
	cv::distanceTransform(img_sobel_xneg, img_DT_xneg, CV_DIST_L2, 3);
	cv::distanceTransform(img_sobel_yneg, img_DT_yneg, CV_DIST_L2, 3);

	img_DT.clear();
	img_DT.push_back(img_DT_xpos);
	img_DT.push_back(img_DT_ypos);
	img_DT.push_back(img_DT_xneg);
	img_DT.push_back(img_DT_yneg);

	return true;
}
bool CParkingSlotTracking::correctPosition(std::vector<cv::Point2d> pt_list_src, std::vector<cv::Point2d> pt_list_predicted, cv::Mat img_src_ss, std::vector<cv::Mat> img_template_DT, std::vector<cv::Point2d>& pt_list_corrected){
	
	int tmp_x[4] = { pt_list_src[0].x, pt_list_src[1].x, pt_list_src[2].x, pt_list_src[3].x };
	int tmp_y[4] = { pt_list_src[0].y, pt_list_src[1].y, pt_list_src[2].y, pt_list_src[3].y };

	int max_x = *std::max_element(tmp_x, tmp_x + 4);
	int min_x = *std::min_element(tmp_x, tmp_x + 4);
	int max_y = *std::max_element(tmp_y, tmp_y + 4);
	int min_y = *std::min_element(tmp_y, tmp_y + 4);

	int tmp_img_sizex = img_src_ss.cols;
	int tmp_img_sizey = img_src_ss.rows;

	if (max_x >= tmp_img_sizex || min_x < 0 || max_y >= tmp_img_sizey || min_y < 0){
		return false;
	}

	std::vector<double> arr_cost;
	std::vector<std::vector<cv::Point2d>> tmp_point_disturbance;
	int idx = 0;
	for (int idx_x = -m_disturbanceX; idx_x <= m_disturbanceX; idx_x++){
		for (int idx_y = -m_disturbanceY; idx_y <= m_disturbanceY; idx_y++){
			for (int idx_ang = -m_disturbanceANG; idx_ang <= m_disturbanceANG; idx_ang++){
				std::vector<cv::Point2d> pt_list_disturbance;
				getDisturbancePoints(pt_list_predicted, pt_list_disturbance, idx_x, idx_y, idx_ang);

				std::vector<cv::Point2d> pt_list_disturbance_estimate;
				estimateSpace(pt_list_disturbance, pt_list_disturbance_estimate);

				cv::Mat img_cropped_disturbance;
				cropImage(img_src_ss, img_cropped_disturbance, pt_list_disturbance_estimate);

				std::vector<cv::Mat> img_DT_disturbance;
				calculateDT(img_cropped_disturbance, img_DT_disturbance);

				double cost = 0;
				for (int idx_img = 0; idx_img < img_DT_disturbance.size(); idx_img++){
					cv::Mat img_subtracted;
					pow(img_template_DT[idx_img] - img_DT_disturbance[idx_img], 2, img_subtracted);
					cv::Mat img_sqrt;
					sqrt(img_subtracted, img_sqrt);
					cost += (double)sum(img_sqrt)[0];
				}
				arr_cost.push_back(cost / (double)img_DT_disturbance.size());
				idx++;
				tmp_point_disturbance.push_back(pt_list_disturbance);
			}
		}
	}
	int index_min_cost = std::min_element(arr_cost.begin(), arr_cost.end()) - arr_cost.begin();
	
	cv::imshow("debug", m_debug);

	pt_list_corrected = tmp_point_disturbance[index_min_cost];
	
	return true;
}
bool CParkingSlotTracking::getDisturbancePoints(std::vector<cv::Point2d> pt_list_predicted, std::vector<cv::Point2d>& pt_list_disturbance, int x_disturbance, int y_disturbance, int ang_disturbance){
	
	double ang_dis = ang_disturbance / 180. * M_PI;
	
	double tmp_point1_x = pt_list_predicted[0].x + (double)x_disturbance;
	double tmp_point1_y = pt_list_predicted[0].y + (double)y_disturbance;
	double tmp_point2_x = pt_list_predicted[1].x + (double)x_disturbance;
	double tmp_point2_y = pt_list_predicted[1].y + (double)y_disturbance;
		
	double tmp_point_centerx = (tmp_point1_x + tmp_point2_x) / 2;
	double tmp_point_centery = (tmp_point1_y + tmp_point2_y) / 2;

	tmp_point1_x = tmp_point1_x - tmp_point_centerx;
	tmp_point1_y = tmp_point1_y - tmp_point_centery;
	tmp_point2_x = tmp_point2_x - tmp_point_centerx;
	tmp_point2_y = tmp_point2_y - tmp_point_centery;

	double tmp_rot_point1_x = cos(ang_dis) * tmp_point1_x - sin(ang_dis) * tmp_point1_y;
	double tmp_rot_point1_y = sin(ang_dis) * tmp_point1_x + cos(ang_dis) * tmp_point1_y;
	double tmp_rot_point2_x = cos(ang_dis) * tmp_point2_x - sin(ang_dis) * tmp_point2_y;
	double tmp_rot_point2_y = sin(ang_dis) * tmp_point2_x + cos(ang_dis) * tmp_point2_y;

	tmp_point1_x = tmp_rot_point1_x + tmp_point_centerx;
	tmp_point1_y = tmp_rot_point1_y + tmp_point_centery;
	tmp_point2_x = tmp_rot_point2_x + tmp_point_centerx;
	tmp_point2_y = tmp_rot_point2_y + tmp_point_centery;

	cv::Point2d tmp_point1;
	tmp_point1.x = tmp_point1_x;
	tmp_point1.y = tmp_point1_y;
	cv::Point2d tmp_point2;
	tmp_point2.x = tmp_point2_x;
	tmp_point2.y = tmp_point2_y;

	pt_list_disturbance.clear();
	pt_list_disturbance.push_back(tmp_point1);
	pt_list_disturbance.push_back(tmp_point2);

	return true;
}
bool CParkingSlotTracking::KalmanFiltering(std::vector<cv::Point2d> pt_list_predicted, std::vector<cv::Point2d> pt_list_corrected, std::vector<cv::Point2d>& pt_list_filtered){
	cv::Mat mat_pt_corrected = (cv::Mat_<double>(4, 1) << pt_list_corrected[0].x, pt_list_corrected[0].y, pt_list_corrected[1].x, pt_list_corrected[1].y );
	cv::Mat mat_pt_predicted = (cv::Mat_<double>(4, 1) << pt_list_predicted[0].x, pt_list_predicted[0].y, pt_list_predicted[1].x, pt_list_predicted[1].y);

	cv::Mat P_priori_matrix = m_F_matrix * m_P_matrix * m_F_matrix.t() + m_Q_matrix;
	cv::Mat K_matrix = P_priori_matrix * m_H_matrix.t() * (m_H_matrix * P_priori_matrix * m_H_matrix.t() + m_R_matrix).inv();

	cv::Mat state_priori_matrix = m_F_matrix * mat_pt_predicted;
	cv::Mat state_posteriori_matrix = state_priori_matrix + K_matrix *(mat_pt_corrected - m_H_matrix * state_priori_matrix);
	m_P_matrix = P_priori_matrix - K_matrix * m_H_matrix * P_priori_matrix;

	cv::Point2d tmp_point1;
	tmp_point1.x = state_posteriori_matrix.at<double>(0, 0);
	tmp_point1.y = state_posteriori_matrix.at<double>(1, 0);
	cv::Point2d tmp_point2;
	tmp_point2.x = state_posteriori_matrix.at<double>(2, 0);
	tmp_point2.y = state_posteriori_matrix.at<double>(3, 0);

	pt_list_filtered.clear();
	pt_list_filtered.push_back(tmp_point1);
	pt_list_filtered.push_back(tmp_point2);
	
	return true;
}

// ===========================================================================


// ===========================================================================
// External interfaces
// ---------------------------------------------------------------------------
std::vector<cv::Point2d> CParkingSlotTracking::getCornerpoint(void){
	return m_pt_list_tracking;
}


// ===========================================================================