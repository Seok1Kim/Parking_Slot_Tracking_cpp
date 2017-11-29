#include <iostream>
#include <fstream>
#include <opencv\highgui.h>
#include <opencv2\imgproc\imgproc.hpp>
#include <math.h>

#include "ParkingSlotTracking.h"


int start_frame[8] = {107, 63, 96, 82, 50, 111, 81, 100};

int set_num = 1;
std::string str_set_num = std::to_string(set_num);

std::vector<cv::Point2d> pt_list;
std::vector<cv::Point2d> pt_list_motion;
std::vector<cv::Point2d> pt_motion;
std::vector<cv::Point2d> pt_list_tracking;
std::vector<cv::Point2d> pt_list_corrected;

bool trackingFlag = false;

typedef struct{
	double dt;
	double speed;
	double yawrate;
}MotionData;

std::vector<MotionData> MotionVec;

void getMousepoint(int event, int x, int y, int flags, void* userdata);

std::vector<std::string> Split(std::string str, char delimiter);

int main(int argc, char *argv[], char *envp[]){

	if (argc < 1) { // parser
		std::cout << "incorrect arguments <path> <start frame> <end frame> <threshold> <slot_width_min> <slot_width_max>" << std::endl;
		return 0;
	}
	int frameNum = start_frame[set_num - 1];

	/********************************************************************************************************************
													Motion CSV file parser
	********************************************************************************************************************/
	char csv_file[1000];
	sprintf_s(csv_file, 1000, "D:/git/Parking_Slot_Tracking_cpp/DB/171124/rectified/set%01d/data/motion.csv", set_num);
	std::ifstream file;
	file.open(csv_file);
	std::string inputData;
	MotionVec.clear();
	MotionData tmpmotiondata;
	while (file >> inputData)
	{
		std::vector<std::string> seperatedval = Split(inputData, ';');
		tmpmotiondata.dt = stod(seperatedval[0]);
		tmpmotiondata.speed = stod(seperatedval[1]);
		tmpmotiondata.yawrate = stod(seperatedval[2]);
		MotionVec.push_back(tmpmotiondata);
	}
	file.close();

	/********************************************************************************************************************
													Mouse callback setting
	********************************************************************************************************************/
	cv::namedWindow("image");
	cv::setMouseCallback("image", getMousepoint, NULL);

	CParkingSlotTracking ParkingSlotTracking;
	CParkingSlotTracking Motion;

	char image_ss[1000];
	sprintf_s(image_ss, 1000, "D:/git/Parking_Slot_Tracking_cpp/DB/171124/rectified/set%01d/output/labeled/class_1/%08d.jpg", set_num, frameNum);

	char image_color[1000];
	sprintf_s(image_color, 1000, "D:/git/Parking_Slot_Tracking_cpp/DB/171124/rectified/set%01d/images/%08d.jpg", set_num, frameNum);

	cv::Mat image_origin = cv::imread(image_color);
	cv::Mat image_ss_origin = cv::imread(image_ss, 0);

	if (image_origin.data == NULL) {
		std::cout << "no image data in " << image_ss << std::endl;
	}

	while (1){

		cv::imshow("image", image_origin);
		cv::waitKey(20);

		if (pt_list.size() != 0){
			for (unsigned int idx = 0; idx < pt_list.size(); idx++){
				
				cv::circle(image_origin, pt_list[idx], 3, cv::Scalar(0, 0, 255), -1);
				cv::waitKey(10);
				cv::imshow("image", image_origin);
				cv::waitKey(20);
			}
			if (pt_list.size() == 2){
				pt_list_motion.clear();
				pt_list_motion.push_back(pt_list[0]);
				pt_list_motion.push_back(pt_list[1]);
				pt_list_tracking.clear();
				pt_list_tracking.push_back(pt_list[0]);
				pt_list_tracking.push_back(pt_list[1]);
			}

			while (pt_list.size() == 2){

				frameNum += 1;

				char image_ss[1000];
				sprintf_s(image_ss, 1000, "D:/git/Parking_Slot_Tracking_cpp/DB/171124/rectified/set%01d/output/labeled/class_1/%08d.jpg", set_num, frameNum);

				char image_color[1000];
				sprintf_s(image_color, 1000, "D:/git/Parking_Slot_Tracking_cpp/DB/171124/rectified/set%01d/images/%08d.jpg", set_num, frameNum);

				cv::Mat image_origin = cv::imread(image_color);
				cv::Mat image_ss_origin = cv::imread(image_ss, 0);

				if (image_origin.data == NULL) {
					std::cout << "no image data in " << image_ss << std::endl;
					break;
				}

				trackingFlag = true;
				ParkingSlotTracking.Run(trackingFlag, image_ss_origin, pt_list_tracking, MotionVec[frameNum].yawrate, MotionVec[frameNum].speed, MotionVec[frameNum].dt, pt_list_corrected);

				pt_motion.clear();
				Motion.predictPosition(pt_list_motion, MotionVec[frameNum].yawrate, MotionVec[frameNum].speed, MotionVec[frameNum].dt, pt_motion);

				std::vector<cv::Point2d> pt_tracking;
				pt_tracking = ParkingSlotTracking.getCornerpoint();

				cv::circle(image_origin, pt_tracking[0], 3, cv::Scalar(255, 0, 0), -1);
				cv::circle(image_origin, pt_tracking[1], 3, cv::Scalar(255, 0, 0), -1);
				cv::circle(image_origin, pt_motion[0], 3, cv::Scalar(0, 0, 255), -1);
				cv::circle(image_origin, pt_motion[1], 3, cv::Scalar(0, 0, 255), -1);
				cv::waitKey(20);

				pt_list_motion.clear();
				pt_list_motion.push_back(pt_motion[0]);
				pt_list_motion.push_back(pt_motion[1]);
				pt_list_tracking.clear();
				pt_list_tracking.push_back(pt_tracking[0]);
				pt_list_tracking.push_back(pt_tracking[1]);

				cv::imshow("image_test", image_origin);
				cv::waitKey(20);
			}
		}
	}
}

void getMousepoint(int event, int x, int y, int flags, void* userdata){
	if (event == cv::EVENT_FLAG_LBUTTON){
		cv::Point2d it = cv::Point2d(x, y);
		pt_list.push_back(it);
	}
	else if(event == cv::EVENT_FLAG_RBUTTON){
		pt_list.clear();
	}
}

std::vector<std::string> Split(std::string str, char delimiter)
{
	std::vector<std::string> internal;
	std::stringstream ss(str);
	std::string tok;

	while (getline(ss, tok, delimiter))
	{
		internal.push_back(tok);
	}

	return internal;
}