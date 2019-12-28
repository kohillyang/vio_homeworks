
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <thread>
#include <iomanip>
#include <QString>
#include <QFile>
#include <QIODevice>
#include <QTextStream>
#include <cv.h>
#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <eigen3/Eigen/Dense>
#include "System.h"

using namespace std;
using namespace cv;
using namespace Eigen;

const int nDelayTimes = 2;
string sData_path = "/home/dataset/EuRoC/MH-05/mav0/";
string sConfig_path = "/data3/zyx/yks/vio_homeworks/h007/VINS-Course/config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
    const QString imu_path = "/data3/zyx/yks/vio_homeworks/h007/vio_data_simulation/bin/imu_pose.txt";
    QFile imu_file(imu_path);
    if (!imu_file.open(QIODevice::ReadOnly | QIODevice::Text)){
        std::exit(-1);
    }
    QTextStream imu_stream(&imu_file);
    // timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
	QString line = imu_stream.readLine();
	while (!line.isNull()) // read imu data
	{   QStringList line_split = line.split(" ");
	    Q_ASSERT(line_split.size() == 14);
	    double dStampNSec = line_split[0].toDouble();
        Vector3d vAcc = Vector3d(line_split[11].toDouble(),line_split[12].toDouble(),line_split[13].toDouble());
        Vector3d vGyr = Vector3d(line_split[8].toDouble(),line_split[9].toDouble(),line_split[10].toDouble());
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
    imu_file.close();
}

void PubImageData()
{
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		pSystem->PubImageData(dStampNSec / 1e9, img);
	}
}

#ifdef __APPLE__
// support for MacOS
void DrawIMGandGLinMainThrd(){
	string sImage_file = sConfig_path + "MH_05_cam0.txt";

	cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

	ifstream fsImage;
	fsImage.open(sImage_file.c_str());
	if (!fsImage.is_open())
	{
		cerr << "Failed to open image file! " << sImage_file << endl;
		return;
	}

	std::string sImage_line;
	double dStampNSec;
	string sImgFileName;

	pSystem->InitDrawGL();
	while (std::getline(fsImage, sImage_line) && !sImage_line.empty())
	{
		std::istringstream ssImuData(sImage_line);
		ssImuData >> dStampNSec >> sImgFileName;
		// cout << "Image t : " << fixed << dStampNSec << " Name: " << sImgFileName << endl;
		string imagePath = sData_path + "cam0/data/" + sImgFileName;

		Mat img = imread(imagePath.c_str(), 0);
		if (img.empty())
		{
			cerr << "image is empty! path: " << imagePath << endl;
			return;
		}
		//pSystem->PubImageData(dStampNSec / 1e9, img);
		cv::Mat show_img;
		cv::cvtColor(img, show_img, CV_GRAY2RGB);
		if (SHOW_TRACK)
		{
			for (unsigned int j = 0; j < pSystem->trackerData[0].cur_pts.size(); j++)
			{
				double len = min(1.0, 1.0 *  pSystem->trackerData[0].track_cnt[j] / WINDOW_SIZE);
				cv::circle(show_img,  pSystem->trackerData[0].cur_pts[j], 2, cv::Scalar(255 * (1 - len), 0, 255 * len), 2);
			}

			cv::namedWindow("IMAGE", CV_WINDOW_AUTOSIZE);
			cv::imshow("IMAGE", show_img);
		  // cv::waitKey(1);
		}

		pSystem->DrawGLFrame();
		usleep(50000*nDelayTimes);
	}
	fsImage.close();

} 
#endif

int main(int argc, char **argv)
{
//	if(argc != 3)
//	{
//		cerr << "./run_euroc PATH_TO_FOLDER/MH-05/mav0 PATH_TO_CONFIG/config \n"
//			<< "For example: ./run_euroc /home/stevencui/dataset/EuRoC/MH-05/mav0/ ../config/"<< endl;
//		return -1;
//	}
//	sData_path = argv[1];
//	sConfig_path = argv[2];

	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	// sleep(5);
	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

#ifdef __linux__	
//	std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
	DrawIMGandGLinMainThrd();
#endif

	thd_PubImuData.join();
	thd_PubImageData.join();

	// thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
