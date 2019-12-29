
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
	double last_timeStamp = -1e99;
	while (!line.isNull()) // read imu data
	{   QStringList line_split = line.split(" ");
	    Q_ASSERT(line_split.size() == 14);
	    double dStampNSec = line_split[0].toDouble();
        Vector3d vAcc = Vector3d(line_split[11].toDouble(),line_split[12].toDouble(),line_split[13].toDouble());
        Vector3d vGyr = Vector3d(line_split[8].toDouble(),line_split[9].toDouble(),line_split[10].toDouble());
        if(dStampNSec > 0){
            pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
        }
		usleep(5000*nDelayTimes);
        line = imu_stream.readLine();
        if(last_timeStamp >= dStampNSec){
            cerr << "last_timeStamp < dStampNSec" << endl;
        }
        last_timeStamp = dStampNSec / 1e9;
	}
    imu_file.close();
}

void PubImageData()
{
    const QString cam_pose_path = "/data3/zyx/yks/vio_homeworks/h007/vio_data_simulation/bin/cam_pose.txt";
    QFile cam_pose_file(cam_pose_path);
    if (!cam_pose_file.open(QIODevice::ReadOnly | QIODevice::Text)){
        std::exit(-1);
    }
    QTextStream cam_pose_stream(&cam_pose_file);
    QString line = cam_pose_stream.readLine();
    std::vector<double> timestamps;
    int frame_id = 0;
    cv::namedWindow("source image");
    map<int, std::vector<std::vector<double>>> fid2feature;
    while (!line.isNull()){
        line = cam_pose_stream.readLine();
        QStringList line_split = line.split(" ");
        auto time_stamp = line_split[0].toDouble();
        timestamps.push_back(time_stamp);
        line = cam_pose_stream.readLine();
        // Read features from this path
        std::vector<std::vector<double>> feature_of_this_frame;
        Mat img(ROW*2, COL*2, CV_8UC1);
        memset(img.data, 0, img.rows * img.cols);
        {
            QString frame_path = QString("/data3/zyx/yks/vio_homeworks/h007/vio_data_simulation/bin/keyframe/all_points_%1.txt").arg(frame_id);
            QFile frame_file(frame_path);
            if (!frame_file.open(QIODevice::ReadOnly | QIODevice::Text)){
                std::exit(-1);
            }
            QTextStream frame_stream(&frame_file);
            QString frame_line = frame_stream.readLine();
            int point_id = 0;
            while(!frame_line.isNull()){
                QStringList frame_line_split = frame_line.split(" ");
                // x, y, z, 1, u, v
                double u = frame_line_split[4].toDouble();
                double v = frame_line_split[5].toDouble();
                double cx = u *460 + 255;
                double cy = v *460 + 255;
                double vx = 0;
                double vy = 0;
                if(frame_id > 0){
                    vector<double> & last_feature_point = fid2feature[frame_id-1][point_id];
                    double lastTime = last_feature_point[0];
                    double last_cx = last_feature_point[3];
                    double last_cy = last_feature_point[4];
                    vx = (cx - last_cx) / (time_stamp - lastTime);
                    vy = (cy - last_cy) / (time_stamp - lastTime);
                } else{
                    // The first frame, velocity can't be estimated.
                }
                feature_of_this_frame.push_back({time_stamp, u, v, cx, cy, vx, vy});
                cv::circle(img, cv::Point2d(cx, cy), 3, cv::Scalar(255, 255, 255), -1);
                frame_line = frame_stream.readLine();
                point_id += 1;
            }
        }
        fid2feature[frame_id] = feature_of_this_frame;
        pSystem->PubImageData(time_stamp / 1e9, feature_of_this_frame, img);
        line = cam_pose_stream.readLine();
        usleep(50000*nDelayTimes);
        frame_id += 1;
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
