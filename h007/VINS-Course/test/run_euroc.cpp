
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
string sConfig_path = "../config/";

std::shared_ptr<System> pSystem;

void PubImuData()
{
	string sImu_data_file = sConfig_path + "MH_05_imu0.txt";
	cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
	ifstream fsImu;
	fsImu.open(sImu_data_file.c_str());
	if (!fsImu.is_open())
	{
		cerr << "Failed to open imu file! " << sImu_data_file << endl;
		return;
	}

	std::string sImu_line;
	double dStampNSec = 0.0;
	Vector3d vAcc;
	Vector3d vGyr;
	while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data
	{
		std::istringstream ssImuData(sImu_line);
		ssImuData >> dStampNSec >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
		// cout << "Imu t: " << fixed << dStampNSec << " gyr: " << vGyr.transpose() << " acc: " << vAcc.transpose() << endl;
		pSystem->PubImuData(dStampNSec / 1e9, vGyr, vAcc);
		usleep(5000*nDelayTimes);
	}
	fsImu.close();
}

void PubImageData()
{
    // open imu file
    const QString imu_path = "../../vio_data_simulation/gen/imu_pose.txt";
    QFile imu_file(imu_path);
    if (!imu_file.open(QIODevice::ReadOnly | QIODevice::Text)){
        cerr << "File opening failed" << " " << imu_path.toStdString() << endl;
        std::exit(-1);
    }
    QTextStream imu_stream(&imu_file);
    // timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
    QString imu_line = imu_stream.readLine();
    double last_imu_timestamp = imu_line.split(" ")[0].toDouble();

    const QString cam_pose_path = "../../vio_data_simulation/gen/cam_pose.txt";
    QFile cam_pose_file(cam_pose_path);
    if (!cam_pose_file.open(QIODevice::ReadOnly | QIODevice::Text)){
        cerr << "File opening failed" << " " << cam_pose_path.toStdString() << endl;
        std::exit(-1);
    }
    QTextStream cam_pose_stream(&cam_pose_file);
    QString line = cam_pose_stream.readLine();
    std::vector<double> timestamps;
    int frame_id = 0;
    cv::namedWindow("source image");
    map<int, std::vector<std::vector<double>>> fid2feature;
    while (!line.isNull()){
//        line = cam_pose_stream.readLine();
        QStringList line_split = line.split(" ");
        double time_stamp = line_split[0].toDouble();
        cout << "Cam at " << time_stamp << endl;
        timestamps.push_back(time_stamp);
//        line = cam_pose_stream.readLine();
        // Read features from this path
        std::vector<std::vector<double>> feature_of_this_frame;
        Mat img(ROW*2, COL*2, CV_8UC1);
        memset(img.data, 0, img.rows * img.cols);
        {
            QString frame_path = QString("../../vio_data_simulation/gen/keyframe/all_points_%1.txt").arg(frame_id);
            QFile frame_file(frame_path);
            if (!frame_file.open(QIODevice::ReadOnly | QIODevice::Text)){
                cerr << "File opening failed" << " " <<  frame_path.toStdString() << endl;
                std::exit(-1);
            }
            QTextStream frame_stream(&frame_file);
            QString frame_line = frame_stream.readLine();
            int point_id = 0;
            while(!frame_line.isNull()){
                QStringList frame_line_split = frame_line.split(" ");
                // x, y, z, 1, u, v
                double x = frame_line_split[0].toDouble();
                double y = frame_line_split[1].toDouble();
                double z = frame_line_split[2].toDouble();
                double u = frame_line_split[4].toDouble();
                double v = frame_line_split[5].toDouble();
                double cx = u *1 + 0;
                double cy = v *1 + 0;
                double vx = 0;
                double vy = 0;
                if(frame_id > 0){
                    vector<double> & last_feature_point = fid2feature[frame_id-1][point_id];
                    double lastTime = last_feature_point[0];
                    double last_cx = last_feature_point[6];
                    double last_cy = last_feature_point[7];
                    vx = (cx - last_cx) / (time_stamp - lastTime);
                    vy = (cy - last_cy) / (time_stamp - lastTime);
                } else{
                    // The first frame, velocity can't be estimated.
                }
                auto featurepoint = std::vector<double>(); //push_back({time_stamp, x, y, z, u, v, cx, cy, vx, vy});
                featurepoint.push_back(time_stamp);  //0
                featurepoint.push_back(x); //1
                featurepoint.push_back(y); //2
                featurepoint.push_back(z); //3
                featurepoint.push_back(u); // 4
                featurepoint.push_back(v); // 5
                featurepoint.push_back(cx); // 6
                featurepoint.push_back(cy); // 7
                featurepoint.push_back(vx); // 8
                featurepoint.push_back(vy); // 9
                feature_of_this_frame.push_back(featurepoint);
//                cv::circle(img, cv::Point2d(cx, cy), 3, cv::Scalar(255, 255, 255), -1);
                frame_line = frame_stream.readLine();
                point_id += 1;
            }
        }
        fid2feature[frame_id] = feature_of_this_frame;
        while(last_imu_timestamp <= time_stamp && !imu_line.isNull()){
            QStringList imu_line_split = imu_line.split(" ");
            Q_ASSERT(imu_line_split.size() == 14);
            double dStampNSec = imu_line_split[0].toDouble();
            Vector3d vAcc = Vector3d(imu_line_split[11].toDouble(),imu_line_split[12].toDouble(),imu_line_split[13].toDouble());
            Vector3d vGyr = Vector3d(imu_line_split[8].toDouble(),imu_line_split[9].toDouble(),imu_line_split[10].toDouble());
            if(dStampNSec > 0){
                pSystem->PubImuData(dStampNSec, vGyr, vAcc);
            }
            imu_line = imu_stream.readLine();
            last_imu_timestamp = imu_line.split(" ")[0].toDouble();
            usleep(5000*nDelayTimes);
        }
        // publish next imu data
        if(!imu_line.isNull()){
            QStringList imu_line_split = imu_line.split(" ");
            Q_ASSERT(imu_line_split.size() == 14);
            double dStampNSec = imu_line_split[0].toDouble();
            Vector3d vAcc = Vector3d(imu_line_split[11].toDouble(),imu_line_split[12].toDouble(),imu_line_split[13].toDouble());
            Vector3d vGyr = Vector3d(imu_line_split[8].toDouble(),imu_line_split[9].toDouble(),imu_line_split[10].toDouble());
            if(dStampNSec > 0){
                pSystem->PubImuData(dStampNSec, vGyr, vAcc);
            }
            imu_line = imu_stream.readLine();
            last_imu_timestamp = imu_line.split(" ")[0].toDouble();
            usleep(5000*nDelayTimes);
        }
        pSystem->PubImageData(time_stamp, feature_of_this_frame, img);
        line = cam_pose_stream.readLine();
        usleep(50000*nDelayTimes);
        frame_id += 1;
        cout << frame_id << "published" << endl;
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
	pSystem.reset(new System(sConfig_path));
	
	std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);
		
	// sleep(5);
//	std::thread thd_PubImuData(PubImuData);

	std::thread thd_PubImageData(PubImageData);

#ifdef __linux__	
//	std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
	DrawIMGandGLinMainThrd();
#endif

//	thd_PubImuData.join();
	thd_PubImageData.join();

	 thd_BackEnd.join();
	// thd_Draw.join();

	cout << "main end... see you ..." << endl;
	return 0;
}
