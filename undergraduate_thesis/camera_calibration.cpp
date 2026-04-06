#include<opencv2/opencv.hpp>
#include<fstream>
#include<iostream>

using namespace std;
using namespace cv;

//通过函数计算重投影误差
double computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
	const vector< vector< Point2f > >& imagePoints,
	const vector< Mat >& rvecs, const vector< Mat >& tvecs,
	const Mat& cameraMatrix, const Mat& distCoeffs)
{
	vector< Point2f > imagePoints2;
	int i, totalPoints = 0;
	double totalErr = 0, err;
	vector< float > perViewErrors;
	perViewErrors.resize(objectPoints.size());

	for (i = 0; i < (int)objectPoints.size(); ++i) {
		projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
			distCoeffs, imagePoints2);
		err = norm(Mat(imagePoints[i]), Mat(imagePoints2), NORM_L2);
		int n = (int)objectPoints[i].size();
		perViewErrors[i] = (float)sqrt(err*err / n);
		totalErr += err * err;
		totalPoints += n;
	}
	return sqrt(totalErr / totalPoints);
	cout << " computeReprojectionErrors finished!" << endl;
}


int main()
{
	//##############################1、声明变量##############################
	Size boardSize;															//标定板列方向、行方向角点个数
	Size imageSize;															//标定图像大小
	int num_imgs;															//标定图像数量
	float square_size;														//每个黑白格的尺寸大小（要求黑白格为正方形，长宽相等）
	string imgs_directory;												//标定图像所在目录
	string imgs_suffix;													//标定图像后缀名
	string para_directory;												//标定参数保存目录
	double err;																	//重投影误差
	vector<vector<Point3f>> object_points;				//参考坐标点的集合
	vector<vector<Point2f>> image_points;				//角点的集合
	//##############################2、参数初始化############################
	//以下参数更换成自己的
	boardSize.width = 11;																	//行方向角点个数，填反了也没关系，只要前后保持一致就可以
	boardSize.height = 8;																	//列方向角点个数，填反了也没关系，只要前后保持一致就可以
	imageSize.height =512;																//标定图像高度
	imageSize.width = 512;																//标定图像宽度
	num_imgs = 20;																			//标定图像张数
	square_size = 15.0f;																		//每个黑白格的尺寸大小
	//标定图像所在文件夹路径
	imgs_directory = "/home/eric/undergraduate_thesis/build/image/use/";						
	imgs_suffix = ".jpg";																	//标定图像后缀名
	para_directory = "./calibration_results/cameraParaments.xml";				//标定参数保存路径
	
	//##############################3、读取标定图片并计算角点#####################
	Mat img, gray;									//标定图像、标定图像灰度图
	vector<Point2f> corners;				//存放单张标定图像的角点的容器

	for (int k = 1; k <= num_imgs; k++)
	{
		string imgs_filename = imgs_directory + to_string(k) + imgs_suffix;
		//读取标定图像，默认以彩色图像（BGR）格式读取
		img = imread(imgs_filename);

		//将彩色图像转换成灰度图
		cvtColor(img, gray, COLOR_BGR2GRAY);

		//寻找角点 
		bool found = false;
		found = findChessboardCorners(gray, boardSize, corners, CALIB_CB_EXHAUSTIVE | CALIB_CB_ACCURACY);

		if (found)
		{
			//绘制角点
			drawChessboardCorners(img, boardSize, corners, found);
			//保存角点坐标
			std::ofstream myfile;
			string txtName = "./calibration_results/corner" + to_string(k) + ".txt";
			myfile.open(txtName, ios::out);
			for (unsigned int j = 0; j < corners.size(); j++)
			{
				float x_pos = corners.at(j).x;
				float y_pos = corners.at(j).y;
				myfile << x_pos << " " << y_pos << std::endl;
			}
			myfile.close();

            string imgName = "./calibration_results/corners_color_" + to_string(k) + ".jpg";
            imwrite(imgName, img);
		}
		//生成参考坐标点，棋盘格角点的真实世界坐标
		vector< Point3f > obj;
		for (int i = 0; i < boardSize.height; i++)
			for (int j = 0; j < boardSize.width; j++)
				obj.push_back(Point3f((float)j * square_size, (float)i * square_size, 0));

		if (found) {
			cout << k << ". Found corners!" << endl;
			image_points.push_back(corners);
			object_points.push_back(obj);
		}
	}
	cout << "calchessboardCorners finished!" << endl;
	//##############################4、开始标定##############################
	Mat intrinsic_matrix;
	Mat distortion_matrix;
	vector< Mat > rvecs, tvecs;
	//单目相机标定核心函数
	double rms = calibrateCamera(object_points, image_points, imageSize, intrinsic_matrix, distortion_matrix, rvecs, tvecs,
		0, TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 30, DBL_EPSILON));

	cout << "intrinsic_matrix" << intrinsic_matrix << endl;
	cout << "distortion_matrix" << distortion_matrix << endl;
	//cout << "rvecs" << rvecs << endl;
	//cout << "tvecs" << tvecs << endl;
	//##############################5、矫正图像（可选）#########################
	//矫正第一张标定图像，可替换自己像矫正的
	Mat src_img, dst_img;
	String src_img_path = imgs_directory + "1" + imgs_suffix;
	src_img = imread(src_img_path);
	undistort(src_img, dst_img, intrinsic_matrix, distortion_matrix);
	//保存矫正后图像
	imwrite("./calibration_results/dst_img.jpg", dst_img);
	//##############################6、计算重投影误差（可选）####################
	//通过函数计算重投影误差，其值和rms=calibrateCamera()输出的一摸一样
	err = computeReprojectionErrors(object_points, image_points, rvecs, tvecs, intrinsic_matrix, distortion_matrix);
	std::cout << "rms = " << rms << std::endl;
	std::cout << "err = " << err << std::endl;

	//##############################7、保存相机参数##############################

	FileStorage fs(para_directory, FileStorage::WRITE);
	fs << "imageSize" << imageSize;
	fs << "boardSize" << boardSize;
	fs << "square_size" << square_size;
	fs << "intrinsic_matrix" << intrinsic_matrix;
	fs << "distortion_matrix" << distortion_matrix;
	if (!rvecs.empty() && !tvecs.empty())
	{
		CV_Assert(rvecs[0].type() == tvecs[0].type());
		for (int i = 0; i < (int)rvecs.size(); i++)
		{
			//指定第一张标定图像对应的R、T作为外参
			if (i == 0)
			{
				CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
				CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
				//*.t() is MatExpr (not Mat) so we can use assignment operator
				Mat R = rvecs[i].t();
				Mat T = tvecs[i].t();
				fs << "R" << R;
				fs << "T" << T;
			}
		}
		//cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
		fs.release();
	}
	cout << "save paramengts successfully!" << endl;
	cout << "monocular calibration finished!" << endl;
	return 0;
}