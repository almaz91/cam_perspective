#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Mat             rot2euler(const Mat &rotationMatrix);
vector<Point2d> Form2D(const Mat& src);
pair<Mat, Mat>  perspFind(const Mat& im, const vector<Point3d>& model_points, const vector<Point2d>& image_points);
vector<Point3d> Form3D();

int main()
{
	Mat src = imread("test.jpg");
	if (src.data == nullptr)
	{
		cout << "There is no image on the folder. Please put image on the folder with name \"test.jpg\"" << endl;
		system("pause");
		return 0;
	}

	// 3D model points forming
	vector<Point3d> model_points = Form3D();

	// 2D model points forming
	vector<Point2d> image_points = Form2D(src);

	// Euler angles and translation vector finding
	pair<Mat, Mat> result = perspFind(src, model_points, image_points);

	cout << "Euler angles: " << endl << result.first << endl;
	cout << "Translation Vector: " << endl << result.second << endl;

	system("pause");
	return 0;
}

Mat rot2euler(const Mat &rotationMatrix)
{
	Mat euler(3, 1, CV_64F);

	double m00 = rotationMatrix.at<double>(0, 0);
	double m02 = rotationMatrix.at<double>(0, 2);
	double m10 = rotationMatrix.at<double>(1, 0);
	double m11 = rotationMatrix.at<double>(1, 1);
	double m12 = rotationMatrix.at<double>(1, 2);
	double m20 = rotationMatrix.at<double>(2, 0);
	double m22 = rotationMatrix.at<double>(2, 2);

	double x, y, z;

	// Assuming the angles are in radians.
	if (m10 > 0.998) { // singularity at north pole
		x = 0;
		y = CV_PI / 2;
		z = atan2(m02, m22);
	}
	else if (m10 < -0.998) { // singularity at south pole
		x = 0;
		y = -CV_PI / 2;
		z = atan2(m02, m22);
	}
	else
	{
		x = atan2(-m12, m11);
		y = asin(m10);
		z = atan2(-m20, m00);
	}

	euler.at<double>(0) = x;
	euler.at<double>(1) = y;
	euler.at<double>(2) = z;

	return euler;
}

vector<Point2d> Form2D(const Mat& src)
{
	Mat bw;

	int thresh = 85;

	int maxx, maxy, minx, miny;
	int maxx_y, maxy_x, minx_y, miny_x;

	maxx = maxy = -numeric_limits<int>::max();
	minx = miny = numeric_limits<int>::max();
	maxx_y = maxy_x = 0;
	minx_y = miny_x = 0;

	vector<Point2d> image_points;

	int blockSize = 2;
	int apertureSize = 3;
	double k = 0.04;

	cvtColor(src, bw, COLOR_BGR2GRAY);

	Mat dst = Mat::zeros(src.size(), CV_32FC1);
	cornerHarris(bw, dst, blockSize, apertureSize, k);
	Mat dst_norm, dst_norm_scaled;
	normalize(dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat());
	convertScaleAbs(dst_norm, dst_norm_scaled);
	for (int i = 0; i < dst_norm.rows; i++)
	{
		for (int j = 0; j < dst_norm.cols; j++)
		{
			if ((int)dst_norm.at<float>(i, j) > thresh)
			{
				if (j < minx)
				{
					minx = j;
					minx_y = i;
				}
				if (j > maxx)
				{
					maxx = j;
					maxx_y = i;
				}
				if (i < miny)
				{
					miny_x = j;
					miny = i;
				}
				if (i > maxy)
				{
					maxy_x = j;
					maxy = i;
				}
			}
		}
	}

	image_points.push_back(Point2d(minx, minx_y));     // Upper left point of paper sheeet
	image_points.push_back(Point2d(miny_x, miny));     // Lower left point of paper sheeet
	image_points.push_back(Point2d(maxy_x, maxy));     // Upper right point of paper sheeet
	image_points.push_back(Point2d(maxx, maxx_y));     // Lower right point of paper sheeet

	circle(dst_norm_scaled, Point2d(minx, minx_y), 5, Scalar(0), 2, 8, 0);
	circle(dst_norm_scaled, Point2d(maxx, maxx_y), 5, Scalar(0), 2, 8, 0);
	circle(dst_norm_scaled, Point2d(miny_x, miny), 5, Scalar(0), 2, 8, 0);
	circle(dst_norm_scaled, Point2d(maxy_x, maxy), 5, Scalar(0), 2, 8, 0);

	//imshow("corners_window", dst_norm_scaled);
	//waitKey(2000);
	return image_points;
}

pair<Mat, Mat> perspFind(const Mat& im, const vector<Point3d>& model_points, const vector<Point2d>& image_points)
{
	// Camera internals
	double focal_length = im.cols;                                  // Approximate focal length
	Point2d center = Point2d(im.cols / 2, im.rows / 2);
	Mat camera_matrix = (Mat_<double>(3, 3) << focal_length, 0, center.x, 0, focal_length, center.y, 0, 0, 1);
	Mat dist_coeffs = Mat::zeros(4, 1, DataType<double>::type);     // Assuming no lens distortion

																	// Output rotation and translation
	Mat rotation_vector;                                            // Rotation in axis-angle form
	Mat translation_vector;

	// Solve for pose by three points
	solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false, SOLVEPNP_SQPNP);

	double Rtmp[9] = { 0.0 };
	Mat R(3, 3, DataType<double>::type, Rtmp);
	Rodrigues(rotation_vector, R);                                  // Convert the rotation matrix to the rotation vector

	Mat euler_angles(3, 1, CV_64F);
	euler_angles = rot2euler(R) * 180 / CV_PI;                      // Convert to Euler angles

	return make_pair(euler_angles, translation_vector);
}

vector<Point3d> Form3D()
{
	vector<Point3d> model_points;

	model_points.push_back(Point3d(0.0f, 0.0f, 0.0f));               // Upper left point of paper sheeet
	model_points.push_back(Point3d(0.0f, 29.7f, 0.0f));              // Lower left point of paper sheeet
	model_points.push_back(Point3d(21.0f, 0.0f, 0.0f));              // Upper right point of paper sheeet
	model_points.push_back(Point3d(21.0f, 29.7f, 0.0f));             // Lower right point of paper sheeet

	return model_points;
}
