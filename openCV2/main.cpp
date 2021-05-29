#include <iostream>
#include <opencv4/opencv2/core/core.hpp>
#include <opencv4/opencv2/highgui/highgui.hpp>
#include <opencv4/opencv2/calib3d.hpp>
#include <opencv4/opencv2/imgproc/imgproc.hpp>

using namespace std;
using namespace cv;

Mat src, src_gray;
    Mat bw;
int thresh = 85;
int max_thresh = 255;
const char* source_window = "Source image";
const char* corners_window = "Corners detected";

Mat rot2euler(const Mat & rotationMatrix)
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

void cornerHarris_demo( int, void* )
{
    int maxx, maxy, minx, miny;
    int maxx_y, maxy_x, minx_y, miny_x;
    maxx = maxy = -std::numeric_limits<int>::max();
    minx = miny = std::numeric_limits<int>::max();

    maxx_y = maxy_x = 0;
    minx_y = miny_x = 0;

    std::vector<cv::Point2d> image_points;

    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    Mat dst = Mat::zeros( src.size(), CV_32FC1 );
    cornerHarris( bw, dst, blockSize, apertureSize, k );
    Mat dst_norm, dst_norm_scaled;
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > thresh )
            {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(0), 2, 8, 0 );

                if(j < minx)
                {
                    minx = j;
                    minx_y = i;
                }
                if(j > maxx)
                {
                    maxx = j;
                    maxx_y = i;
                }
                if(i < miny)
                {
                    miny_x = j;
                    miny = i;
                }
                if(i > maxy)
                {
                    maxy_x = j;
                    maxy = i;
                }
            }
        }
    }

    image_points.push_back( cv::Point2d(minx, minx_y) );     // Upper left point
    image_points.push_back( cv::Point2d(maxx, maxx_y) );     // Lower left point
    image_points.push_back( cv::Point2d(miny_x, miny) );     // Upper right point

    namedWindow( corners_window );
    imshow( corners_window, dst_norm_scaled );
}

std::vector<cv::Point2d> segmentize() {

    Mat src = imread("test.jpg");
    if (src.data == nullptr)
    {
        cout << "There is no source image in folder" << endl;
        return {};
    }

    //imshow("src", src);

    // Создаем бинарное изображение из исходного (RU)

    cvtColor( src, bw, COLOR_BGR2GRAY );
    namedWindow( source_window );
    createTrackbar( "Threshold: ", source_window, &thresh, max_thresh, cornerHarris_demo );

//    cvtColor(src, bw, COLOR_BGRA2GRAY);
//    threshold(bw, bw, 120, 255, THRESH_BINARY);
//    namedWindow( "bw" );
//    //imshow("bw", bw);

//    // Выполняем алгоритм distance transform (RU)
//    Mat dist;
//    distanceTransform(bw, dist, DIST_L2, 5);

//    // Нормализуем изображение в диапозоне {0.0 1.0} (RU)
//    normalize(dist, dist, 0, 1., NORM_MINMAX);
//    //imshow("dist", dist);

//    // Выполняем Threshold для определения пиков
//    // Это будут маркеры для объектов на переднем плане (RU)
//    threshold(dist, dist, 0.5, 1., THRESH_BINARY);
//    //imshow("dist2", dist);


    cornerHarris_demo(0, 0);
    waitKey(0);
    return {};

/*
    int maxx, maxy, minx, miny;
    int maxx_y, maxy_x, minx_y, miny_x;
    maxx = maxy = -std::numeric_limits<int>::max();
    minx = miny = std::numeric_limits<int>::max();

    maxx_y = maxy_x = 0;
    minx_y = miny_x = 0;

    for(int y = 0; y < dist.rows; ++y)
    {
        for(int x = 0; x < dist.cols; ++x)
        {
            const cv::Vec3b &px = dist.at<cv::Vec3b>(y, x);
            if(px(0) == 0 && px(1) == 0 && px(2) == 0)
            {
                continue;
            }
            if(x < minx)
            {
                minx = x;
                minx_y = y;
            }
            if(x > maxx)
            {
                maxx = x;
                maxx_y = y;
            }
            if(y < miny)
            {
                miny_x = x;
                miny = y;
            }
            if(y > maxy)
            {
                maxy_x = x;
                maxy = y;
            }
        }
    }

    // 2D image points
    std::vector<cv::Point2d> image_points;
    image_points.push_back( cv::Point2d(minx, minx_y) );     // Upper left point
    image_points.push_back( cv::Point2d(maxx, maxx_y) );     // Lower left point
    image_points.push_back( cv::Point2d(miny_x, miny) );     // Upper right point
//    //image_points.push_back( cv::Point2d(maxy_x, maxy) );     // Lower right point

    waitKey(0);
    return image_points;
*/

//    cv::Mat subimg;
//    dist(cv::Rect(cv::Point(minx, minx_y), cv::Point(maxx, maxx_y))).copyTo(subimg);

//    imshow("dist3", subimg);
//    // Create the CV_8U version of the distance image
//    // It is needed for cv::findContours() (EN)
//    // Создаем CV_8U версию distance изображения
//    // Это нужно для фуекции cv::findContours() (RU)
//    Mat dist_8u;
//    dist.convertTo(dist_8u, CV_8U);

//    // Find total markers (EN)
//    // Находим все маркеры (RU)
//    vector<vector<Point> > contours;
//    findContours(dist_8u, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

//    auto ncomp = static_cast<int>(contours.size());

//    // Create the marker image for the watershed algorithm (EN)
//    // Создаем маркерное изображение для алгоритма watershed (RU)
//    Mat markers = Mat::zeros(dist.size(), CV_32SC1);

//    for ( int i = 0; i< contours.size(); i++ ) {
//        // draw contour
//        cv::drawContours(markers, contours, i, cv::Scalar(255,0,0), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());

//        // draw bounding rect
//        cv::Rect rect = boundingRect(cv::Mat(contours[i]));
//        cv::rectangle(markers, rect.tl(), rect.br(), cv::Scalar(0,255,0), 2, 8, 0);

//        // draw rotated rect
//        cv::RotatedRect minRect = minAreaRect(cv::Mat(contours[i]));
//        cv::Point2f rect_points[4];
//        minRect.points( rect_points );
//        for ( int j = 0; j < 4; j++ ) {
//            cv::line( markers, rect_points[j], rect_points[(j+1)%4], cv::Scalar(0,0,255), 1, 8 ); // blue
//        }
//    }



    // Draw the foreground markers (EN)
    // Рисуем маркеры переднего плана (RU)
//    for (int i =0; i < ncomp; i++)
//        drawContours(markers, contours, i, Scalar::all(i+1), -1);

//    // вывести координаты первого контура
//    vector<Point> firstcontour = contours.at(0);
//    for (int i = 0; i < firstcontour.size(); i++) {
//        Point coordinate_i_ofcontour = firstcontour.at(i);
//        cout << endl << "contour with coordinates: x = " << coordinate_i_ofcontour.x << " y = " << coordinate_i_ofcontour.y;
//    }


//    // Draw background marker
//    //circle(markers, Point(400,400), 100, CV_RGB(255,255,255), 1);
//    imshow("markers", markers);

//    // Perform the watershed algorithm (EN)
//    // Выполняем алгоритм watershed (RU)
//    watershed(src, markers);

//    // Generate random colors (EN)
//    // Генерируем случайные цвета (RU)
//    vector<Vec3b> colors;

//    for (int i = 0; i < ncomp; i++)
//    {
//        int b = theRNG().uniform(0, 255);
//        int g = theRNG().uniform(0, 255);
//        int r = theRNG().uniform(0, 255);

//        colors.emplace_back(static_cast<uchar>(b), static_cast<uchar>(g), static_cast<uchar>(r));
//    }

//    // Create the result image (EN)
//    // Создаем результирующее изображение (RU)
//    Mat dst = Mat::zeros(markers.size(), CV_8UC3);

//    // Fill labeled objects with random colors (EN)
//    // Заполняем помеченные объекты случайным цветом (RU)
//    for (int i = 0; i < markers.rows; i++)
//    {
//        for (int j = 0; j < markers.cols; j++)
//        {
//            int index = markers.at<int>(i,j);
//            dst.at<Vec3b>(i,j) = index > 0 && index <= ncomp ? colors[index-1] : Vec3b(0,0,0);
//        }
//    }

    //imshow("dst", dst);
}

int findpose(std::vector<cv::Point3d> model_points, std::vector<cv::Point2d> image_points)
{
    // Read input image
    cv::Mat im = cv::imread("test.jpg");

    // Camera internals
    double focal_length = im.cols; // Approximate focal length
    Point2d center = cv::Point2d(im.cols/2, im.rows/2);
    cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
    cv::Mat dist_coeffs = cv::Mat::zeros(4, 1, cv::DataType<double>::type); // Assuming no lens distortion

    //cout << "Camera Matrix " << endl << camera_matrix << endl ;

    // Output rotation and translation
    cv::Mat rotation_vector; // Rotation in axis-angle form
    cv::Mat translation_vector;

    // Solve for pose
    cv::solvePnP(model_points, image_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, false, SOLVEPNP_SQPNP);

    double Rtmp[9] = {0.0};
    Mat R(3, 3, DataType<double>::type, Rtmp);
    Rodrigues(rotation_vector, R); //пересчет углов поворота в матрицу поворота

    Mat measured_eulers(3, 1, CV_64F);
    measured_eulers = rot2euler(R); // преобразование в углы Эйлера

    auto euler_degrees = measured_eulers * 180 / CV_PI;

    cout << "Rotation Vector in Euler degrees: " << endl << euler_degrees << endl;
    cout << "Translation Vector: " << endl << translation_vector << endl;

    return 0;
}



int main(int argc, char** argv)
{
//    if( argc != 2)
//    {
//        cout <<" Usage: display_image ImageToLoadAndDisplay" << endl;
//        return -1;
//    }

    // 3D model points.
    std::vector<cv::Point3d> model_points;
    model_points.push_back(cv::Point3d(0.0f, 0.0f, 0.0f));               // Upper left point
    model_points.push_back(cv::Point3d(0.0f, -30.0f, 0.0f));             // Lower left point
    model_points.push_back(cv::Point3d(-20.0f, 0.0f, 0.0f));             // Upper right point

    std::vector<cv::Point2d> image_points = segmentize();
    findpose(model_points, image_points);

    return 0;
}
