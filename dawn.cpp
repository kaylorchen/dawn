#include <stdio.h>
#include <opencv2/opencv.hpp>

cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) << 259.245985399985, 0, 333.918731202799, 0, 259.751981070593, 272.314983608258, 0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(1,4) << -0.3157, 0.0802, 0, 0);
cv::Mat src, dst;
int lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;

int main(int argc, char **argv)
{
    /// 装载图像
    src = cv::imread(argv[1]);

    if (!src.data)
    {
        return -1;
    }

    cv::imshow("original image", src);
    cv::Mat undistort_img;
    cv::undistort(src, undistort_img, cameraMatrix, distCoeffs);
    cv::imshow("undistort", undistort_img);
    cv::Mat contours;
	cv::Canny(undistort_img, contours, lowThreshold, lowThreshold*ratio, kernel_size);
    cv::imshow("contours",contours);
    int w = 100;
    cv::Rect r1(320 - w/2, 240, w, 240);
    cv::Mat mask;
    mask = cv::Mat::zeros(src.size(), CV_8UC1);
	mask(r1).setTo(255);
    contours.copyTo(dst, mask);
    cv::imshow("final", dst);
    cv::waitKey();

    return 0;
}