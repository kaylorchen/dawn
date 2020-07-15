#include <stdio.h>
#include <opencv2/opencv.hpp>
const double PI = 3.1415926535897932;

void printDegrees(std::vector<cv::Vec4i> lines)
{
    std::vector<cv::Vec4i>::const_iterator it = lines.begin();
    while (it != lines.end())
    {
        double tmp;
        if ((*it)[0] - (*it)[1] != 0)
        {
            tmp = atan((double)((*it)[1] - (*it)[3]) / ((*it)[0] - (*it)[2]));
            // printf("%f\n",(double)((*it)[1] - (*it)[3]) / ((*it)[0] - (*it)[2]));
        }
        else
        {
            tmp = PI / 2;
        }

        printf("%f, %f degrees\n", tmp, tmp * 180 / PI);
        it++;
    }
}

class LineFinder
{
private:
    cv::Mat img;
    //包含被检测直线的端点的向量
    std::vector<cv::Vec4i> lines;

    //累加器分辨率参数
    double deltaRho;
    double deltaTheta;

    //确认直线之前必须受到的最小投票数
    int minVote;

    //直线的最小长度
    double minLength;
    //直线上允许的最大空隙
    double maxGap;

public:
    LineFinder() : deltaRho(1), deltaTheta(PI / 180), minVote(10), minLength(0.0), maxGap(0.0) {}
    void setAccResolution(double dRho, double dTheta)
    {
        deltaRho = dRho;
        deltaTheta = dTheta;
    }
    void setminVote(int minv)
    {
        minVote = minv;
    }
    void setLengthAndGap(double length, double gap)
    {
        minLength = length;
        maxGap = gap;
    }
    std::vector<cv::Vec4i> findLines(cv::Mat &binary)
    {
        lines.clear();
        cv::HoughLinesP(binary, lines, deltaRho, deltaTheta, minVote, minLength, maxGap);
        return lines;
    }
    void drawDetectedLines(cv::Mat &image, cv::Scalar color = cv::Scalar(255, 255, 255))
    {
        std::vector<cv::Vec4i>::const_iterator it = lines.begin();
        while (it != lines.end())
        {
            // printf("A(%d, %d), B(%d, %d)\n", (*it)[0], (*it)[1], (*it)[2], (*it)[3]);
            printf("%d %d %d %d\n", (*it)[0], (*it)[1], (*it)[2], (*it)[3]);
            cv::Point pt1((*it)[0], (*it)[1]);
            cv::Point pt2((*it)[2], (*it)[3]);
            cv::line(image, pt1, pt2, color);
            it++;
        }
    }
};

cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 259.245985399985, 0, 333.918731202799, 0, 259.751981070593, 272.314983608258, 0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(1, 4) << -0.3157, 0.0802, 0, 0);
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
    cv::Canny(undistort_img, contours, lowThreshold, lowThreshold * ratio, kernel_size);
    cv::imshow("contours", contours);
    int w = 100;

    cv::Mat mask;
    mask = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> edge;
    std::vector<cv::Point> pts;
    int top_w = 80;
    int bot_w = 400;
    pts.push_back(cv::Point(320 - top_w / 2, 100));
    pts.push_back(cv::Point(320 + top_w / 2, 100));
    pts.push_back(cv::Point(320 + bot_w / 2, 640));
    pts.push_back(cv::Point(320 - bot_w / 2, 640));
    edge.push_back(pts);
    cv::drawContours(mask, edge, 0, cv::Scalar::all(255), -1);
    contours.copyTo(dst, mask);
    // undistort_img.copyTo(dst, mask);
    cv::imshow("final", dst);
    LineFinder cFinder;
    cFinder.setLengthAndGap(80, 20);
    cFinder.setminVote(80);
    std::vector<cv::Vec4i> lines = cFinder.findLines(dst);
    printDegrees(lines);
    cFinder.drawDetectedLines(src);
    cv::imshow("detected result", src);
    cv::Mat tmp = cv::Mat::zeros(src.size(), CV_8UC1);
    cFinder.drawDetectedLines(tmp);
    cv::imshow("tmp", tmp);
    cv::waitKey();
    return 0;
}