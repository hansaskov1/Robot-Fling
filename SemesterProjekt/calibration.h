#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <pylon/PylonIncludes.h>
#include <vector>
#include <string>
#include <mutex>
#include <chrono>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <thread>

class Calibration
{
public:
    Calibration();

    ~Calibration();

    //cv::Point2f hcBallCenterPosition(cv::Mat image,int innerCircleSize, int outCircleSize);

    void init(int);

    bool createTranformMatrix(cv::Mat undisWorldCalImg[4]);

    cv::Mat calcTransMatCamToWorld(cv::Point2f targetPixelPos);

    bool connectToCam();

    bool calibrate();

    bool calibrate2();

    cv::Mat getImage();

    cv::Mat getRawImage();

    bool mRun= true;
    void setFileNames(const std::vector<cv::String> &fileNames);

private:
    void grapPictures();


    bool isCalibrated;
    bool camRunning = false;

    std::mutex mtx;

    std::thread *grapThread;

    std::vector<cv::String> mFileNames;

    cv::Size mPatternSize = cv::Size(10, 7);

    cv::Mat pixelToWorld;
    cv::Mat transMatCamToBall;
    cv::Mat mCvImage;
    cv::Mat mMapX, mMapY;

};

#endif // CALIBRATION_H
