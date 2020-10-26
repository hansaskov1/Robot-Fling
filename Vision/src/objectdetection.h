#ifndef OBJECTDETECTION_H
#define OBJECTDETECTION_H

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

class ObjectDetection
{
public:
    ObjectDetection();

    cv::Point2f hcBallCenterPosition(cv::Mat undisImage,int innerCircleSize, int outCircleSize);

    cv::Point2f colorMorphLineByLine(cv::Mat image);
};

#endif // OBJECTDETECTION_H
