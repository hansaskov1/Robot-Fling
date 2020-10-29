#include "objectdetection.h"

ObjectDetection::ObjectDetection()
{

}

cv::Point2f ObjectDetection::hcBallCenterPosition(cv::Mat undisImage, int innerCircleSize, int outCircleSize)
{
    cv::Mat grayImage;
    cv::cvtColor(undisImage,grayImage,cv::COLOR_BGR2GRAY);
    cv::medianBlur(grayImage,grayImage,5);
    std::vector<cv::Vec3f> circle;
    cv::HoughCircles(grayImage,circle,cv::HOUGH_GRADIENT,1,grayImage.rows/16,100,30,innerCircleSize,outCircleSize);
    cv::Point centerCircle;
    for(int i = 0; i <circle.size(); i++){
        cv::Vec3i c = circle[i];
        cv::Point center = cv::Point(c[i] , c[1]);
        centerCircle = cv::Point(c[i] , c[1]);
        //Circle center
        cv::circle(grayImage,center,1,cv::Scalar(0,100,100),3,cv::LINE_AA);
        // circle outline
        int radius = c[2];
        cv::circle(grayImage,center,radius,cv::Scalar(255,0,255),3,cv::LINE_AA);
    }
    cv::imshow("ball",grayImage);
    //cv::waitKey(0);
    return centerCircle;
}

cv::Point2f ObjectDetection::colorMorphLineByLine(cv::Mat image){
    //convert to HSV
    cv::Mat ballHSV;
    cv::cvtColor(image,ballHSV,cv::COLOR_BGR2HSV);

    //find Orange
    cv::Mat orange, opened;
    cv::inRange(ballHSV,cv::Scalar(10,200,100),cv::Scalar(25,255,255),orange);

    //create 5x5 element and morph open
    cv::Mat five_by_five_element( 5, 5, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(orange, opened, cv::MORPH_OPEN,five_by_five_element);

    //morph closed
    cv::Mat closed;
    cv::morphologyEx(opened, closed, cv::MORPH_CLOSE,five_by_five_element);
    cv::imwrite("../Images/BallWorldCords/imgtemp.png",closed);
    cv::imshow("closed",closed);
    //cv::waitKey();

    int xStart = 0,xEnd = 0;
    int yStart = 0,yEnd = 0;
    bool wol = false;

    int count = 0;

    //std::cout << "Find y on: " << closed.cols << ", " << closed.rows << std::endl;
    //find yStart & yEnd
    for (int y = 1; y < closed.rows; y++){
        for (int x = 1; x < closed.cols; x++){
            if((int)closed.at<uchar>(y,x) != 0){
                //std::cout << x << ", " << y << ": " << (int)closed.at<uchar>(y,x) << std::endl;
                count++;
            }
            if(yStart == 0 && (int)closed.at<uchar>(y,x) != 0){
                yStart = y;
            }
            if(!wol && (int)closed.at<uchar>(y,x) !=0){
                wol = true;
            }
            if(yStart != 0 && yEnd == 0 && !wol && x == closed.cols - 1){
                yEnd = y - 1;
            }
            if(wol && x == closed.cols - 1){
                wol = false;
            }
        }
    }
    //std::cout << count << " y: " << yStart << ", " << yEnd << std::endl;

    count = 0;
    //std::cout << "Find x on: " << closed.cols << ", " << closed.rows << std::endl;
    //find xStart & xEnd
    for (int x = 1; x < closed.cols; x++){
        for (int y = 1; y < closed.rows; y++){
            if((int)closed.at<uchar>(y,x) != 0){
                //std::cout << x << ", " << y << ": " << (int)closed.at<uchar>(y,x) << std::endl;
                count++;
            }
            if(xStart == 0 && (int)closed.at<uchar>(y,x) != 0){
                xStart = x;
            }
            if(!wol && (int)closed.at<uchar>(y,x) !=0){
                wol = true;
            }
            if(xStart != 0 && xEnd == 0 && !wol && y == closed.rows - 1){
                xEnd = x - 1;
            }
            if(wol && y == closed.rows - 1){
                wol = false;
            }
        }
    }
    //std::cout << count << " x: " << xStart << ", " << xEnd << std::endl;

    std::cout << "Start: " << xStart << ", " << yStart << " \nend: " << xEnd << ", " << yEnd << std::endl;
    int xBall, yBall;
    xBall = xStart + ((xEnd - xStart) / 2);
    yBall = yStart + ((yEnd - yStart) / 2);
    cv::Point2f ballPos;
    ballPos = cv::Point(xBall,yBall);
    std::cout << "Ball center cords: " << ballPos  << std::endl;
    return ballPos;
}
