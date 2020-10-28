#include <stdio.h>
#include <chrono>

#include "calibration.h"
#include "objectdetection.h"

using namespace cv;

int main()
{
    Calibration c;
    ObjectDetection o;
    c.calibrate();
    c.connectToCam();
    cv::Mat image;

    cv::Mat worldCalImg[4];
    for(int i = 0; i < 4; i++){
        cv::String path = "";
        path = "../Images/BallWorldCords/img" + std::to_string(i) + ".png";
        worldCalImg[i] = cv::imread(path, cv::IMREAD_COLOR);
    }
    c.createTranformMatrix(worldCalImg);

    int imgNr = 0,rImgNr = 0;
    while(c.mRun){
        int keyPressed = cv::waitKey(1);
        if(keyPressed == 'q'){
            c.mRun = false;
        }else if(keyPressed == 'o'){
            image = c.getImage();
            cv::Point ballPos =o.colorMorphLineByLine(image);
            cv::Mat camToBall = c.calcTransMatCamToWorld(ballPos);
            std::cout << camToBall << std::endl;
            cv::imshow("image",image);
        }else if(keyPressed == 'f'){
            image = c.getImage();
            cv::Point ballPos = o.hcBallCenterPosition(image,10,30);
            cv::Mat camToBall = c.calcTransMatCamToWorld(ballPos);
            std::cout << camToBall << std::endl;
            cv::imshow("image",image);
        }else if(keyPressed == 'p'){
            image = c.getImage();
            cv::String path = "";
            path = "../Images/BallWorldCords/img" + std::to_string(imgNr) + ".png";
            cv::imwrite(path, image);
            imgNr++;
        }else if(keyPressed == 'r'){
            image = c.getRawImage();
            cv::String path = "";
            path = "../Images/CalibrationImagesROI/img" + std::to_string(rImgNr) + ".png";
            cv::imwrite(path, image);
            rImgNr++;
        }else if(keyPressed == 'c'){
            c.calibrate();
            c.createTranformMatrix(worldCalImg);
        }else{
            image = c.getImage();
            cv::imshow("video",image);
            //std::cout << "Im running!" << std::endl;
        }
    }

    return 0;
}
