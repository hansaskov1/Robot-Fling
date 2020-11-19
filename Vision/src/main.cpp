#include <stdio.h>
#include <chrono>

#include "calibration.h"
#include "objectdetection.h"

using namespace cv;

int main()
{
    Calibration c;
    ObjectDetection o;
    c.calibrate2();
    c.connectToCam();
    cv::Mat image;

    cv::Mat worldCalImg[4];
    for(int i = 0; i < 4; i++){
        cv::String path = "";
        path = "../Images/BallWorldCordsROI/img" + std::to_string(i) + ".png";
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
            //cv::imshow("image",image);
        }else if(keyPressed == 'f'){
            image = c.getImage();
            cv::Point ballPos = o.hcBallCenterPosition(image,10,30);
            cv::Mat camToBall = c.calcTransMatCamToWorld(ballPos);
            std::cout << camToBall << std::endl;
            //cv::imshow("image",image);
        }else if(keyPressed == 'p'){
            image = c.getImage();
            cv::String path = "";
            path = "../Images/BallWorldCordsROI/img" + std::to_string(imgNr) + ".png";
            cv::imwrite(path, image);
            imgNr++;
        }else if(keyPressed == 'r'){
            image = c.getRawImage();
            cv::String path = "";
            path = "../Images/CalibrationImages/img" + std::to_string(rImgNr) + ".png";
            cv::imwrite(path, image);
            rImgNr++;
        }else if(keyPressed == 'c'){
            c.calibrate();
            c.createTranformMatrix(worldCalImg);
        }else{
            std::vector<cv::String> FileNames;
            cv::glob("../Images/CalibrationImages/img*.png", FileNames, false);
            c.setFileNames(FileNames);
            image = c.getImage();
            /*std::string r;
            uchar depth = image.type() & CV_MAT_DEPTH_MASK;
            uchar chans = 1 + (image.type() >> CV_CN_SHIFT);
            switch (depth) {
            case CV_8U:
                r = "8U";
                break;
            case CV_8S:
                r = "8S";
                break;
            case CV_16U:
                r = "16U";
                break;
            case CV_16S:
                r = "16S";
                break;
            case CV_32S:
                r = "32S";
                break;
            case CV_32F:
                r = "32F";
                break;
            case CV_64F:
                r = "64F";
                break;
            default:
                r = "User";
                break;
            }
            r += "C";
            r+= (chans+'0');
            std::cout << r << std::endl;
*/
            cv::imshow("video",image);
            //std::cout << "Im running!" << std::endl;
        }
    }

    return 0;
}
