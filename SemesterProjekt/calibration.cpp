#include "calibration.h"
#include "objectdetection.h"


Calibration::Calibration()
{
    cv::glob("../Images/CalibrationImages/img*.png", mFileNames, false);
}

Calibration::~Calibration()
{
    delete &mtx;
}

bool Calibration::createTranformMatrix(cv::Mat undisWorldCalImg[4])
{
    //Define mat size and type
    cv::Mat tMat = cv::Mat::zeros(undisWorldCalImg[0].rows, undisWorldCalImg[0].cols,undisWorldCalImg[0].type());

    //pixel coordiants
    cv::Point2f pixelCords[4];

    ObjectDetection o;

    pixelCords[0] = o.hcBallCenterPosition(undisWorldCalImg[0],10,30);
    pixelCords[1] = o.hcBallCenterPosition(undisWorldCalImg[1],10,30);
    pixelCords[2] = o.hcBallCenterPosition(undisWorldCalImg[2],10,30);
    pixelCords[3] = o.hcBallCenterPosition(undisWorldCalImg[3],10,30);

    //World coordinats
    cv::Point2f worldCords[4];

    worldCords[0] = cv::Point2f(0,0);
    worldCords[1] = cv::Point2f(0,5*15);
    worldCords[2] = cv::Point2f(5*15,5*15);
    worldCords[3] = cv::Point2f(5*15,0);

    //get perspektive transformation Matrix
    tMat = cv::getPerspectiveTransform(pixelCords,worldCords);
    pixelToWorld = tMat;
}

cv::Mat Calibration::calcTransMatCamToWorld(cv::Point2f targetPixelPos)
{
    std::vector<cv::Point2f> pixelCoords = {targetPixelPos};
    std::vector<cv::Point2f> worldCoordsCalc;

    cv::perspectiveTransform(pixelCoords, worldCoordsCalc, pixelToWorld);

    cv::Mat camToBall(cv::Matx44f::eye());

    camToBall.at<float>(0,3) = worldCoordsCalc[0].x;
    camToBall.at<float>(1,3) = worldCoordsCalc[0].y;
    camToBall.at<float>(2,3) = 2;//z value (Ball radius)

    transMatCamToBall = camToBall;
    return camToBall;
}

bool Calibration::connectToCam()
{
    auto startTid = std::chrono::high_resolution_clock::now();
    auto nuTid = std::chrono::high_resolution_clock::now();
    grapThread = new std::thread (&Calibration::grapPictures, this);

    while(!camRunning){
        //std::cout << "fisk" << std::endl;
        nuTid = std::chrono::high_resolution_clock::now();
        if(std::chrono::duration_cast<std::chrono::microseconds>(nuTid - startTid) > std::chrono::milliseconds(10000)){
            mCvImage = cv::imread("../Images/CalibrationImages/img0.png", cv::IMREAD_COLOR);
            break;
        }
    }
}

bool Calibration::calibrate()
{
    std::vector<cv::Point3f> objp;
    for(int i = 0 ; i<mPatternSize.height-1 ; i++)
    {
        for(int j = 0 ; j<mPatternSize.width-1 ; j++)
           objp.push_back(cv::Point3f(j,i,0));
    }

    std::vector<std::vector<cv::Point2f>> q(mFileNames.size());
    std::vector<std::vector<cv::Point3f> > objpoints;
    cv::Mat img;
    // Detect feature points
    std::size_t i = 0;
    for (auto const &f : mFileNames) {
      //std::cout << std::string(f) << std::endl;
      img = cv::imread(f,CV_8UC3);

      bool patternFound;

      // 1. Read in the image an call cv::findChessboardCorners()
      patternFound = cv::findChessboardCorners(img,cv::Size(mPatternSize.width-1,mPatternSize.height-1), q[i],cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE);
      // 2. Use cv::cornerSubPix() to refine the found corner detections
      if(patternFound){
          cv::cornerSubPix(img, q[i],mPatternSize,cv::Size(-1,-1),cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
      }
      objpoints.push_back(objp);
      i++;
    }

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440, 1080);

    std::cout << img.rows << " " << img.cols << std::endl;
    //cv::TermCriteria crit = cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001);


    std::cout << "Calibrating..." << std::endl;
    float error = cv::calibrateCamera(objpoints ,q ,frameSize , K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics,
                                      perViewErrors,flags,cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 5,
                                                                          DBL_EPSILON));
    // 4. Call "float error = cv::calibrateCamera()" with the input coordinates
    // and output parameters as declared above...

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mMapX, mMapY);


}

bool Calibration::calibrate2()
{
//    std::vector<cv::String> fileNames;
//    cv::glob("../calibrationImages/Image*.png", fileNames, false);
    cv::Size patternSize(10 - 1, 7 - 1);
    std::vector<std::vector<cv::Point2f>> q(mFileNames.size());

    // Detect feature points

    std::size_t i = 0;
    for (auto const &f : mFileNames) {
      std::cout << std::string(f) << std::endl;

      cv::Mat img = cv::imread(f, cv::IMREAD_COLOR);
      cv::Mat imgGray;
      cv::cvtColor(img, imgGray, CV_BGR2GRAY);
      bool success = cv::findChessboardCorners(imgGray, patternSize, q[i],
                                               cv::CALIB_CB_ADAPTIVE_THRESH +
                                                   cv::CALIB_CB_NORMALIZE_IMAGE);

      if (!success) {
        std::cout << "Could not find chessboard!" << std::endl;
        continue;
      }

      cv::cornerSubPix(imgGray, q[i], cv::Size(5, 5), cv::Size(2, 2),
                       cv::TermCriteria());

      // Display
//      cv::drawChessboardCorners(img, patternSize, q[i], success);
//      cv::imshow("chessboard detection", img);
//      cv::waitKey(0);

      i++;
    }

    std::vector<cv::Point3f> Qview;
    for (int i = 0; i < patternSize.height; i++) {
      for (int j = 0; j < patternSize.width; j++) {
        Qview.push_back(cv::Point3f(i * 0.015f, j * 0.015f, 0.0f));
      }
    }
    std::vector<std::vector<cv::Point3f>> Q;
    for (size_t i = 0; i < q.size(); i++)
      Q.push_back(Qview);

    cv::Matx33f K(cv::Matx33f::eye());  // intrinsic camera matrix
    cv::Vec<float, 5> k(0, 0, 0, 0, 0); // distortion coefficients

    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<double> stdIntrinsics, stdExtrinsics, perViewErrors;
    int flags = cv::CALIB_FIX_ASPECT_RATIO + cv::CALIB_FIX_K3 +
                cv::CALIB_ZERO_TANGENT_DIST + cv::CALIB_FIX_PRINCIPAL_POINT;
    cv::Size frameSize(1440, 1080);

    std::cout << "Calibrating..." << std::endl;
    double error = cv::calibrateCamera(
        Q, q, frameSize, K, k, rvecs, tvecs, stdIntrinsics, stdExtrinsics,
        perViewErrors, flags,
        cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 5,
                         DBL_EPSILON));

    std::cout << "Reprojection error = " << error << "\nK =\n"
              << K << "\nk=\n"
              << k << std::endl;

    // Precompute lens correction interpolation
    //cv::Mat mapX, mapY;
    cv::initUndistortRectifyMap(K, k, cv::Matx33f::eye(), K, frameSize, CV_32FC1,
                                mMapX, mMapY);
}

cv::Mat Calibration::getImage()
{
    cv::Mat imageIn, imageOut;
    //std::cout << "mtx locking for out.." << std::endl;
    while(!mtx.try_lock());
    //td::cout << "mtx locked for out" << std::endl;
    imageIn = mCvImage;
    //std::cout << "mtx unlocked for out" << std::endl;
    cv::remap(imageIn,imageOut,mMapX,mMapY,1,1);
    imageOut.adjustROI(-190,-200,-430,-300);
    mtx.unlock();
    return imageOut;
}

cv::Mat Calibration::getRawImage()
{
    cv::Mat imageIn;
    while(!mtx.try_lock());
    imageIn = mCvImage;
    mtx.unlock();
    //imageIn.adjustROI(-180,-190,-420,-290);

    return imageIn;
}

void Calibration::grapPictures()
{
    int myExposure = 30000;

    // Automagically call PylonInitialize and PylonTerminate to ensure the pylon runtime system
    // is initialized during the lifetime of this object.
    Pylon::PylonAutoInitTerm autoInitTerm;

    try
    {
        // Create an instant camera object with the camera device found first.
        Pylon::CInstantCamera camera( Pylon::CTlFactory::GetInstance().CreateFirstDevice());

        // Get a camera nodemap in order to access camera parameters.
        GenApi::INodeMap& nodemap= camera.GetNodeMap();

        // Open the camera before accessing any parameters.
        camera.Open();
        // Create pointers to access the camera Width and Height parameters.
        GenApi::CIntegerPtr width= nodemap.GetNode("Width");
        GenApi::CIntegerPtr height= nodemap.GetNode("Height");

        // The parameter MaxNumBuffer can be used to control the count of buffers
        // allocated for grabbing. The default value of this parameter is 10.
        // camera.MaxNumBuffer = 5;

        // Create a pylon ImageFormatConverter object.
        Pylon::CImageFormatConverter formatConverter;
        // Specify the output pixel format.
        formatConverter.OutputPixelFormat= Pylon::PixelType_BGR8packed;
        // Create a PylonImage that will be used to create OpenCV images later.
        Pylon::CPylonImage pylonImage;


        // Set exposure to manual
        GenApi::CEnumerationPtr exposureAuto( nodemap.GetNode( "ExposureAuto"));
        if ( GenApi::IsWritable( exposureAuto)){
            exposureAuto->FromString("Off");
            std::cout << "Exposure auto disabled." << std::endl;
        }

        // Set custom exposure
        GenApi::CFloatPtr exposureTime = nodemap.GetNode("ExposureTime");
        std::cout << "Old exposure: " << exposureTime->GetValue() << std::endl;
        if(exposureTime.IsValid()) {
            if(myExposure >= exposureTime->GetMin() && myExposure <= exposureTime->GetMax()) {
                exposureTime->SetValue(myExposure);
            }else {
                exposureTime->SetValue(exposureTime->GetMin());
                std::cout << ">> Exposure has been set with the minimum available value." << std::endl;
                std::cout << ">> The available exposure range is [" << exposureTime->GetMin() << " - " << exposureTime->GetMax() << "] (us)" << std::endl;
            }
        }else {

            std::cout << ">> Failed to set exposure value." << std::endl;

        }
        std::cout << "New exposure: " << exposureTime->GetValue() << std::endl;

        // Start the grabbing of c_countOfImagesToGrab images.
        // The camera device is parameterized with a default configuration which
        // sets up free-running continuous acquisition.
        camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);

        // This smart pointer will receive the grab result data.
        Pylon::CGrabResultPtr ptrGrabResult;

        // image grabbing loop
        int frame = 1;
        std::vector<int> compression_params;
            compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
            compression_params.push_back(9);
        while ( camera.IsGrabbing())
        {
            // Wait for an image and then retrieve it. A timeout of 5000 ms is used.
            camera.RetrieveResult( 5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);

            // Image grabbed successfully?
            if (ptrGrabResult->GrabSucceeded())
            {
                // Access the image data.
                //cout << "SizeX: " << ptrGrabResult->GetWidth() << endl;
                //cout << "SizeY: " << ptrGrabResult->GetHeight() << endl;

                // Convert the grabbed buffer to a pylon image.
                formatConverter.Convert(pylonImage, ptrGrabResult);

                // Create an OpenCV image from a pylon image.

                //std::cout << "mtx locking for in.." << std::endl;
                while (!mtx.try_lock()){

                }
                //std::cout << "mtx locked for input" << std::endl;
                if(!camRunning)
                    camRunning = true;
                mCvImage = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());
                mtx.unlock();
                //std::cout << "mtx unlocked for input" << std::endl;


                //////////////////////////////////////////////////////
                //////////// Here your code begins ///////////////////
                //////////////////////////////////////////////////////


//                // Create an OpenCV display window.
//                cv::namedWindow( "myWindow", CV_WINDOW_NORMAL); // other options: CV_AUTOSIZE, CV_FREERATIO

//                // Display the current image in the OpenCV display window.
//                cv::imshow( "myWindow", openCvImage);

//                // Detect key press and save image if 'p' i pressed
//                int keyPressed = cv::waitKey(1);
//                if(keyPressed == 'p'){
//                    std::cout << "Pic nr: " << n << std::endl;
//                    std::string path = "/home/suspend/workspace/pylon_openCV/src/ball/img" + std::to_string(n)+ ".png";
//                    cv::imwrite(path, openCvImage,compression_params);
//                    std::cout << "Pic nr:" << n << " saved" << std::endl;
//                    n++;
//                }

                // Detect key press and quit if 'q' is pressed
                if(!mRun){ //quit
                    std::cout << "Shutting down camera..." << std::endl;
                    camera.Close();
                    std::cout << "Camera successfully closed." << std::endl;
                    break;
                }

                ////////////////////////////////////////////////////
                //////////// Here your code ends ///////////////////
                ////////////////////////////////////////////////////




                frame++;

            }
            else
            {
                std::cout << "Error: " << ptrGrabResult->GetErrorCode() << " " << ptrGrabResult->GetErrorDescription() << std::endl;
            }
        }

    }
    catch (GenICam::GenericException &e)
    {
        // Error handling.
        camRunning = false;
        std::cerr << "An exception occurred." << std::endl
        << e.GetDescription() << std::endl;
    }

}

void Calibration::setFileNames(const std::vector<cv::String> &fileNames)
{
    mFileNames = fileNames;
}
