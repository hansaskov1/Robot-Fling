#ifndef SHOWVIDEO_H
#define SHOWVIDEO_H

#include <QThread>
#include <QThreadPool>
#include <QLabel>
#include <thread>
#include <chrono>
#include "mainwindow.h"

class showVideo : public QRunnable
{
public:
    showVideo (QLabel *label, Calibration *value) {
        mLabel = label;
        c = value;
        mLabel->setScaledContents(true);
    }

    void run() {
        int i = 0;
        while(c->mRun) {
            image = c->getImage();
            mLabel->setPixmap(QPixmap::fromImage(QImage((const uchar*)image.data, image.cols, image.rows, image.step, QImage::Format_RGB888).rgbSwapped()));
            std::this_thread::sleep_for (std::chrono::milliseconds(100));
        }
    }

private:
    QLabel *mLabel;
    Calibration *c;
    cv::Mat image;
};

#endif // SHOWVIDEO_H
