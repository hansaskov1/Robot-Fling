#ifndef SHOWVIDEO_H
#define SHOWVIDEO_H

#include <QThread>
#include <QThreadPool>
#include <QLabel>
#include "mainwindow.h"

class showVideo : public QRunnable
{
public:
    showVideo (QLabel *label) {
        mLabel = label;
    }

    void run() {
        c.connectToCam();
        while(true) {
                image = c.getImage();
                const uchar *qImageBuffer1 = (const uchar*)image.data;
                QImage img1(qImageBuffer1, image.cols, image.rows, image.step, QImage::Format_RGB888);
                pixmap = QPixmap::fromImage(img1.rgbSwapped());
                mLabel->setPixmap(pixmap);
                mLabel->setScaledContents(true);
            }
            /*
            image = c.getImage();
            const uchar *qImageBuffer = (const uchar*)image.data;
            QImage img(qImageBuffer, image.cols, image.rows, image.step, QImage::Format_RGB888);
            pixmap = QPixmap::fromImage(img.rgbSwapped());
            mLabel->setPixmap(pixmap);
            mLabel->setScaledContents(true);
            */
       }

private:
    QLabel *mLabel;
    Calibration c;
    cv::Mat image;
    QPixmap pixmap;
};

#endif // SHOWVIDEO_H
