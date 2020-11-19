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
        while(true) {
            image = c.getImage();
            const uchar *qImageBuffer = (const uchar*)image.data;
            QImage img(qImageBuffer, image.cols, image.rows, image.step, QImage::Format_RGB888);
            pixmap = QPixmap::fromImage(img.rgbSwapped());
            mLabel->setPixmap(pixmap);
            mLabel->setScaledContents(true);
        }
    }
private:
    QLabel *mLabel;
    Calibration c;
    cv::Mat image;
    QPixmap pixmap;
};

#endif // SHOWVIDEO_H
