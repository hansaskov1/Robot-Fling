/********************************************************************************
** Form generated from reading UI file 'MainWindow.ui'
**
** Created by: Qt User Interface Compiler version 5.15.1
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QSpacerItem>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_MainWindow
{
public:
    QWidget *centralwidget;
    QVBoxLayout *verticalLayout_2;
    QWidget *widget;
    QHBoxLayout *horizontalLayout;
    QWidget *widget_4;
    QGridLayout *gridLayout;
    QSpacerItem *verticalSpacer;
    QLineEdit *lineEdit_8;
    QLineEdit *lineEdit_4;
    QLabel *label;
    QPushButton *bSend;
    QLabel *label_10;
    QLineEdit *lineEdit_3;
    QLabel *label_2;
    QSpacerItem *horizontalSpacer_2;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout;
    QLabel *label_7;
    QPushButton *bOpenGrip;
    QPushButton *bCloseGrip;
    QPushButton *bCalibrate;
    QSpacerItem *verticalSpacer_2;
    QWidget *widget_3;
    QGridLayout *gridLayout_3;
    QPushButton *bDisconnect;
    QPushButton *bSaveConnect;
    QWidget *widget_5;
    QGridLayout *gridLayout_2;
    QSpacerItem *horizontalSpacer;
    QLabel *label_5;
    QLineEdit *lineEdit_2;
    QLineEdit *lineEdit;
    QLabel *label_6;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QString::fromUtf8("MainWindow"));
        MainWindow->resize(582, 433);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        verticalLayout_2 = new QVBoxLayout(centralwidget);
        verticalLayout_2->setObjectName(QString::fromUtf8("verticalLayout_2"));
        widget = new QWidget(centralwidget);
        widget->setObjectName(QString::fromUtf8("widget"));
        horizontalLayout = new QHBoxLayout(widget);
        horizontalLayout->setObjectName(QString::fromUtf8("horizontalLayout"));
        widget_4 = new QWidget(widget);
        widget_4->setObjectName(QString::fromUtf8("widget_4"));
        gridLayout = new QGridLayout(widget_4);
        gridLayout->setObjectName(QString::fromUtf8("gridLayout"));
        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        gridLayout->addItem(verticalSpacer, 7, 1, 1, 1);

        lineEdit_8 = new QLineEdit(widget_4);
        lineEdit_8->setObjectName(QString::fromUtf8("lineEdit_8"));

        gridLayout->addWidget(lineEdit_8, 2, 1, 1, 1);

        lineEdit_4 = new QLineEdit(widget_4);
        lineEdit_4->setObjectName(QString::fromUtf8("lineEdit_4"));

        gridLayout->addWidget(lineEdit_4, 1, 1, 1, 1);

        label = new QLabel(widget_4);
        label->setObjectName(QString::fromUtf8("label"));

        gridLayout->addWidget(label, 1, 0, 1, 1);

        bSend = new QPushButton(widget_4);
        bSend->setObjectName(QString::fromUtf8("bSend"));

        gridLayout->addWidget(bSend, 3, 1, 1, 1);

        label_10 = new QLabel(widget_4);
        label_10->setObjectName(QString::fromUtf8("label_10"));

        gridLayout->addWidget(label_10, 2, 0, 1, 1);

        lineEdit_3 = new QLineEdit(widget_4);
        lineEdit_3->setObjectName(QString::fromUtf8("lineEdit_3"));

        gridLayout->addWidget(lineEdit_3, 0, 1, 1, 1);

        label_2 = new QLabel(widget_4);
        label_2->setObjectName(QString::fromUtf8("label_2"));

        gridLayout->addWidget(label_2, 0, 0, 1, 1);


        horizontalLayout->addWidget(widget_4);

        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        horizontalLayout->addItem(horizontalSpacer_2);

        widget_2 = new QWidget(widget);
        widget_2->setObjectName(QString::fromUtf8("widget_2"));
        verticalLayout = new QVBoxLayout(widget_2);
        verticalLayout->setObjectName(QString::fromUtf8("verticalLayout"));
        label_7 = new QLabel(widget_2);
        label_7->setObjectName(QString::fromUtf8("label_7"));

        verticalLayout->addWidget(label_7);

        bOpenGrip = new QPushButton(widget_2);
        bOpenGrip->setObjectName(QString::fromUtf8("bOpenGrip"));

        verticalLayout->addWidget(bOpenGrip);

        bCloseGrip = new QPushButton(widget_2);
        bCloseGrip->setObjectName(QString::fromUtf8("bCloseGrip"));

        verticalLayout->addWidget(bCloseGrip);

        bCalibrate = new QPushButton(widget_2);
        bCalibrate->setObjectName(QString::fromUtf8("bCalibrate"));

        verticalLayout->addWidget(bCalibrate);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);


        horizontalLayout->addWidget(widget_2);


        verticalLayout_2->addWidget(widget);

        widget_3 = new QWidget(centralwidget);
        widget_3->setObjectName(QString::fromUtf8("widget_3"));
        gridLayout_3 = new QGridLayout(widget_3);
        gridLayout_3->setObjectName(QString::fromUtf8("gridLayout_3"));
        gridLayout_3->setContentsMargins(-1, 9, -1, 9);
        bDisconnect = new QPushButton(widget_3);
        bDisconnect->setObjectName(QString::fromUtf8("bDisconnect"));

        gridLayout_3->addWidget(bDisconnect, 1, 2, 1, 1);

        bSaveConnect = new QPushButton(widget_3);
        bSaveConnect->setObjectName(QString::fromUtf8("bSaveConnect"));

        gridLayout_3->addWidget(bSaveConnect, 0, 2, 1, 1);

        widget_5 = new QWidget(widget_3);
        widget_5->setObjectName(QString::fromUtf8("widget_5"));
        gridLayout_2 = new QGridLayout(widget_5);
        gridLayout_2->setObjectName(QString::fromUtf8("gridLayout_2"));
        horizontalSpacer = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer, 2, 2, 1, 1);

        label_5 = new QLabel(widget_5);
        label_5->setObjectName(QString::fromUtf8("label_5"));

        gridLayout_2->addWidget(label_5, 2, 0, 1, 1);

        lineEdit_2 = new QLineEdit(widget_5);
        lineEdit_2->setObjectName(QString::fromUtf8("lineEdit_2"));

        gridLayout_2->addWidget(lineEdit_2, 2, 1, 1, 1);

        lineEdit = new QLineEdit(widget_5);
        lineEdit->setObjectName(QString::fromUtf8("lineEdit"));

        gridLayout_2->addWidget(lineEdit, 1, 1, 1, 1);

        label_6 = new QLabel(widget_5);
        label_6->setObjectName(QString::fromUtf8("label_6"));

        gridLayout_2->addWidget(label_6, 1, 0, 1, 1);


        gridLayout_3->addWidget(widget_5, 0, 0, 2, 1);


        verticalLayout_2->addWidget(widget_3);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 582, 21));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        MainWindow->setStatusBar(statusbar);
        QWidget::setTabOrder(lineEdit_3, lineEdit_4);
        QWidget::setTabOrder(lineEdit_4, lineEdit_8);
        QWidget::setTabOrder(lineEdit_8, lineEdit);
        QWidget::setTabOrder(lineEdit, lineEdit_2);
        QWidget::setTabOrder(lineEdit_2, bSaveConnect);
        QWidget::setTabOrder(bSaveConnect, bSend);
        QWidget::setTabOrder(bSend, bDisconnect);
        QWidget::setTabOrder(bDisconnect, bOpenGrip);
        QWidget::setTabOrder(bOpenGrip, bCloseGrip);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QCoreApplication::translate("MainWindow", "Kontrolcenter", nullptr));
        label->setText(QCoreApplication::translate("MainWindow", "Hastighed", nullptr));
        bSend->setText(QCoreApplication::translate("MainWindow", "Apply", nullptr));
        label_10->setText(QCoreApplication::translate("MainWindow", "Gribekraft", nullptr));
        label_2->setText(QCoreApplication::translate("MainWindow", "St\303\270rrelse", nullptr));
        label_7->setText(QCoreApplication::translate("MainWindow", "Manuel Styring", nullptr));
        bOpenGrip->setText(QCoreApplication::translate("MainWindow", "Open Gripper", nullptr));
        bCloseGrip->setText(QCoreApplication::translate("MainWindow", "Close Gripper", nullptr));
        bCalibrate->setText(QCoreApplication::translate("MainWindow", "Calibrate Gripper", nullptr));
        bDisconnect->setText(QCoreApplication::translate("MainWindow", " Disconnect fra Server ", nullptr));
        bSaveConnect->setText(QCoreApplication::translate("MainWindow", "Gem og Connect til Server", nullptr));
        label_5->setText(QCoreApplication::translate("MainWindow", "Set Port", nullptr));
        label_6->setText(QCoreApplication::translate("MainWindow", "Set IP", nullptr));
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
