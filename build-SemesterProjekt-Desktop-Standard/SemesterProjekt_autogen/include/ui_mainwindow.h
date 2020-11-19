/********************************************************************************
** Form generated from reading UI file 'mainwindow.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_MAINWINDOW_H
#define UI_MAINWINDOW_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
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
    QGridLayout *gridLayout_5;
    QWidget *widget_3;
    QGridLayout *gridLayout_3;
    QPushButton *bDisconnect;
    QPushButton *bSaveConnect;
    QWidget *widget_5;
    QGridLayout *gridLayout_2;
    QSpacerItem *horizontalSpacer_2;
    QLineEdit *liIP;
    QLabel *lIP;
    QLineEdit *liGripperIP;
    QLabel *lGripperIP;
    QWidget *widget;
    QGridLayout *gridLayout_4;
    QWidget *widget_2;
    QVBoxLayout *verticalLayout;
    QPushButton *bOpenGrip;
    QPushButton *bCloseGrip;
    QPushButton *bCalibrate;
    QSpacerItem *verticalSpacer_2;
    QWidget *widget_4;
    QVBoxLayout *verticalLayout_2;
    QComboBox *cbMethod;
    QPushButton *bSend;
    QSpacerItem *verticalSpacer;
    QWidget *widget_6;
    QGridLayout *gridLayout;
    QLabel *lImage;
    QSpacerItem *horizontalSpacer;
    QMenuBar *menubar;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *MainWindow)
    {
        if (MainWindow->objectName().isEmpty())
            MainWindow->setObjectName(QStringLiteral("MainWindow"));
        MainWindow->resize(1684, 1085);
        centralwidget = new QWidget(MainWindow);
        centralwidget->setObjectName(QStringLiteral("centralwidget"));
        gridLayout_5 = new QGridLayout(centralwidget);
        gridLayout_5->setObjectName(QStringLiteral("gridLayout_5"));
        widget_3 = new QWidget(centralwidget);
        widget_3->setObjectName(QStringLiteral("widget_3"));
        gridLayout_3 = new QGridLayout(widget_3);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(-1, 9, -1, 9);
        bDisconnect = new QPushButton(widget_3);
        bDisconnect->setObjectName(QStringLiteral("bDisconnect"));

        gridLayout_3->addWidget(bDisconnect, 1, 2, 1, 1);

        bSaveConnect = new QPushButton(widget_3);
        bSaveConnect->setObjectName(QStringLiteral("bSaveConnect"));

        gridLayout_3->addWidget(bSaveConnect, 0, 2, 1, 1);

        widget_5 = new QWidget(widget_3);
        widget_5->setObjectName(QStringLiteral("widget_5"));
        gridLayout_2 = new QGridLayout(widget_5);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        horizontalSpacer_2 = new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout_2->addItem(horizontalSpacer_2, 1, 2, 1, 1);

        liIP = new QLineEdit(widget_5);
        liIP->setObjectName(QStringLiteral("liIP"));

        gridLayout_2->addWidget(liIP, 1, 1, 1, 1);

        lIP = new QLabel(widget_5);
        lIP->setObjectName(QStringLiteral("lIP"));

        gridLayout_2->addWidget(lIP, 1, 0, 1, 1);

        liGripperIP = new QLineEdit(widget_5);
        liGripperIP->setObjectName(QStringLiteral("liGripperIP"));

        gridLayout_2->addWidget(liGripperIP, 2, 1, 1, 1);

        lGripperIP = new QLabel(widget_5);
        lGripperIP->setObjectName(QStringLiteral("lGripperIP"));

        gridLayout_2->addWidget(lGripperIP, 2, 0, 1, 1);


        gridLayout_3->addWidget(widget_5, 0, 0, 2, 1);


        gridLayout_5->addWidget(widget_3, 1, 0, 1, 1);

        widget = new QWidget(centralwidget);
        widget->setObjectName(QStringLiteral("widget"));
        gridLayout_4 = new QGridLayout(widget);
        gridLayout_4->setObjectName(QStringLiteral("gridLayout_4"));
        widget_2 = new QWidget(widget);
        widget_2->setObjectName(QStringLiteral("widget_2"));
        verticalLayout = new QVBoxLayout(widget_2);
        verticalLayout->setObjectName(QStringLiteral("verticalLayout"));
        bOpenGrip = new QPushButton(widget_2);
        bOpenGrip->setObjectName(QStringLiteral("bOpenGrip"));

        verticalLayout->addWidget(bOpenGrip);

        bCloseGrip = new QPushButton(widget_2);
        bCloseGrip->setObjectName(QStringLiteral("bCloseGrip"));

        verticalLayout->addWidget(bCloseGrip);

        bCalibrate = new QPushButton(widget_2);
        bCalibrate->setObjectName(QStringLiteral("bCalibrate"));

        verticalLayout->addWidget(bCalibrate);

        verticalSpacer_2 = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout->addItem(verticalSpacer_2);


        gridLayout_4->addWidget(widget_2, 1, 3, 1, 1);

        widget_4 = new QWidget(widget);
        widget_4->setObjectName(QStringLiteral("widget_4"));
        verticalLayout_2 = new QVBoxLayout(widget_4);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        cbMethod = new QComboBox(widget_4);
        cbMethod->setObjectName(QStringLiteral("cbMethod"));

        verticalLayout_2->addWidget(cbMethod);

        bSend = new QPushButton(widget_4);
        bSend->setObjectName(QStringLiteral("bSend"));

        verticalLayout_2->addWidget(bSend);

        verticalSpacer = new QSpacerItem(20, 40, QSizePolicy::Minimum, QSizePolicy::Expanding);

        verticalLayout_2->addItem(verticalSpacer);


        gridLayout_4->addWidget(widget_4, 1, 0, 1, 1);

        widget_6 = new QWidget(widget);
        widget_6->setObjectName(QStringLiteral("widget_6"));
        gridLayout = new QGridLayout(widget_6);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        lImage = new QLabel(widget_6);
        lImage->setObjectName(QStringLiteral("lImage"));

        gridLayout->addWidget(lImage, 0, 0, 1, 1);

        horizontalSpacer = new QSpacerItem(40, 1, QSizePolicy::Expanding, QSizePolicy::Minimum);

        gridLayout->addItem(horizontalSpacer, 1, 0, 1, 1);


        gridLayout_4->addWidget(widget_6, 1, 2, 1, 1);


        gridLayout_5->addWidget(widget, 0, 0, 1, 1);

        MainWindow->setCentralWidget(centralwidget);
        menubar = new QMenuBar(MainWindow);
        menubar->setObjectName(QStringLiteral("menubar"));
        menubar->setGeometry(QRect(0, 0, 1684, 26));
        MainWindow->setMenuBar(menubar);
        statusbar = new QStatusBar(MainWindow);
        statusbar->setObjectName(QStringLiteral("statusbar"));
        MainWindow->setStatusBar(statusbar);

        retranslateUi(MainWindow);

        QMetaObject::connectSlotsByName(MainWindow);
    } // setupUi

    void retranslateUi(QMainWindow *MainWindow)
    {
        MainWindow->setWindowTitle(QApplication::translate("MainWindow", "MainWindow", Q_NULLPTR));
        bDisconnect->setText(QApplication::translate("MainWindow", " Disconnect", Q_NULLPTR));
        bSaveConnect->setText(QApplication::translate("MainWindow", "Connect", Q_NULLPTR));
        lIP->setText(QApplication::translate("MainWindow", "Set IP", Q_NULLPTR));
        lGripperIP->setText(QApplication::translate("MainWindow", "Set Gripper IP", Q_NULLPTR));
        bOpenGrip->setText(QApplication::translate("MainWindow", "Open Gripper", Q_NULLPTR));
        bCloseGrip->setText(QApplication::translate("MainWindow", "Close Gripper", Q_NULLPTR));
        bCalibrate->setText(QApplication::translate("MainWindow", "Calibrate", Q_NULLPTR));
        cbMethod->clear();
        cbMethod->insertItems(0, QStringList()
         << QApplication::translate("MainWindow", "Alex", Q_NULLPTR)
         << QApplication::translate("MainWindow", "Kenneth", Q_NULLPTR)
        );
        bSend->setText(QApplication::translate("MainWindow", "Start", Q_NULLPTR));
        lImage->setText(QString());
    } // retranslateUi

};

namespace Ui {
    class MainWindow: public Ui_MainWindow {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_MAINWINDOW_H
