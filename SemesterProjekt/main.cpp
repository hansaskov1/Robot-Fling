#include "mainwindow.h"

#include <QApplication>
#include <iostream>
#include <QtSql/QSqlDatabase>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    MainWindow w;
    w.show();
    /*
    QSqlDatabase db = QSqlDatabase::addDatabase("QODBC3");
    db.setDatabaseName("Driver={MySQL ODBC 8.0 Unicode Driver};DATABASE=kastDB;");
    db.setUserName("root");
    db.setPassword("Tarzan12!");
    db.setHostName("localhost");
    if (db.open())
        std::cout << "Success!" << std::endl;
        */
    return a.exec();
}
