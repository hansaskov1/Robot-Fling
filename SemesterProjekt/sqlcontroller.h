#ifndef SQLCONTROLLER_H
#define SQLCONTROLLER_H

#include "Throw.h"
#include <iostream>
#include <QtSql>
#include <QSqlDatabase>


class SQLController
{
public:
    SQLController();
    SQLController(std::string hostname, std::string username, std::string password, std::string dbname);

    bool connect(std::string hostname, std::string username, std::string password, std::string dbname);

    std::vector<Throw> getThrows(bool&);
    std::vector<Throw> searchThrow(std::string, std::string, bool&);
    bool insert(Throw);
};

#endif // SQLCONTROLLER_H
