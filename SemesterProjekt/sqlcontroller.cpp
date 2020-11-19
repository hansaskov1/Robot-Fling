#include "sqlcontroller.h"

SQLController::SQLController()
{

}

SQLController::SQLController(std::string hostname, std::string username, std::string password, std::string dbname)
{
    QSqlDatabase database = QSqlDatabase::addDatabase("QODBC3");
    database.setHostName(QString::fromStdString(hostname));
    database.setUserName(QString::fromStdString(username));
    database.setPassword(QString::fromStdString(password));
    database.setDatabaseName("Driver={MySQL ODBC 8.0 Unicode Driver};DATABASE=" + QString::fromStdString(dbname) + ";");
    if (database.open())
        std::cout << "Connected" << std::endl;
}

bool SQLController::connect(std::string hostname, std::string username, std::string password, std::string dbname)
{
    QSqlDatabase database = QSqlDatabase::addDatabase("QODBC3");
    database.setHostName(QString::fromStdString(hostname));
    database.setUserName(QString::fromStdString(username));
    database.setPassword(QString::fromStdString(password));
    database.setDatabaseName("Driver={MySQL ODBC 8.0 Unicode Driver};DATABASE=" + QString::fromStdString(dbname) + ";");
    return database.open();
}

std::vector<Throw> SQLController::getThrows(bool &success)
{
    QSqlQuery query;
    query.prepare("SELECT * FROM kast");
    success = query.exec();
    std::vector<Throw> kast;
    while(query.next()) {
        kast.push_back(Throw(query.value(0).toUInt(), query.value(1).toString(), query.value(2).toDouble(), query.value(3).toDouble(), query.value(4).toBool()));
    }
    kast.shrink_to_fit();
    return kast;
}

std::vector<Throw> SQLController::searchThrow(std::string searchWord, std::string searchFor, bool &success)
{
    QSqlQuery query;
    query.prepare("SELECT * FROM kast WHERE " + QString::fromStdString(searchFor) + " like '%" + QString::fromStdString(searchWord) + "%'");
    success = query.exec();
    std::vector<Throw> kast;
    while(query.next()) {
        kast.push_back(Throw(query.value(0).toUInt(), query.value(1).toString(), query.value(2).toDouble(), query.value(3).toDouble(), query.value(4).toBool()));
    }
    kast.shrink_to_fit();
    return kast;
}

bool SQLController::insert(Throw kast)
{
    QSqlQuery query;
    //query.prepare("INSERT INTO bil (bil_id, registreringsnr, model, aargang, kunde_id) VALUES ('" + QString::number(bil.bilId()) + "','" + bil.regNr() + "','" + bil.model() + "','" + QString::number(bil.vintage()) + "','" + QString::number(bil.kundeId()) + "')");
    return query.exec();
}
