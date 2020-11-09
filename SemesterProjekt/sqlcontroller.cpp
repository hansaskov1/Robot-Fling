#include "sqlcontroller.h"

SQLController::SQLController()
{

}

SQLController::SQLController(std::string hostname, std::string username, std::string password, std::string dbname)
{
    QSqlDatabase database = QSqlDatabase::addDatabase("QMYSQL");
    database.setHostName(QString::fromStdString(hostname));
    database.setPort(3306);
    database.setUserName(QString::fromStdString(username));
    database.setPassword(QString::fromStdString(password));
    database.setDatabaseName(QString::fromStdString(dbname));
    if (database.open())
        std::cout << "Connected" << std::endl;
}

bool SQLController::connect(std::string hostname, std::string username, std::string password, std::string dbname)
{
    QSqlDatabase database = QSqlDatabase::addDatabase("QMYSQL");
    database.setHostName(QString::fromStdString(hostname));
    database.setPort(3306);
    database.setUserName(QString::fromStdString(username));
    database.setPassword(QString::fromStdString(password));
    database.setDatabaseName(QString::fromStdString(dbname));
    return database.open();
}

std::vector<Throw> SQLController::getThrows(bool &success)
{
    QSqlQuery query;
    query.prepare("SELECT * from kast, jointpose, jointvelocity, toolpose, toolvelocity where kast.kastID = jointpose.kastID and kast.kastID = jointvelocity.kastID and kast.kastID = toolpose.kastID and kast.kastID = toolvelocity.kastID and jointpose.stiNr = jointvelocity.stiNr and jointpose.stiNr = toolpose.stiNr and jointpose.stiNr = toolvelocity.stiNr and jointvelocity.stiNr = toolvelocity.stiNr and toolpose.stiNr = toolvelocity.stiNr and toolpose.stiNr = jointvelocity.stiNr and jointposeID = jointvelocityID and jointposeID = toolposeID and jointposeID = toolvelocityID and jointvelocityID = toolvelocityID and toolposeID = toolvelocityID and toolposeID = jointvelocityID");
    success = query.exec();
    std::vector<Throw> kast;
    std::vector<Path> paths;
    Path path;
    unsigned int kastID = 0;
    unsigned int pathID = 0;
    while(query.next()) {
        std::vector<double> jointpose;
        jointpose.push_back(query.value(6).toDouble());
        jointpose.push_back(query.value(7).toDouble());
        jointpose.push_back(query.value(8).toDouble());
        jointpose.push_back(query.value(9).toDouble());
        jointpose.push_back(query.value(10).toDouble());
        jointpose.push_back(query.value(11).toDouble());

        std::vector<double> jointvelocity;
        jointvelocity.push_back(query.value(15).toDouble());
        jointvelocity.push_back(query.value(16).toDouble());
        jointvelocity.push_back(query.value(17).toDouble());
        jointvelocity.push_back(query.value(18).toDouble());
        jointvelocity.push_back(query.value(19).toDouble());
        jointvelocity.push_back(query.value(20).toDouble());

        std::vector<double> toolpose;
        toolpose.push_back(query.value(24).toDouble());
        toolpose.push_back(query.value(25).toDouble());
        toolpose.push_back(query.value(26).toDouble());
        toolpose.push_back(query.value(27).toDouble());
        toolpose.push_back(query.value(28).toDouble());
        toolpose.push_back(query.value(29).toDouble());

        std::vector<double> toolvelocity;
        toolvelocity.push_back(query.value(33).toDouble());
        toolvelocity.push_back(query.value(34).toDouble());
        toolvelocity.push_back(query.value(35).toDouble());
        toolvelocity.push_back(query.value(36).toDouble());
        toolvelocity.push_back(query.value(37).toDouble());
        toolvelocity.push_back(query.value(38).toDouble());

        path.addPoint(jointpose, jointvelocity, toolpose, toolvelocity);

        if (pathID != query.value(12).toUInt()) {
            paths.push_back(path);
            pathID = query.value(12).toUInt();
        }
        if (kastID != query.value(0).toUInt()) {
            kastID = query.value(0).toUInt();
            QString objekt = query.value(1).toString();
            double vinkel = query.value(2).toDouble();
            double hastighed = query.value(3).toDouble();
            bool success = query.value(4).toBool();
            kast.push_back(Throw(kastID, objekt, vinkel, hastighed, paths, success));
        }
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
        //kast.push_back(Throw(query.value(0).toUInt(), query.value(1).toString(), query.value(2).toDouble(), query.value(3).toDouble(), query.value(4).toBool()));
    }
    kast.shrink_to_fit();
    return kast;
}

bool SQLController::insert(Throw kast)
{
    QSqlQuery query;
    QString sql = "INSERT INTO kast VALUES (" + QString::number(kast.getKastID()) + ", '" + kast.getObjekt() + "', " + QString::number(kast.getVinkel()) + ", " + QString::number(kast.getHastighed()) + ", " + QString::number(kast.isSuccess()) + "); ";
    /*
    sql.append("INSERT INTO sti VALUES ");
    kast.getPaths().shrink_to_fit();
    for (unsigned int i = 1; i < kast.getPaths().size(); i++)
        sql.append("(" + QString::number(i) + ", " + QString::number(kast.getKastID()) + "), ");
    sql.append("(" + QString::number(kast.getPaths().size()) + ", " + QString::number(kast.getKastID()) + "); ");
    */

    sql.append("INSERT INTO jointpose VALUES ");
    //paths
    for (unsigned int i = 0; i < kast.getPaths().size(); i++) {
        //jointposevector
        kast.getPaths().at(i).getJointPoses().shrink_to_fit();
        for (unsigned int j = 0; j < kast.getPaths().at(i).getJointPoses().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            kast.getPaths().at(i).getJointPoses().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < kast.getPaths().at(i).getJointPoses().at(j).size(); u++)
                sql.append(QString::number(kast.getPaths().at(i).getJointPoses().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(kast.getKastID()));
            (i == kast.getPaths().size() - 1 && j == kast.getPaths().at(i).getJointPoses().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO jointvelocity VALUES ");
    //paths
    for (unsigned int i = 0; i < kast.getPaths().size(); i++) {
        //jointposevector
        kast.getPaths().at(i).getJointVel().shrink_to_fit();
        for (unsigned int j = 0; j < kast.getPaths().at(i).getJointVel().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            kast.getPaths().at(i).getJointVel().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < kast.getPaths().at(i).getJointVel().at(j).size(); u++)
                sql.append(QString::number(kast.getPaths().at(i).getJointVel().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(kast.getKastID()));
            (i == kast.getPaths().size() - 1 && j == kast.getPaths().at(i).getJointVel().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO toolpose VALUES ");
    //paths
    for (unsigned int i = 0; i < kast.getPaths().size(); i++) {
        //jointposevector
        kast.getPaths().at(i).getToolPose().shrink_to_fit();
        for (unsigned int j = 0; j < kast.getPaths().at(i).getToolPose().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            kast.getPaths().at(i).getToolPose().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < kast.getPaths().at(i).getToolPose().at(j).size(); u++)
                sql.append(QString::number(kast.getPaths().at(i).getToolPose().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(kast.getKastID()));
            (i == kast.getPaths().size() - 1 && j == kast.getPaths().at(i).getToolPose().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO toolvelocity VALUES ");
    //paths
    for (unsigned int i = 0; i < kast.getPaths().size(); i++) {
        //jointposevector
        kast.getPaths().at(i).getToolVel().shrink_to_fit();
        for (unsigned int j = 0; j < kast.getPaths().at(i).getToolVel().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            kast.getPaths().at(i).getToolVel().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < kast.getPaths().at(i).getToolVel().at(j).size(); u++)
                sql.append(QString::number(kast.getPaths().at(i).getToolVel().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(kast.getKastID()));
            (i == kast.getPaths().size() - 1 && j == kast.getPaths().at(i).getToolVel().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    //std::cout << sql.toStdString() << std::endl;

    query.prepare(sql);
    return query.exec();
}
