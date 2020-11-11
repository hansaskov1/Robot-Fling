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
    query.prepare("SELECT * from throw, jointpose, jointvelocity, toolpose, toolvelocity where throw.throwID = jointpose.throwID and throw.throwID = jointvelocity.throwID and throw.throwID = toolpose.throwID and throw.throwID = toolvelocity.throwID and jointpose.stiNr = jointvelocity.stiNr and jointpose.stiNr = toolpose.stiNr and jointpose.stiNr = toolvelocity.stiNr and jointvelocity.stiNr = toolvelocity.stiNr and toolpose.stiNr = toolvelocity.stiNr and toolpose.stiNr = jointvelocity.stiNr and jointposeID = jointvelocityID and jointposeID = toolposeID and jointposeID = toolvelocityID and jointvelocityID = toolvelocityID and toolposeID = toolvelocityID and toolposeID = jointvelocityID");
    success = query.exec();
    std::vector<Throw> throws;
    std::vector<Path> paths;
    Path path;
    unsigned int throwID = 0;
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
        if (throwID != query.value(0).toUInt()) {
            throwID = query.value(0).toUInt();
            QString object = query.value(1).toString();
            double angle = query.value(2).toDouble();
            double speed = query.value(3).toDouble();
            bool success = query.value(4).toBool();
            throws.push_back(Throw(throwID, object, angle, speed, paths, success));
        }
    }
    throws.shrink_to_fit();
    return throws;
}

std::vector<Throw> SQLController::searchThrow(std::string searchWord, std::string searchFor, bool &success)
{
    QSqlQuery query;
    query.prepare("SELECT * FROM throw WHERE " + QString::fromStdString(searchFor) + " like '%" + QString::fromStdString(searchWord) + "%'");
    success = query.exec();
    std::vector<Throw> throws;
    while(query.next()) {
        //throw.push_back(Throw(query.value(0).toUInt(), query.value(1).toString(), query.value(2).toDouble(), query.value(3).toDouble(), query.value(4).toBool()));
    }
    throws.shrink_to_fit();
    return throws;
}

bool SQLController::insert(Throw t)
{
    QSqlQuery selectQuery;

    selectQuery.prepare("SELECT throwID FROM throw");
    selectQuery.exec();

    while(selectQuery.next()) {
        if (t.getThrowID() == selectQuery.value(0).toUInt())
            t.setThrowID(selectQuery.value(0).toUInt() + 1);
        std::cout << t.getThrowID() << std::endl;
    }

    QSqlQuery throwQuery;

    throwQuery.prepare("INSERT INTO throw (object, angle, speed, success) VALUES ('" + t.getObject() + "', " + QString::number(t.getAngle()) + ", " + QString::number(t.getSpeed()) + ", " + QString::number(t.isSuccess()) + ");");
    throwQuery.exec();

    QSqlQuery query;

    QString sql = "";
    sql.append("INSERT INTO jointpose VALUES ");
    //paths
    for (unsigned int i = 0; i < t.getPaths().size(); i++) {
        //jointposevector
        t.getPaths().at(i).getJointPoses().shrink_to_fit();
        for (unsigned int j = 0; j < t.getPaths().at(i).getJointPoses().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            t.getPaths().at(i).getJointPoses().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < t.getPaths().at(i).getJointPoses().at(j).size(); u++)
                sql.append(QString::number(t.getPaths().at(i).getJointPoses().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(t.getThrowID()));
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getJointPoses().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO jointvelocity VALUES ");
    //paths
    for (unsigned int i = 0; i < t.getPaths().size(); i++) {
        //jointposevector
        t.getPaths().at(i).getJointVel().shrink_to_fit();
        for (unsigned int j = 0; j < t.getPaths().at(i).getJointVel().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            t.getPaths().at(i).getJointVel().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < t.getPaths().at(i).getJointVel().at(j).size(); u++)
                sql.append(QString::number(t.getPaths().at(i).getJointVel().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(t.getThrowID()));
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getJointVel().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO toolpose VALUES ");
    //paths
    for (unsigned int i = 0; i < t.getPaths().size(); i++) {
        //jointposevector
        t.getPaths().at(i).getToolPose().shrink_to_fit();
        for (unsigned int j = 0; j < t.getPaths().at(i).getToolPose().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            t.getPaths().at(i).getToolPose().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < t.getPaths().at(i).getToolPose().at(j).size(); u++)
                sql.append(QString::number(t.getPaths().at(i).getToolPose().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(t.getThrowID()));
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getToolPose().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    sql.append("INSERT INTO toolvelocity VALUES ");
    //paths
    for (unsigned int i = 0; i < t.getPaths().size(); i++) {
        //jointposevector
        t.getPaths().at(i).getToolVel().shrink_to_fit();
        for (unsigned int j = 0; j < t.getPaths().at(i).getToolVel().size(); j++) {
            sql.append("(" + QString::number(j+1) + ", ");
            //jointpose
            t.getPaths().at(i).getToolVel().at(j).shrink_to_fit();
            for (unsigned int u = 0; u < t.getPaths().at(i).getToolVel().at(j).size(); u++)
                sql.append(QString::number(t.getPaths().at(i).getToolVel().at(j).at(u)) + ", ");
            sql.append(QString::number(i+1) + ", " + QString::number(t.getThrowID()));
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getToolVel().size() - 1) ? sql.append("); ") : sql.append("), ");
        }
    }

    //std::cout << sql.toStdString() << std::endl;

    query.prepare(sql);
    return query.exec();
}
