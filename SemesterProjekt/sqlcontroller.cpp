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
    int throwID = t.getThrowID();
    t.getPaths().shrink_to_fit();
    double throwTime = 0;
    for (unsigned int i = 0; i < t.getPaths().size(); i++)
        if (t.getPaths().at(i).getThrowTime() != 0)
            throwTime = t.getPaths().at(i).getThrowTime();
    QString throwString = "INSERT INTO throw (throwID, object, angle, speed, throwTime, success) VALUES (" + QString::number(throwID) + ", '" + t.getObject() + "', " + QString::number(t.getAngle()) + ", " + QString::number(t.getSpeed()) + ", " + QString::number(throwTime) + ", " + QString::number(t.isSuccess()) + ");";
    QSqlQuery throwQuery;

    throwQuery.prepare(throwString);
    std::cout << throwString.toStdString() << std::endl;
    throwQuery.exec();

    QSqlQuery query1;
    QSqlQuery query2;
    QSqlQuery query3;
    QSqlQuery query4;

    QString jointpose = "INSERT INTO jointpose VALUES ";
    QString jointvel = "INSERT INTO jointvelocity VALUES ";
    QString toolpose = "INSERT INTO toolpose VALUES ";
    QString toolvel = "INSERT INTO toolvelocity VALUES ";
    //paths
    for (unsigned int i = 0; i < t.getPaths().size(); i++) {
        //jointposevector
        t.getPaths().at(i).getJointPoses().shrink_to_fit();
        for (unsigned int j = 0; j < t.getPaths().at(i).getJointPoses().size(); j++) {
            jointpose.append("(" + QString::number(j+1) + ", ");
            jointvel.append("(" + QString::number(j+1) + ", ");
            toolpose.append("(" + QString::number(j+1) + ", ");
            toolvel.append("(" + QString::number(j+1) + ", ");
            //jointpose
            for (unsigned int u = 0; u < 6; u++) {
                jointpose.append(QString::number(t.getPaths().at(i).getJointPoses().at(j).at(u)) + ", ");
                jointvel.append(QString::number(t.getPaths().at(i).getJointVel().at(j).at(u)) + ", ");
                toolpose.append(QString::number(t.getPaths().at(i).getToolPose().at(j).at(u)) + ", ");
                toolvel.append(QString::number(t.getPaths().at(i).getToolVel().at(j).at(u)) + ", ");
            }
            jointpose.append(QString::number(t.getPaths().at(i).getElapsedTime().at(j)) + ", " + QString::number(i+1) + ", " + QString::number(throwID));
            jointvel.append(QString::number(t.getPaths().at(i).getElapsedTime().at(j)) + ", " + QString::number(i+1) + ", " + QString::number(throwID));
            toolpose.append(QString::number(t.getPaths().at(i).getElapsedTime().at(j)) + ", " + QString::number(i+1) + ", " + QString::number(throwID));
            toolvel.append(QString::number(t.getPaths().at(i).getElapsedTime().at(j)) + ", " + QString::number(i+1) + ", " + QString::number(throwID));
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getJointPoses().size() - 1) ? jointpose.append("); ") : jointpose.append("), ");
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getJointVel().size() - 1) ? jointvel.append("); ") : jointvel.append("), ");
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getToolPose().size() - 1) ? toolpose.append("); ") : toolpose.append("), ");
            (i == t.getPaths().size() - 1 && j == t.getPaths().at(i).getToolVel().size() - 1) ? toolvel.append("); ") : toolvel.append("), ");
        }
        std::cout << "Path: " << i << std::endl;
    }
    query1.prepare(jointpose);
    std::cout << jointpose.toStdString() << std::endl << std::endl;
    query1.exec();
    query2.prepare(jointvel);
    std::cout << jointvel.toStdString() << std::endl << std::endl;
    query2.exec();
    query3.prepare(toolpose);
    std::cout << toolpose.toStdString() << std::endl << std::endl;
    query3.exec();
    query4.prepare(toolvel);
    std::cout << toolvel.toStdString() << std::endl << std::endl;
    query4.exec();

    return true;
}
