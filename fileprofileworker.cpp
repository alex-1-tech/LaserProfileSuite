#include "fileprofileworker.h"
#include <QFile>
#include <QTextStream>
#include <QDebug>
#include <QCoreApplication>
#include <cmath>

FileProfileWorker::FileProfileWorker(QObject *parent)
    : QObject(parent)
    , timer(new QTimer(this))
    , currentIndex(0)
    , pointsPerUpdate(50)
{
    connect(timer, &QTimer::timeout, this, &FileProfileWorker::onTimeout);
}

FileProfileWorker::~FileProfileWorker()
{
    stop();
}

void FileProfileWorker::start()
{
    QString filename = "profile1.csv";

    QStringList searchPaths;
    searchPaths << QCoreApplication::applicationDirPath() + "../.."
                << "../..";

    bool fileFound = false;
    QString foundPath;

    for (const QString& path : searchPaths) {
        QString fullPath = path + "/" + filename;
        if (QFile::exists(fullPath)) {
            foundPath = fullPath;
            fileFound = true;
            break;
        }
    }

    if (!fileFound) {
        generateTestProfile();
        return;
    }

    if (!loadProfileFromCSV(foundPath)) {
        emit error("Failed to load profile from CSV file");
        return;
    }

    currentIndex = 0;
    timer->start(100);
}

void FileProfileWorker::generateTestProfile(){
    qDebug() << "File profile.csv not found, creating test data...";

    // (Sinusoid)
    for (int i = 0; i < 1000; ++i) {
        double x = i * 0.3; // x: 0...300
        double z = 50.0 * std::sin(x * 0.1) + 20.0 * std::cos(x * 0.05);

        ProfilePoint point;
        point.x = x - 150.0; // x: -150...150
        point.z = z;
        point.valid = true;

        if (i > 200 && i < 210) {
            point.valid = false;
        }

        profileData.append(point);
    }

    emit profileReady(profileData);
    timer->start(100);
}

void FileProfileWorker::stop()
{
    if (timer->isActive()) {
        timer->stop();
    }
}

void FileProfileWorker::setUpdateInterval(int ms)
{
    if (timer->isActive()) {
        timer->setInterval(ms);
    }
}

bool FileProfileWorker::loadProfileFromCSV(const QString& filename)
{
    QFile file(filename);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Cannot open file:" << filename;
        return false;
    }

    profileData.clear();
    QTextStream in(&file);

    // Пропускаем первую строку (заголовок)
    QString header = in.readLine();
    qDebug() << "CSV header:" << header;

    int lineNumber = 1;
    int loadedPoints = 0;
    int invalidPoints = 0;

    while (!in.atEnd()) {
        QString line = in.readLine().trimmed();
        lineNumber++;

        if (line.isEmpty()) {
            continue;
        }

        // Qt 5.7.1 - QString::split (Qt 6.*.* - Qt::SkipEmptyParts)
        QStringList parts = line.split(';');

        QStringList cleanParts;
        for (const QString& part : parts) {
            if (!part.trimmed().isEmpty()) {
                cleanParts.append(part);
            }
        }

        // x и z
        if (cleanParts.size() < 2) {
            qDebug() << "Warning: Line" << lineNumber << "has only" << cleanParts.size() << "parts:" << line;
            continue;
        }

        ProfilePoint point;
        bool okX, okZ;

        point.x = cleanParts[0].trimmed().toDouble(&okX);
        if (!okX) {
            qDebug() << "Warning: Line" << lineNumber << "invalid x value:" << cleanParts[0];
            continue;
        }

        point.z = cleanParts[1].trimmed().toDouble(&okZ);
        if (!okZ) {
            qDebug() << "Warning: Line" << lineNumber << "invalid z value:" << cleanParts[1];
            continue;
        }

        // invalid point:  point <= -999.0;
        if (point.z <= -999.0 || qAbs(point.z + 999.999) < 0.001) {
            point.valid = false;
            invalidPoints++;
        } else {
            point.valid = true;
        }


        // ** ---unused--- **
        if (cleanParts.size() >= 3) {
            QString validStr = cleanParts[2].trimmed().toLower();
            if (validStr == "0" || validStr == "false" || validStr == "invalid") {
                point.valid = false;
                invalidPoints++;
            } else if (validStr == "1" || validStr == "true" || validStr == "valid") {
                point.valid = true;
            }
        }
        // ** ------------ **

        profileData.append(point);
        loadedPoints++;

        // Debug information
        if (loadedPoints <= 5) {
            qDebug() << "Loaded point" << loadedPoints << ": x=" << point.x
                     << "z=" << point.z << "valid=" << point.valid;
        }
    }

    file.close();

    qDebug() << "Successfully loaded" << loadedPoints << "points from" << filename;
    qDebug() << "Valid points:" << (loadedPoints - invalidPoints);
    qDebug() << "Invalid points:" << invalidPoints;

    if (profileData.isEmpty()) {
        qDebug() << "Error: No points loaded from file";
        return false;
    }

    return true;
}
void FileProfileWorker::onTimeout()
{
    if (profileData.isEmpty()) {
        return;
    }

    if (pointsPerUpdate >= profileData.size() || currentIndex == 0) {
        emit profileReady(profileData);
        currentIndex = profileData.size();
    } else {
        int endIndex = qMin(currentIndex + pointsPerUpdate, profileData.size());
        QVector<ProfilePoint> partialData;
        for (int i = currentIndex; i < endIndex; ++i) {
            partialData.append(profileData[i]);
        }
        emit profileReady(partialData);
        currentIndex = endIndex;
    }

    if (currentIndex >= profileData.size()) {
        currentIndex = 0;
    }
}
