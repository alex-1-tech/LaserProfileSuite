#include <QApplication>
#include <QMetaType>

#include "mainwindow.h"
#include "profileprovider.h"


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    qRegisterMetaType<ProfilePoint>("ProfilePoint");
    qRegisterMetaType<QVector<ProfilePoint>>("QVector<ProfilePoint>");

    MainWindow w;
    w.show();

    return a.exec();
}
