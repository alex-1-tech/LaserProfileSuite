#include "laserworker.h"
#include <iostream>

LaserWorker::LaserWorker(QObject* parent)
    : QObject(parent), timer(nullptr)
{
}

LaserWorker::~LaserWorker()
{
    stop();
}

void LaserWorker::start()
{
    std::cout << "[INFO] Laser worker started\n";

    if (!timer) {
        timer = new QTimer(this);
        timer->setInterval(50); // 20 Hz

        connect(timer, SIGNAL(timeout()),
                this, SLOT(updateProfile()));
    }

    timer->start();
}

void LaserWorker::stop()
{
    if (timer) {
        timer->stop();
        delete timer;
        timer = nullptr;
    }

    std::cout << "[INFO] Laser worker stopped\n";
}

void LaserWorker::updateProfile()
{
    auto profile = provider.loadProfile();
    emit profileReady(profile);
}
