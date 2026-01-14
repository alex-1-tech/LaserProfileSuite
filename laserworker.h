#pragma once

/**
 * @file    laserworker.h
 * @brief   Worker class for streaming laser profile data in real-time
 * @author  alex-1-tech
 * @date    2025
 */

#include <QObject>
#include <QVector>
#include <QTimer>
#include "profileprovider.h"

/**
 * @class   LaserWorker
 * @brief   Real-time laser profile data streaming worker
 *
 * This worker class operates in a separate thread and provides:
 * - Real-time profile data acquisition from laser device
 * - Configurable streaming frequency (default 20 Hz)
 * - Thread-safe start/stop operations
 * - Periodic data emission via signals
 *
 * @note Uses ProfileProvider for actual data acquisition,
 *       allowing easy switching between different data sources
 */
class LaserWorker : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Constructor for LaserWorker
     * @param parent Parent QObject (optional)
     */
    explicit LaserWorker(QObject* parent = nullptr);

    ~LaserWorker();             ///< @brief Destructor - ensures proper cleanup and stops streaming

public slots:
    void start();               ///< @brief Start laser data acquisition and streaming
    void stop();                ///< @brief Stop laser data acquisition and streaming

signals:
    /**
     * @brief Emitted when new laser profile data is available
     * @param profile Vector of profile points containing current laser measurements
     */
    void profileReady(const QVector<ProfilePoint>& profile);

private slots:
    /**
     * @brief Timer timeout handler - acquires and emits new profile data
     *
     * Called periodically by QTimer. Retrieves latest profile from provider
     * and emits it via profileReady signal.
     */
    void updateProfile();

private:
    QTimer* timer;              ///< Periodic data acquisition timer (50ms = 20Hz)
    ProfileProvider provider;   ///< Data source provider for laser profiles
};
