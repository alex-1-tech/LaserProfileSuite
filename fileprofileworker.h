#pragma once

/**
 * @file    fileprofileworker.h
 * @brief   Worker class for loading and streaming profile measurement data
 * @author  alex-1-tech
 * @date    2025
 */

#include <QObject>
#include <QTimer>
#include <QVector>
#include "segmentinfo.h"

/**
 * @class   FileProfileWorker
 * @brief   Loads profile data from CSV files and streams it periodically
 *
 * This worker class operates in a separate thread and provides:
 * - CSV file loading with error handling
 * - Synthetic data generation for testing
 * - Periodic data emission via signals
 * - Configurable streaming rate
 */
class FileProfileWorker : public QObject
{
    Q_OBJECT

public:
    explicit FileProfileWorker(QObject *parent = nullptr);
    ~FileProfileWorker();

public slots:
    void start();               ///< Start loading and streaming data
    void stop();                ///< Stop data streaming

    /// @brief Set update interval in milliseconds
    /// @param ms Interval between data emissions
    void setUpdateInterval(int ms);

signals:
    /// @brief Emitted when new profile data is available
    /// @param profile Vector of profile points (full or partial)
    void profileReady(const QVector<ProfilePoint>& profile);

    /// @brief Emitted when an error occurs
    /// @param message Error description
    void error(const QString& message);

private slots:
    void onTimeout();   ///<  Timer timeout handler - emits next data chunk

private:
    QTimer* timer;                      ///< Periodic emission timer
    QVector<ProfilePoint> profileData;  ///< Complete loaded profile
    int currentIndex;                   ///< Current position in stream
    int pointsPerUpdate;                ///< Points per emission cycle

    /// @brief Generate synthetic test profile (sinusoidal)
    void generateTestProfile();

    /// @brief Load profile data from CSV file
    /// @param filename Path to CSV file
    /// @return true if loaded successfully, false otherwise
    bool loadProfileFromCSV(const QString& filename);
};
