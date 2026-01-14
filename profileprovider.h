#pragma once

/**
 * @file    profileprovider.h
 * @brief   Provider class for acquiring laser profile data from Keyence LJV7 controller
 * @author  alex-1-tech
 * @date    2025
 */

#include <QVector>
#include <QPointF>
#include <QMetaType>
#include "segmentinfo.h"

/**
 * @class ProfileProvider
 * @brief Provides laser profile data from Keyence LJV7 measurement controller
 *
 * This class serves as a bridge between the raw LJV7 controller API and
 * the application's data model, converting binary profile data into
 * structured QVector<ProfilePoint> format suitable for Qt-based applications.
 *
 * @note Requires LJV7_IF.dll and properly connected Keyence laser sensor
 */
class ProfileProvider
{
public:
    /**
     * @brief Load profile data from Keyence laser controller
     * @return QVector<ProfilePoint> containing converted profile measurements
     *
     * Performs the following operations:
     * 1. Initializes LJV7 controller DLL
     * 2. Opens USB connection to device
     * 3. Requests profile data in current position mode
     * 4. Converts raw binary data to millimeter scale
     * 5. Handles invalid measurement points
     * 6. Returns structured profile data
     *
     * @throws std::exception via LJController wrapper
     */
    QVector<ProfilePoint> loadProfile();
};

Q_DECLARE_METATYPE(ProfilePoint)            ///< Enable ProfilePoint for Qt signals/slots
Q_DECLARE_METATYPE(QVector<ProfilePoint>)   ///< Enable ProfilePoint vector for Qt signals/slots
