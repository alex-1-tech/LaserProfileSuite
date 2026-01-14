#pragma once
/**
 * @file    segmentinfo.h
 * @brief   Geometric data structures for laser profile analysis
 * @author  alex-1-tech
 * @date    2025
 */

#include <QString>
#include <QVector>
#include <QPointF>

/**
 * @struct ProfilePoint
 * @brief Represents a single measurement point from laser profile
 *
 * Contains X-Z coordinates with validity flag for invalid measurements.
 * Coordinates are typically in millimeters (mm).
 */
struct ProfilePoint {
    double x;      ///< X coordinate (mm)
    double z;      ///< Z coordinate (mm)
    bool valid;    ///< Flag indicating valid measurement
};

/**
 * @struct SegmentInfo
 * @brief Geometric segment information extracted from profile data
 *
 * Represents a line segment identified in the profile data, with
 * comprehensive geometric properties for analysis and visualization.
 */
struct SegmentInfo {
    int id;                         ///< Unique segment identifier
    QString type;                   ///< Type: "line", "arc", "corner", etc.
    double length;                  ///< Segment length (mm)
    double angle;                   ///< Segment angle relative to horizontal (degrees)
    double startX, endX;            ///< X coordinates of segment endpoints (mm)
    double startZ, endZ;            ///< Z coordinates of segment endpoints (mm)
    double rmsError;                ///< Root mean square error of fit
    int pointCount;                 ///< Number of profile points in this segment

    // Line merging capabilities
    bool isLongLine;                ///< True if this is a merged long line
    QVector<int> mergedSegmentIds;  ///< IDs of segments merged into this line
    QVector<QPointF> points;        ///< All points belonging to this segment/line
};

/**
 * @struct GeometryResult
 * @brief Container for geometric analysis results
 *
 * Holds complete results from profile segmentation including
 * individual segments and merged long lines.
 */
struct GeometryResult {
    QVector<SegmentInfo> segments;  ///< All detected segments
    QVector<SegmentInfo> longLines; ///< Merged long lines (for linear features)
    int segmentCount;               ///< Total number of detected segments
    int lineCount;                  ///< Number of merged long lines
    double totalLength;             ///< Sum of all segment lengths (mm)
    double minLength;               ///< Minimum segment length (mm)
    double maxLength;               ///< Maximum segment length (mm)
    QString status;                 ///< Processing status/error message
};

/**
 * @struct DistanceInfo
 * @brief Distance measurement between two geometric features
 *
 * Represents a distance measurement between two lines/segments
 * with measurement type and endpoints.
 */
struct DistanceInfo {
    int line1Id;                  ///< ID of first line/segment
    int line2Id;                  ///< ID of second line/segment
    QString line1Type;            ///< Type of first line
    QString line2Type;            ///< Type of second line
    double distance;              ///< Distance between lines (mm)
    QString measurementType;      ///< "parallel", "perpendicular", "diagonal"
    QPointF measurementStart;     ///< Start point of measurement vector
    QPointF measurementEnd;       ///< End point of measurement vector
};

/**
 * @struct DistanceResult
 * @brief Container for distance measurement results
 */
struct DistanceResult {
    QVector<DistanceInfo> distances;    ///< All distance measurements
    QString status;                     ///< Processing status/error message
};

/**
 * @struct RANSACParams
 * @brief Parameters for RANSAC (Random Sample Consensus) line detection
 *
 * Controls the behavior of RANSAC algorithm for robust line fitting
 * in noisy profile data.
 */
struct RANSACParams {
    int maxIterations = 600;                ///< Maximum iterations for RANSAC
    double distanceThreshold = 0.15;        ///< Max point-to-line distance for inliers (mm)
    int minInliers = 3;                     ///< Minimum inliers to form a line
    double minLineLength = 0.5;             ///< Minimum line length to keep (mm)
    int maxLinesToFind = 25;                ///< Maximum lines to detect
    double mergeAngleTolerance = 6.0;       ///< Angle tolerance for merging lines (degrees)
    double mergeDistanceTolerance = 0.3;    ///< Distance tolerance for merging lines (mm)
};

/**
 * @struct LinearFitParams
 * @brief Parameters for linear regression-based segmentation
 *
 * Controls iterative linear fitting with merging capabilities.
 */
struct LinearFitParams {
    int minPointsPerSegment = 3;            ///< Minimum points to form a segment
    double residualTolerance = 0.08;        ///< Maximum residual error (mm)
    double mergeAngleTolerance = 4.5;       ///< Angle tolerance for merging (degrees)
    double mergeDistanceTolerance = 0.4;    ///< Distance tolerance for merging (mm)
};

/**
 * @struct HoughParams
 * @brief Parameters for Hough Transform line detection
 *
 * Controls Hough Transform implementation for line detection
 * in profile data.
 */
struct HoughParams {
    double dTheta = 1.5;                    ///< Angular resolution (degrees)
    int thetaBins = 120;                    ///< Number of angle bins
    double dRho = 0.15;                     ///< Distance resolution (mm)
    double minVotesPercent = 15.0;          ///< Minimum votes as percentage of points
    int minPointsPerLine = 3;               ///< Minimum points to form a line
    double mergeAngleTolerance = 3.0;       ///< Angle tolerance for merging (degrees)
    double mergeDistanceTolerance = 0.5;    ///< Distance tolerance for merging (mm)
};
