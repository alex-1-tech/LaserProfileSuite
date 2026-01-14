#pragma once

/**
 * @file    profileanalyzer.h
 * @brief   Static analyzer class for geometric segmentation of profile data
 * @author  alex-1-tech
 * @date    2025
 */

#include "segmentinfo.h"
#include <QVector>
#include <QPointF>
#include <QString>
#include <utility>

/**
 * @class   ProfileAnalyzer
 * @brief   Static analyzer for segmenting profile data into geometric primitives
 *
 * Provides multiple algorithms for line extraction:
 * - Linear fit segmentation
 * - Hough transform detection
 * - RANSAC robust fitting
 */
class ProfileAnalyzer
{
public:
    static GeometryResult analyzeGeometry(const QVector<ProfilePoint>& profile,
                                          const LinearFitParams& params = LinearFitParams());  ///< Linear fit analysis
    static GeometryResult analyzeGeometryHough(const QVector<ProfilePoint>& profile,
                                               const HoughParams& params = HoughParams());  ///< Hough transform analysis
    static GeometryResult analyzeGeometryRANSAC(const QVector<ProfilePoint>& profile,
                                                const RANSACParams& params = RANSACParams());  ///< RANSAC analysis

private:
    static QVector<SegmentInfo> extractMicroSegments(const QVector<double>& x, const QVector<double>& z);  ///< Extract initial micro-segments
    static QVector<SegmentInfo> extractSegmentsWithLinearFit(const QVector<double>& x, const QVector<double>& z,
                                                             const LinearFitParams& params);  ///< Linear fit segmentation
    static QVector<SegmentInfo> extractSegmentsWithHough(const QVector<double>& x, const QVector<double>& z,
                                                         const HoughParams& params);  ///< Hough-based segmentation
    static QVector<SegmentInfo> extractSegmentsWithRANSAC(const QVector<double>& x, const QVector<double>& z,
                                                          const RANSACParams& params);  ///< RANSAC-based segmentation

    static QVector<SegmentInfo> mergeLines(const QVector<SegmentInfo>& segments,
                                           double angleTolerance, double distanceTolerance);  ///< Merge similar lines
    static QVector<SegmentInfo> mergeHoughLines(const QVector<SegmentInfo>& segments,
                                                double angleTolerance, double distanceTolerance);  ///< Merge Hough lines

    static std::pair<double, double> linearFit(const QVector<double>& x, const QVector<double>& z,
                                               int start, int end);  ///< Compute linear regression coefficients
    static double calculateAngle(double dx, double dz);  ///< Calculate angle from direction vector
    static QString classifySegment(double angle);  ///< Classify segment by angle (horizontal/vertical/diagonal)
    static double calculateOverlap(const SegmentInfo& a, const SegmentInfo& b);  ///< Calculate overlap between segments
    static double normalizeAngle180(double ang);  ///< Normalize angle to [-180, 180] range
    static double angleDifference(double a, double b);  ///< Compute minimal angle difference
    static double projectScalarAlong(double px, double pz, double x0, double z0, double ux, double uz);  ///< Project point onto line
    static double perpDistance(double px, double pz, double x0, double z0, double ux, double uz);  ///< Calculate perpendicular distance
    static double distanceBetweenLines(const SegmentInfo& line1, const SegmentInfo& line2);  ///< Compute distance between two lines
    static double distancePointToLine(double px, double pz, double x1, double z1, double x2, double z2);  ///< Distance from point to line
};
