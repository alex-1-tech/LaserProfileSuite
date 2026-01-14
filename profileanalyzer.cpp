#include "profileanalyzer.h"
#include <QVector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <QDebug>
#include <QHash>
#include <QPair>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================ ANGLE AND GEOMETRIC UTILITIES ============================

double ProfileAnalyzer::calculateAngle(double dx, double dz) {
    // Calculate angle in degrees [-180, 180] from direction vector
    if (std::abs(dx) < 1e-12) {
        return (dz >= 0.0) ? 90.0 : -90.0;
    }
    double angleRad = std::atan2(dz, dx);
    return angleRad * 180.0 / M_PI;
}

QString ProfileAnalyzer::classifySegment(double angle) {
    // Classify segment based on angle: horizontal, vertical, or sloped
    double absAngle = std::abs(angle);
    if (absAngle < 10.0) return "horizontal";
    if (absAngle > 80.0 && absAngle < 100.0) return "vertical";
    return "sloped";
}

double ProfileAnalyzer::normalizeAngle180(double ang) {
    // Normalize angle to [0, 180) range for parallel comparison
    double a = std::fmod(std::abs(ang), 180.0);
    if (a < 0.0) a += 180.0;
    return a;
}

double ProfileAnalyzer::angleDifference(double a, double b) {
    // Calculate minimal angular difference [0, 180]
    double diff = std::fabs(a - b);
    if (diff > 360.0) diff = std::fmod(diff, 360.0);
    if (diff > 180.0) diff = 360.0 - diff;
    return diff;
}

// ============================ LINEAR ALGEBRA UTILITIES ============================

std::pair<double, double> ProfileAnalyzer::linearFit(const QVector<double>& x,
                                                     const QVector<double>& z,
                                                     int start, int end) {
    // Least squares linear regression: z = slope*x + intercept
    if (start < 0 || end < start || end >= x.size() || end >= z.size()) {
        return {0.0, 0.0};
    }
    int n = end - start + 1;
    if (n < 2) return {0.0, 0.0};

    double sumX = 0.0, sumZ = 0.0, sumXX = 0.0, sumXZ = 0.0;
    for (int i = start; i <= end; ++i) {
        double xi = x[i];
        double zi = z[i];
        sumX += xi;
        sumZ += zi;
        sumXX += xi * xi;
        sumXZ += xi * zi;
    }
    double denom = n * sumXX - sumX * sumX;
    if (std::abs(denom) < 1e-12) return {0.0, 0.0};
    double slope = (n * sumXZ - sumX * sumZ) / denom;
    double intercept = (sumZ - slope * sumX) / n;
    return {slope, intercept};
}

double ProfileAnalyzer::projectScalarAlong(double px, double pz, double x0, double z0, double ux, double uz) {
    // Project point (px,pz) onto direction vector (ux,uz) relative to reference (x0,z0)
    return ( (px - x0) * ux + (pz - z0) * uz );
}

double ProfileAnalyzer::perpDistance(double px, double pz, double x0, double z0, double ux, double uz) {
    // Perpendicular distance from point to line defined by point (x0,z0) and direction (ux,uz)
    double cross = (px - x0) * uz - (pz - z0) * ux;
    return std::abs(cross);
}

double ProfileAnalyzer::distancePointToLine(double px, double pz,
                                            double x1, double z1, double x2, double z2) {
    // Distance from point to line segment (not infinite line)
    double vx = x2 - x1;
    double vz = z2 - z1;
    double wx = px - x1;
    double wz = pz - z1;
    double vlen2 = vx*vx + vz*vz;
    if (vlen2 < 1e-12) {
        // Degenerate segment: distance to point
        return std::sqrt(wx*wx + wz*wz);
    }
    double t = (wx*vx + wz*vz) / vlen2;
    if (t < 0.0) t = 0.0;       // Clamp to segment start
    if (t > 1.0) t = 1.0;       // Clamp to segment end
    double projX = x1 + t * vx;
    double projZ = z1 + t * vz;
    double dx = px - projX;
    double dz = pz - projZ;
    return std::sqrt(dx*dx + dz*dz);
}

double ProfileAnalyzer::distanceBetweenLines(const SegmentInfo& line1, const SegmentInfo& line2) {
    // Minimum distance between two line segments (4 endpoint-to-segment distances)
    double dx1 = line1.endX - line1.startX;
    double dz1 = line1.endZ - line1.startZ;
    double len1 = std::sqrt(dx1*dx1 + dz1*dz1);
    if (len1 < 1e-12) {
        // Degenerate line: use midpoint distance
        double mx1 = 0.5*(line1.startX + line1.endX);
        double mz1 = 0.5*(line1.startZ + line1.endZ);
        double mx2 = 0.5*(line2.startX + line2.endX);
        double mz2 = 0.5*(line2.startZ + line2.endZ);
        return std::sqrt((mx2-mx1)*(mx2-mx1) + (mz2-mz1)*(mz2-mz1));
    }

    // Calculate all 4 possible distances
    double d1 = distancePointToLine(line2.startX, line2.startZ, line1.startX, line1.startZ, line1.endX, line1.endZ);
    double d2 = distancePointToLine(line2.endX, line2.endZ, line1.startX, line1.startZ, line1.endX, line1.endZ);
    double d3 = distancePointToLine(line1.startX, line1.startZ, line2.startX, line2.startZ, line2.endX, line2.endZ);
    double d4 = distancePointToLine(line1.endX, line1.endZ, line2.startX, line2.startZ, line2.endX, line2.endZ);

    return std::min(std::min(d1,d2), std::min(d3,d4));
}

double ProfileAnalyzer::calculateOverlap(const SegmentInfo& a, const SegmentInfo& b) {
    // Calculate X-axis overlap ratio between two segments [0, 1]
    double aMin = std::min(a.startX, a.endX);
    double aMax = std::max(a.startX, a.endX);
    double bMin = std::min(b.startX, b.endX);
    double bMax = std::max(b.startX, b.endX);

    double overlap = std::max(0.0, std::min(aMax, bMax) - std::max(aMin, bMin));
    double total = std::max(aMax, bMax) - std::min(aMin, bMin);

    return (total > 0) ? overlap / total : 0.0;
}

// ============================ MICRO-SEGMENT EXTRACTION ============================

QVector<SegmentInfo> ProfileAnalyzer::extractMicroSegments(const QVector<double>& x,
                                                           const QVector<double>& z) {
    // Create segments between consecutive points for Segments table
    QVector<SegmentInfo> segments;
    if (x.size() < 2 || z.size() < 2 || x.size() != z.size()) return segments;

    for (int i = 0; i + 1 < x.size(); ++i) {
        SegmentInfo seg;
        seg.id = static_cast<int>(segments.size() + 1);
        seg.startX = x[i];
        seg.endX = x[i+1];
        seg.startZ = z[i];
        seg.endZ = z[i+1];
        double dx = seg.endX - seg.startX;
        double dz = seg.endZ - seg.startZ;
        seg.length = std::sqrt(dx*dx + dz*dz);
        seg.angle = calculateAngle(dx, dz);
        seg.type = classifySegment(seg.angle);
        seg.pointCount = 2;
        seg.isLongLine = false;
        seg.mergedSegmentIds.clear();
        seg.mergedSegmentIds.append(seg.id);
        segments.append(seg);
    }

    return segments;
}

// ============================ LINEAR FIT SEGMENTATION ============================

QVector<SegmentInfo> ProfileAnalyzer::extractSegmentsWithLinearFit(const QVector<double>& x,
                                                                   const QVector<double>& z,
                                                                   const LinearFitParams& params) {
    // Sliding window linear fit segmentation for Lines table
    QVector<SegmentInfo> segments;
    if (x.size() < 2 || z.size() < 2 || x.size() != z.size()) return segments;

    const int minPointsPerSegment = params.minPointsPerSegment;
    const double residualTolerance = params.residualTolerance;

    int start = 0;
    for (int i = 1; i < x.size(); ++i) {
        if (i - start + 1 < minPointsPerSegment) continue;

        auto fit = linearFit(x, z, start, i);
        double slope = fit.first;
        double intercept = fit.second;

        // Calculate RMS error of current fit
        double sumSq = 0.0;
        int n = i - start + 1;
        for (int k = start; k <= i; ++k) {
            double pred = slope * x[k] + intercept;
            double resid = z[k] - pred;
            sumSq += resid * resid;
        }
        double rms = (n > 0) ? std::sqrt(sumSq / n) : std::numeric_limits<double>::infinity();

        // If error exceeds tolerance, create segment from start to i-1
        if (rms > residualTolerance) {
            int end = i - 1;
            if (end - start + 1 >= minPointsPerSegment) {
                SegmentInfo seg;
                seg.id = static_cast<int>(segments.size() + 1);
                seg.startX = x[start];
                seg.endX = x[end];
                seg.startZ = z[start];
                seg.endZ = z[end];

                double dx = seg.endX - seg.startX;
                double dz = seg.endZ - seg.startZ;
                seg.length = std::sqrt(dx*dx + dz*dz);
                seg.angle = calculateAngle(dx, dz);
                seg.type = classifySegment(seg.angle);
                seg.pointCount = end - start + 1;
                seg.isLongLine = false;
                seg.mergedSegmentIds.clear();
                seg.mergedSegmentIds.append(seg.id);

                segments.append(seg);
            }
            start = i - 1; // Reset window
        }
    }

    // Handle final segment
    if (x.size() - start >= minPointsPerSegment) {
        int end = x.size() - 1;
        SegmentInfo seg;
        seg.id = static_cast<int>(segments.size() + 1);
        seg.startX = x[start];
        seg.endX = x[end];
        seg.startZ = z[start];
        seg.endZ = z[end];

        double dx = seg.endX - seg.startX;
        double dz = seg.endZ - seg.startZ;
        seg.length = std::sqrt(dx*dx + dz*dz);
        seg.angle = calculateAngle(dx, dz);
        seg.type = classifySegment(seg.angle);
        seg.pointCount = end - start + 1;
        seg.isLongLine = false;
        seg.mergedSegmentIds.clear();
        seg.mergedSegmentIds.append(seg.id);

        segments.append(seg);
    }

    return segments;
}

// ============================ LINE MERGING ALGORITHMS ============================

QVector<SegmentInfo> ProfileAnalyzer::mergeLines(const QVector<SegmentInfo>& segments,
                                                 double angleTolerance,
                                                 double distanceTolerance) {
    // Merge collinear and proximate segments into longer lines
    QVector<SegmentInfo> mergedLines;
    if (segments.isEmpty()) return mergedLines;

    QVector<SegmentInfo> work = segments;
    QVector<bool> used(work.size(), false);

    for (int i = 0; i < work.size(); ++i) {
        if (used[i]) continue;

        // Step 1: Collect segments with similar angles
        QVector<int> clusterIdx;
        clusterIdx.append(i);
        used[i] = true;

        for (int j = i + 1; j < work.size(); ++j) {
            if (used[j]) continue;

            double angle1 = work[i].angle;
            double angle2 = work[j].angle;
            double normAngle1 = normalizeAngle180(angle1);
            double normAngle2 = normalizeAngle180(angle2);

            // Handle angle wrap-around (179° ≈ 1°)
            double ad = std::abs(normAngle1 - normAngle2);
            if (ad > 90.0) ad = 180.0 - ad;

            if (ad <= angleTolerance) {
                clusterIdx.append(j);
                used[j] = true;
            }
        }

        // Special handling for vertical segments (80°-100°)
        if (clusterIdx.size() == 1) {
            const auto& seg = work[clusterIdx[0]];
            double absAngle = std::abs(seg.angle);
            bool isVertical = (absAngle > 80.0 && absAngle < 100.0);

            if (isVertical) {
                // Merge all vertical segments with similar angles
                for (int j = i + 1; j < work.size(); ++j) {
                    if (used[j]) continue;

                    const auto& seg2 = work[j];
                    double absAngle2 = std::abs(seg2.angle);
                    bool isVertical2 = (absAngle2 > 80.0 && absAngle2 < 100.0);

                    if (isVertical2) {
                        double ad = angleDifference(seg.angle, seg2.angle);
                        if (ad <= 10.0) { // Wider tolerance for verticals
                            clusterIdx.append(j);
                            used[j] = true;
                        }
                    }
                }
            }
        }

        // Step 2: Compute average direction for cluster
        double avgAngle = 0.0;
        for (int idx : clusterIdx) avgAngle += work[idx].angle;
        avgAngle /= double(clusterIdx.size());

        double angRad = avgAngle * M_PI / 180.0;
        double ux = std::cos(angRad);
        double uz = std::sin(angRad);
        double mag = std::sqrt(ux*ux + uz*uz);
        if (mag > 0.0) { ux /= mag; uz /= mag; }

        // Step 3: Project segments onto average direction
        double refX = work[clusterIdx[0]].startX;
        double refZ = work[clusterIdx[0]].startZ;

        struct ProjectedSegment {
            int idx;
            double centerProj;
            double startProj;
            double endProj;
        };
        QVector<ProjectedSegment> projected;
        projected.reserve(clusterIdx.size());

        for (int idx : clusterIdx) {
            const auto& s = work[idx];
            double sP = projectScalarAlong(s.startX, s.startZ, refX, refZ, ux, uz);
            double eP = projectScalarAlong(s.endX, s.endZ, refX, refZ, ux, uz);
            double center = 0.5 * (sP + eP);
            projected.append({idx, center, std::min(sP,eP), std::max(sP,eP)});
        }

        // Sort by projection along direction
        std::sort(projected.begin(), projected.end(),
                  [](const ProjectedSegment& a, const ProjectedSegment& b){
                      return a.centerProj < b.centerProj;
                  });

        // Step 4: Merge contiguous segments
        QVector<bool> consumed(projected.size(), false);
        for (int a = 0; a < projected.size(); ++a) {
            if (consumed[a]) continue;

            SegmentInfo current = work[projected[a].idx];

            for (int b = a + 1; b < projected.size(); ++b) {
                if (consumed[b]) continue;

                const SegmentInfo& next = work[projected[b].idx];

                // Calculate gap along direction
                double currentEndProj = projectScalarAlong(current.endX, current.endZ, refX, refZ, ux, uz);
                double nextStartProj  = projectScalarAlong(next.startX, next.startZ, refX, refZ, ux, uz);
                double gapAlong = nextStartProj - currentEndProj;

                // Handle overlapping segments
                if (gapAlong < 0.0) {
                    double currentStartProj = projectScalarAlong(current.startX, current.startZ, refX, refZ, ux, uz);
                    double nextEndProj = projectScalarAlong(next.endX, next.endZ, refX, refZ, ux, uz);

                    if (nextEndProj > currentStartProj) {
                        gapAlong = 0.0; // Overlap
                    } else {
                        gapAlong = std::abs(gapAlong); // Reverse order
                    }
                }

                // Calculate perpendicular distance
                double midXcurr = 0.5*(current.startX + current.endX);
                double midZcurr = 0.5*(current.startZ + current.endZ);
                double midXnext = 0.5*(next.startX + next.endX);
                double midZnext = 0.5*(next.startZ + next.endZ);
                double perp = perpDistance(midXnext, midZnext, midXcurr, midZcurr, ux, uz);

                // Adjust tolerance for short segments
                double adjustedDistanceTolerance = distanceTolerance;
                if (current.length < 5.0 || next.length < 5.0) {
                    adjustedDistanceTolerance = distanceTolerance * 2.0;
                }

                // Merge if gaps are within tolerance
                if (gapAlong <= adjustedDistanceTolerance * 3.0 && perp <= adjustedDistanceTolerance * 1.5) {
                    // Extend current segment to encompass next
                    double sProjCur = projectScalarAlong(current.startX, current.startZ, refX, refZ, ux, uz);
                    double eProjCur = projectScalarAlong(current.endX, current.endZ, refX, refZ, ux, uz);
                    double sProjNext = projectScalarAlong(next.startX, next.startZ, refX, refZ, ux, uz);
                    double eProjNext = projectScalarAlong(next.endX, next.endZ, refX, refZ, ux, uz);

                    if (sProjNext < sProjCur) {
                        current.startX = next.startX;
                        current.startZ = next.startZ;
                    }
                    if (eProjNext > eProjCur) {
                        current.endX = next.endX;
                        current.endZ = next.endZ;
                    }

                    // Merge metadata
                    for (int id : next.mergedSegmentIds) current.mergedSegmentIds.append(id);
                    current.pointCount += next.pointCount;

                    // Recalculate geometry
                    double dx = current.endX - current.startX;
                    double dz = current.endZ - current.startZ;
                    current.length = std::sqrt(dx*dx + dz*dz);
                    current.angle = calculateAngle(dx, dz);
                    current.type = classifySegment(current.angle);

                    consumed[b] = true;
                } else {
                    if (gapAlong > adjustedDistanceTolerance * 5.0) break;
                }
            }

            current.isLongLine = (current.mergedSegmentIds.size() > 1 || current.length > 5.0);
            mergedLines.append(current);
        }
    }

    // Step 5: Additional pass for close parallel lines
    bool merged;
    do {
        merged = false;
        for (int i = 0; i < mergedLines.size(); ++i) {
            for (int j = i + 1; j < mergedLines.size(); ++j) {
                double ad = angleDifference(mergedLines[i].angle, mergedLines[j].angle);

                // Check for near-parallel lines
                if (ad <= 5.0) { // Increased tolerance for sloped lines
                    double dist = distanceBetweenLines(mergedLines[i], mergedLines[j]);

                    // Verify line directions are meaningful
                    double dx1 = mergedLines[i].endX - mergedLines[i].startX;
                    double dz1 = mergedLines[i].endZ - mergedLines[i].startZ;
                    double dx2 = mergedLines[j].endX - mergedLines[j].startX;
                    double dz2 = mergedLines[j].endZ - mergedLines[j].startZ;
                    double len1 = std::sqrt(dx1*dx1 + dz1*dz1);
                    double len2 = std::sqrt(dx2*dx2 + dz2*dz2);

                    if (len1 > 0 && len2 > 0) {
                        // Merge if lines are close and parallel
                        if (dist <= distanceTolerance * 2.0) {
                            // Merge extents
                            mergedLines[i].startX = std::min(mergedLines[i].startX, mergedLines[j].startX);
                            mergedLines[i].startZ = std::min(mergedLines[i].startZ, mergedLines[j].startZ);
                            mergedLines[i].endX = std::max(mergedLines[i].endX, mergedLines[j].endX);
                            mergedLines[i].endZ = std::max(mergedLines[i].endZ, mergedLines[j].endZ);

                            // Update geometry
                            double dx = mergedLines[i].endX - mergedLines[i].startX;
                            double dz = mergedLines[i].endZ - mergedLines[i].startZ;
                            mergedLines[i].length = std::sqrt(dx*dx + dz*dz);
                            mergedLines[i].angle = calculateAngle(dx, dz);
                            mergedLines[i].type = classifySegment(mergedLines[i].angle);

                            // Merge segment IDs
                            for (int id : mergedLines[j].mergedSegmentIds) {
                                mergedLines[i].mergedSegmentIds.append(id);
                            }
                            mergedLines[i].pointCount += mergedLines[j].pointCount;
                            mergedLines[i].isLongLine = true;

                            // Remove merged line
                            mergedLines.removeAt(j);
                            merged = true;
                            break;
                        }
                    }
                }
                if (merged) break;
            }
            if (merged) break;
        }
    } while (merged && mergedLines.size() > 1);

    // Renumber IDs
    for (int i = 0; i < mergedLines.size(); ++i) mergedLines[i].id = i + 1;

    return mergedLines;
}

// ============================ HOUGH TRANSFORM IMPLEMENTATION ============================

QVector<SegmentInfo> ProfileAnalyzer::extractSegmentsWithHough(const QVector<double>& x,
                                                               const QVector<double>& z,
                                                               const HoughParams& params) {
    // Hough transform line detection with parameter space clustering
    QVector<SegmentInfo> segments;
    if (x.size() < 10) return segments;

    // 1. NORMALIZATION (critical for stable Hough transform)
    double xMin = *std::min_element(x.begin(), x.end());
    double xMax = *std::max_element(x.begin(), x.end());
    double zMin = *std::min_element(z.begin(), z.end());
    double zMax = *std::max_element(z.begin(), z.end());

    double xCenter = (xMin + xMax) / 2.0;
    double zCenter = (zMin + zMax) / 2.0;
    double diag = std::hypot(xMax - xMin, zMax - zMin);

    // 2. PARAMETER SPACE CONFIGURATION
    const double dTheta = params.dTheta;
    const int thetaBins = params.thetaBins;
    const double maxRho = diag / 2.0 + 5.0; // Add 10mm margin
    const double dRho = params.dRho;
    const int rhoBins = static_cast<int>(2.0 * maxRho / dRho) + 1;

    // Dynamic voting threshold
    const int minVotes = qMax(params.minPointsPerLine,
                              static_cast<int>(x.size() * params.minVotesPercent / 100.0));
    const int minPointsForLine = params.minPointsPerLine;

    // Gap tolerance for connecting points
    const double avgSpacing = (xMax - xMin) / x.size();
    const double maxGap = avgSpacing * 3.0;

    // 3. ACCUMULATOR ARRAY (with blurring)
    QVector<QVector<int>> accumulator(thetaBins, QVector<int>(rhoBins, 0));

    // Vote for each point in parameter space with 3×3 blur
    for (int i = 0; i < x.size(); ++i) {
        double xi = x[i] - xCenter;
        double zi = z[i] - zCenter;

        for (int t = 0; t < thetaBins; ++t) {
            double thetaRad = (t * dTheta) * M_PI / 180.0;
            double rhoExact = xi * std::cos(thetaRad) + zi * std::sin(thetaRad);

            int rCenter = static_cast<int>((rhoExact + maxRho) / dRho);

            // Blur voting (±1 cell)
            for (int dr = -1; dr <= 1; ++dr) {
                int rIdx = rCenter + dr;
                if (rIdx >= 0 && rIdx < rhoBins) {
                    accumulator[t][rIdx] += (dr == 0) ? 2 : 1; // Center gets double weight
                }
            }
        }
    }

    // 4. PEAK DETECTION (non-maximum suppression)
    QVector<QPair<int, int>> peaks;
    const int minPeakDistance = 3; // cells

    for (int t = 1; t < thetaBins - 1; ++t) {
        for (int r = 1; r < rhoBins - 1; ++r) {
            int votes = accumulator[t][r];
            if (votes < minVotes) continue;

            // Check 3×3 neighborhood for local maximum
            bool isPeak = true;
            for (int dt = -1; dt <= 1 && isPeak; ++dt) {
                for (int dr = -1; dr <= 1; ++dr) {
                    if (accumulator[t + dt][r + dr] > votes) {
                        isPeak = false;
                        break;
                    }
                }
            }

            if (!isPeak) continue;

            // Ensure sufficient distance from existing peaks
            bool tooClose = false;
            for (const auto& p : peaks) {
                int dt = t - p.first;
                int dr = r - p.second;
                if (std::hypot(dt, dr) < minPeakDistance) {
                    tooClose = true;
                    break;
                }
            }

            if (!tooClose) {
                peaks.append({t, r});
            }
        }
    }

    // Sort peaks by vote count (strongest first)
    std::sort(peaks.begin(), peaks.end(),
              [&](const QPair<int, int>& a, const QPair<int, int>& b) {
                  return accumulator[a.first][a.second] > accumulator[b.first][b.second];
              });

    // Limit number of lines but keep reasonable count
    int maxLines = qMin(15, peaks.size());

    // 5. LINE EXTRACTION FROM PEAKS
    for (int lineIdx = 0; lineIdx < maxLines; ++lineIdx) {
        int tIdx = peaks[lineIdx].first;
        int rIdx = peaks[lineIdx].second;

        double theta = tIdx * dTheta;
        double thetaRad = theta * M_PI / 180.0;
        double rho = rIdx * dRho - maxRho;

        double ux = std::cos(thetaRad);
        double uz = std::sin(thetaRad);

        // Collect points within ±2*dRho band
        QVector<QPointF> linePoints;
        QVector<int> pointIndices;

        for (int i = 0; i < x.size(); ++i) {
            double xi = x[i] - xCenter;
            double zi = z[i] - zCenter;
            double dist = std::abs(xi * ux + zi * uz - rho);

            if (dist <= dRho * 2.0) { // 0.4mm band
                linePoints.append(QPointF(x[i], z[i]));
                pointIndices.append(i);
            }
        }

        if (linePoints.size() < static_cast<size_t>(minPointsForLine)) continue;

        // Sort by projection onto line direction
        std::sort(linePoints.begin(), linePoints.end(),
                  [ux, uz](const QPointF& a, const QPointF& b) {
                      return a.x() * ux + a.y() * uz < b.x() * ux + b.y() * uz;
                  });

        // 6. CHAINING: Connect points with small gaps
        QVector<QPointF> chain;
        chain.append(linePoints.first());

        for (int i = 1; i < linePoints.size(); ++i) {
            double dist = std::hypot(linePoints[i].x() - chain.last().x(),
                                     linePoints[i].y() - chain.last().y());

            if (dist <= maxGap) {
                chain.append(linePoints[i]);
            } else {
                // Process completed chain
                if (chain.size() >= static_cast<size_t>(minPointsForLine)) {
                    SegmentInfo seg;
                    seg.startX = chain.first().x();
                    seg.startZ = chain.first().y();
                    seg.endX = chain.last().x();
                    seg.endZ = chain.last().y();

                    double dx = seg.endX - seg.startX;
                    double dz = seg.endZ - seg.startZ;
                    seg.length = std::hypot(dx, dz);
                    seg.angle = calculateAngle(dx, dz);
                    seg.type = classifySegment(seg.angle);
                    seg.pointCount = chain.size();
                    seg.isLongLine = (seg.length > 5.0);
                    seg.mergedSegmentIds.append(segments.size() + 1);

                    segments.append(seg);
                }
                chain.clear();
                chain.append(linePoints[i]);
            }
        }

        // Process final chain
        if (chain.size() >= static_cast<size_t>(minPointsForLine)) {
            SegmentInfo seg;
            seg.startX = chain.first().x();
            seg.startZ = chain.first().y();
            seg.endX = chain.last().x();
            seg.endZ = chain.last().y();

            double dx = seg.endX - seg.startX;
            double dz = seg.endZ - seg.startZ;
            seg.length = std::hypot(dx, dz);
            seg.angle = calculateAngle(dx, dz);
            seg.type = classifySegment(seg.angle);
            seg.pointCount = chain.size();
            seg.isLongLine = (seg.length > 5.0);
            seg.mergedSegmentIds.append(segments.size() + 1);

            segments.append(seg);
        }
    }

    return segments;
}

// ============================ HOUGH MERGING ============================

QVector<SegmentInfo> ProfileAnalyzer::mergeHoughLines(const QVector<SegmentInfo>& segments,
                                                      double angleTolerance,
                                                      double distanceTolerance) {
    // Simplified merging for Hough-detected lines
    if (segments.isEmpty()) return segments;

    QVector<SegmentInfo> merged = segments;
    bool changed = true;
    int iteration = 0;
    const int maxIterations = 10; // Prevent infinite loops

    while (changed && iteration < maxIterations) {
        changed = false;
        iteration++;

        for (int i = 0; i < merged.size(); ++i) {
            for (int j = i + 1; j < merged.size(); ++j) {
                double ad = angleDifference(merged[i].angle, merged[j].angle);

                if (ad <= angleTolerance) {
                    // Check for overlap or proximity
                    double overlap = calculateOverlap(merged[i], merged[j]);
                    double dist = distanceBetweenLines(merged[i], merged[j]);

                    if (overlap > 0.5 || dist <= distanceTolerance) { // 50% overlap threshold
                        // Merge the two lines
                        SegmentInfo combined;
                        combined.startX = std::min(merged[i].startX, merged[j].startX);
                        combined.startZ = std::min(merged[i].startZ, merged[j].startZ);
                        combined.endX = std::max(merged[i].endX, merged[j].endX);
                        combined.endZ = std::max(merged[i].endZ, merged[j].endZ);

                        double dx = combined.endX - combined.startX;
                        double dz = combined.endZ - combined.startZ;
                        combined.length = std::hypot(dx, dz);
                        combined.angle = calculateAngle(dx, dz);
                        combined.type = classifySegment(combined.angle);
                        combined.pointCount = merged[i].pointCount + merged[j].pointCount;
                        combined.isLongLine = true;
                        combined.mergedSegmentIds = merged[i].mergedSegmentIds + merged[j].mergedSegmentIds;

                        merged[i] = combined;
                        merged.removeAt(j);
                        changed = true;
                        break;
                    }
                }
            }
            if (changed) break;
        }
    }

    // Renumber IDs
    for (int i = 0; i < merged.size(); ++i) {
        merged[i].id = i + 1;
    }

    return merged;
}

// ============================ RANSAC IMPLEMENTATION ============================

QVector<SegmentInfo> ProfileAnalyzer::extractSegmentsWithRANSAC(const QVector<double>& x,
                                                                const QVector<double>& z,
                                                                const RANSACParams& params) {
    // RANSAC robust line fitting with iterative inlier removal
    QVector<SegmentInfo> detectedLines;
    if (x.size() < 3) return detectedLines;

    // Working copies with original indices
    QVector<double> remainingX = x;
    QVector<double> remainingZ = z;
    QVector<int> originalIndices;
    originalIndices.reserve(x.size());
    for (int i = 0; i < x.size(); ++i) originalIndices.append(i);

    const int maxIterations = params.maxIterations;
    const double distanceThreshold = params.distanceThreshold;
    const int minInliers = params.minInliers;
    const double minLineLength = params.minLineLength;
    const int maxLinesToFind = params.maxLinesToFind;

    // Iteratively find lines until no more good candidates
    while (detectedLines.size() < maxLinesToFind && remainingX.size() >= minInliers) {
        SegmentInfo bestLine;
        int bestInlierCount = 0;
        double bestError = std::numeric_limits<double>::max();
        QVector<int> bestInlierIndices;

        // RANSAC iteration loop
        for (int iter = 0; iter < maxIterations; ++iter) {
            // Randomly select two points
            int idx1 = rand() % remainingX.size();
            int idx2 = rand() % remainingX.size();
            if (idx1 == idx2) continue;

            double x1 = remainingX[idx1], z1 = remainingZ[idx1];
            double x2 = remainingX[idx2], z2 = remainingZ[idx2];

            // Ensure points are not too close
            double dist = std::hypot(x2 - x1, z2 - z1);
            if (dist < 0.5) continue; // Minimum 0.5mm separation

            // Calculate line parameters from two points
            double dx = x2 - x1;
            double dz = z2 - z1;
            double lineLength = std::hypot(dx, dz);
            double angle = calculateAngle(dx, dz);

            // Line normal (for distance calculation)
            double nx = -dz / lineLength;
            double nz = dx / lineLength;

            // Find inliers within distance threshold
            QVector<int> inlierIndices;
            double sumX = 0.0, sumZ = 0.0;
            int inlierCount = 0;

            for (int i = 0; i < remainingX.size(); ++i) {
                double d = std::abs((remainingX[i] - x1) * nx + (remainingZ[i] - z1) * nz);
                if (d <= distanceThreshold) {
                    inlierIndices.append(i);
                    sumX += remainingX[i];
                    sumZ += remainingZ[i];
                    inlierCount++;
                }
            }

            // Check if this model is better than current best
            if (inlierCount >= minInliers && inlierCount > bestInlierCount) {
                // Refine line using PCA on inliers
                double avgX = sumX / inlierCount;
                double avgZ = sumZ / inlierCount;

                // Compute covariance matrix for PCA
                double sumXX = 0.0, sumXZ = 0.0, sumZZ = 0.0;
                for (int idx : inlierIndices) {
                    double xi = remainingX[idx] - avgX;
                    double zi = remainingZ[idx] - avgZ;
                    sumXX += xi * xi;
                    sumXZ += xi * zi;
                    sumZZ += zi * zi;
                }

                // Principal component direction (largest eigenvector)
                double phi = 0.5 * std::atan2(2.0 * sumXZ, sumXX - sumZZ);
                double lineAngle = phi * 180.0 / M_PI;

                // Verify angle consistency
                double angleDiff = std::abs(angleDifference(angle, lineAngle));
                if (angleDiff > 30.0) continue; // Angles differ too much

                // Project points onto principal direction to find endpoints
                double ux = std::cos(phi);
                double uz = std::sin(phi);
                double minProj = std::numeric_limits<double>::max();
                double maxProj = std::numeric_limits<double>::lowest();
                QPointF projMin, projMax;

                for (int idx : inlierIndices) {
                    double proj = (remainingX[idx] - avgX) * ux + (remainingZ[idx] - avgZ) * uz;
                    if (proj < minProj) {
                        minProj = proj;
                        projMin = QPointF(remainingX[idx], remainingZ[idx]);
                    }
                    if (proj > maxProj) {
                        maxProj = proj;
                        projMax = QPointF(remainingX[idx], remainingZ[idx]);
                    }
                }

                double refinedLength = std::hypot(projMax.x() - projMin.x(), projMax.y() - projMin.y());

                // Check length requirement
                if (refinedLength >= minLineLength) {
                    bestInlierCount = inlierCount;
                    bestInlierIndices = inlierIndices;

                    // Store refined line
                    bestLine.startX = projMin.x();
                    bestLine.startZ = projMin.y();
                    bestLine.endX = projMax.x();
                    bestLine.endZ = projMax.y();
                    bestLine.length = refinedLength;
                    bestLine.angle = calculateAngle(projMax.x() - projMin.x(), projMax.y() - projMin.y());
                    bestLine.type = classifySegment(bestLine.angle);
                    bestLine.pointCount = inlierCount;
                    bestLine.isLongLine = (refinedLength > 5.0);
                }
            }
        }

        // If a good line was found, add it and remove inliers
        if (bestInlierCount >= minInliers) {
            // Record which original segments contributed
            for (int idx : bestInlierIndices) {
                bestLine.mergedSegmentIds.append(originalIndices[idx]);
            }

            detectedLines.append(bestLine);

            // Remove inliers from consideration (reverse order to preserve indices)
            std::sort(bestInlierIndices.rbegin(), bestInlierIndices.rend());
            for (int idx : bestInlierIndices) {
                remainingX.removeAt(idx);
                remainingZ.removeAt(idx);
                originalIndices.removeAt(idx);
            }
        } else {
            break; // No more lines found
        }
    }

    return detectedLines;
}

// ============================ PUBLIC API IMPLEMENTATIONS ============================

GeometryResult ProfileAnalyzer::analyzeGeometry(const QVector<ProfilePoint>& profile,
                                                const LinearFitParams& params) {
    GeometryResult result;

    // Extract valid points only
    QVector<double> x, z;
    x.reserve(profile.size());
    z.reserve(profile.size());
    for (int i = 0; i < profile.size(); ++i) {
        const ProfilePoint& point = profile[i];
        if (point.valid) {
            x.append(point.x);
            z.append(point.z);
        }
    }

    if (x.size() < 2) {
        result.status = "Недостаточно валидных точек";
        return result;
    }

    // Validate data (no NaN/Inf)
    for (int i = 0; i < x.size(); ++i) {
        if (std::isnan(x[i]) || std::isinf(x[i]) || std::isnan(z[i]) || std::isinf(z[i])) {
            result.status = "Обнаружены некорректные данные (NaN или Inf)";
            return result;
        }
    }

    try {
        // 1) Segments table: all micro-segments between adjacent points
        result.segments = extractMicroSegments(x, z);
        result.segmentCount = static_cast<int>(result.segments.size());

        // 2) Lines table: linear fit segmentation + merging
        QVector<SegmentInfo> longSegments = extractSegmentsWithLinearFit(x, z, params);
        result.longLines = mergeLines(longSegments, params.mergeAngleTolerance, params.mergeDistanceTolerance);
        result.lineCount = static_cast<int>(result.longLines.size());

        // 3) Sort lines by mean X coordinate
        std::sort(result.longLines.begin(), result.longLines.end(), [](const SegmentInfo& a, const SegmentInfo& b){
            double ax = 0.5 * (a.startX + a.endX);
            double bx = 0.5 * (b.startX + b.endX);
            return ax < bx;
        });

        // 4) Renumber IDs sequentially
        for (int i = 0; i < result.longLines.size(); ++i) {
            result.longLines[i].id = i + 1;
        }

        // 5) Calculate line statistics
        result.totalLength = 0.0;
        result.minLength = std::numeric_limits<double>::max();
        result.maxLength = 0.0;
        for (const auto& line : result.longLines) {
            result.totalLength += line.length;
            if (line.length < result.minLength) result.minLength = line.length;
            if (line.length > result.maxLength) result.maxLength = line.length;
        }
        if (result.longLines.isEmpty()) result.minLength = 0.0;

        result.status = QString("Найдено %1 микросегментов, объединено в %2 линий")
                            .arg(result.segmentCount)
                            .arg(result.lineCount);
    } catch (const std::exception& e) {
        result.status = QString("Ошибка анализа: %1").arg(e.what());
    }

    return result;
}

GeometryResult ProfileAnalyzer::analyzeGeometryHough(const QVector<ProfilePoint>& profile,
                                                     const HoughParams& params) {
    GeometryResult result;

    // Extract valid points
    QVector<double> x, z;
    x.reserve(profile.size());
    z.reserve(profile.size());
    for (int i = 0; i < profile.size(); ++i) {
        const ProfilePoint& point = profile[i];
        if (point.valid) {
            x.append(point.x);
            z.append(point.z);
        }
    }

    if (x.size() < 2) {
        result.status = "Недостаточно валидных точек";
        return result;
    }

    // Validate data
    for (int i = 0; i < x.size(); ++i) {
        if (std::isnan(x[i]) || std::isinf(x[i]) || std::isnan(z[i]) || std::isinf(z[i])) {
            result.status = "Обнаружены некорректные данные (NaN или Inf)";
            return result;
        }
    }

    try {
        // 1) Micro-segments (same as before)
        result.segments = extractMicroSegments(x, z);
        result.segmentCount = static_cast<int>(result.segments.size());

        // 2) Hough Transform line detection
        auto houghSegments = extractSegmentsWithHough(x, z, params);
        result.longLines = mergeHoughLines(houghSegments, params.mergeAngleTolerance, params.mergeDistanceTolerance);

        // 3) Sort lines by mean X
        std::sort(result.longLines.begin(), result.longLines.end(), [](const SegmentInfo& a, const SegmentInfo& b){
            double ax = 0.5 * (a.startX + a.endX);
            double bx = 0.5 * (b.startX + b.endX);
            return ax < bx;
        });

        // 4) Renumber IDs
        for (int i = 0; i < result.longLines.size(); ++i) {
            result.longLines[i].id = i + 1;
        }

        // 5) Statistics
        result.totalLength = 0.0;
        result.minLength = std::numeric_limits<double>::max();
        result.maxLength = 0.0;
        for (const auto& line : result.longLines) {
            result.totalLength += line.length;
            if (line.length < result.minLength) result.minLength = line.length;
            if (line.length > result.maxLength) result.maxLength = line.length;
        }
        if (result.longLines.isEmpty()) result.minLength = 0.0;

        result.status = QString("Hough: %1 микросегментов, %2 линий")
                            .arg(result.segmentCount)
                            .arg(result.longLines.size());

    } catch (const std::exception& e) {
        result.status = QString("Ошибка Hough: %1").arg(e.what());
    }

    return result;
}

GeometryResult ProfileAnalyzer::analyzeGeometryRANSAC(const QVector<ProfilePoint>& profile,
                                                      const RANSACParams& params) {
    GeometryResult result;

    // Extract valid points
    QVector<double> x, z;
    x.reserve(profile.size());
    z.reserve(profile.size());
    for (int i = 0; i < profile.size(); ++i) {
        const ProfilePoint& point = profile[i];
        if (point.valid) {
            x.append(point.x);
            z.append(point.z);
        }
    }

    if (x.size() < 2) {
        result.status = "Недостаточно валидных точек";
        return result;
    }

    // Validate data
    for (int i = 0; i < x.size(); ++i) {
        if (std::isnan(x[i]) || std::isinf(x[i]) || std::isnan(z[i]) || std::isinf(z[i])) {
            result.status = "Обнаружены некорректные данные (NaN или Inf)";
            return result;
        }
    }

    try {
        // 1) Micro-segments
        result.segments = extractMicroSegments(x, z);
        result.segmentCount = static_cast<int>(result.segments.size());

        // 2) RANSAC line detection
        auto ransacSegments = extractSegmentsWithRANSAC(x, z, params);
        result.longLines = mergeLines(ransacSegments, params.mergeAngleTolerance, params.distanceThreshold * 2.0);
        result.lineCount = static_cast<int>(result.longLines.size());

        // 3) Sort lines by mean X
        std::sort(result.longLines.begin(), result.longLines.end(), [](const SegmentInfo& a, const SegmentInfo& b){
            double ax = 0.5 * (a.startX + a.endX);
            double bx = 0.5 * (b.startX + b.endX);
            return ax < bx;
        });

        // 4) Renumber IDs
        for (int i = 0; i < result.longLines.size(); ++i) {
            result.longLines[i].id = i + 1;
        }

        // 5) Statistics
        result.totalLength = 0.0;
        result.minLength = std::numeric_limits<double>::max();
        result.maxLength = 0.0;
        for (const auto& line : result.longLines) {
            result.totalLength += line.length;
            if (line.length < result.minLength) result.minLength = line.length;
            if (line.length > result.maxLength) result.maxLength = line.length;
        }
        if (result.longLines.isEmpty()) result.minLength = 0.0;

        result.status = QString("RANSAC: %1 микросегментов, %2 линий")
                            .arg(result.segmentCount)
                            .arg(result.lineCount);

    } catch (const std::exception& e) {
        result.status = QString("Ошибка RANSAC: %1").arg(e.what());
    }

    return result;
}
