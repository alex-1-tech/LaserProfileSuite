#pragma once

/**
 * @file    plotcontroller.h
 * @brief   Controller class for managing profile visualization on QCustomPlot
 * @author  alex-1-tech
 * @date    2025
 */

#include <QObject>
#include <QVector>
#include <QPointF>
#include "qcustomplot.h"
#include "segmentinfo.h"

/**
 * @class   PlotController
 * @brief   Manages profile visualization and interactive elements on QCustomPlot
 *
 * This controller handles:
 * - Profile data visualization (valid/invalid points)
 * - Segment highlighting and labeling
 * - Distance measurements display
 * - Multiple display modes (points/lines)
 */
class PlotController : public QObject
{
    Q_OBJECT

public:
    explicit PlotController(QCustomPlot* plot, QObject* parent = nullptr);  ///< Constructor
    ~PlotController();  ///< Destructor

    void updateProfile(const QVector<ProfilePoint>& profile);  ///< Update plot with new profile data
    void setDisplayMode(int mode);  ///< Set display mode (0 - points, 1 - lines)
    void highlightSegment(const SegmentInfo& segment);  ///< Highlight specific segment
    void highlightLine(const SegmentInfo& line);  ///< Highlight specific line
    void showDistances(const QVector<DistanceInfo>& distances);  ///< Display distance measurements
    void clearHighlight();  ///< Clear all highlights
    void clearDistances();  ///< Clear distance displays
    void resetPlot();  ///< Reset plot to initial state

signals:
    void profileUpdated(int validPoints, int totalPoints);  ///< Emitted when profile data is updated
    void segmentSelected(int segmentId);  ///< Emitted when segment is selected
    void segmentSelectionCleared();  ///< Emitted when segment selection is cleared

private:
    /// @brief Data structure for graph coordinates
    struct GraphData {
        QVector<double> x;
        QVector<double> z;

        GraphData() = default;
        GraphData(const QVector<double>& x, const QVector<double>& z) : x(x), z(z) {}
        bool isEmpty() const { return x.isEmpty() || z.isEmpty(); }  ///< Check if data is empty
    };

    /// @brief Container for split profile data
    struct ProfileData {
        GraphData validPoints;
        GraphData invalidPoints;

        ProfileData() = default;
        ProfileData(const GraphData& valid, const GraphData& invalid)
            : validPoints(valid), invalidPoints(invalid) {}
    };

    /// @brief Display mode enumeration
    enum class DisplayMode {
        Points = 0,  ///< Display as individual points
        Lines = 1    ///< Display as connected lines
    };

private:
    void setupPlot();  ///< Initialize plot configuration
    void initializeHighlightItems();  ///< Create highlight UI elements
    void cleanup();  ///< Clean up resources

    ProfileData splitProfileData(const QVector<ProfilePoint>& profile);  ///< Split profile into valid/invalid points
    void updateMainGraph(const GraphData& data);  ///< Update main graph with valid data
    void updateInvalidGraph(const GraphData& data);  ///< Update invalid points graph

    void updateGraphStyle();  ///< Update all graph styles
    void updateMainGraphStyle();  ///< Update main graph style
    void updateInvalidGraphStyle();  ///< Update invalid graph style
    void updateSegmentGraphStyle();  ///< Update segment highlight style
    void updateLineGraphsStyle();  ///< Update line graphs style
    void updateDistanceLinesStyle();  ///< Update distance lines style

    void updateSegmentDisplay(const SegmentInfo& segment);  ///< Display segment highlight
    QString formatSegmentLabel(const SegmentInfo& segment) const;  ///< Format segment label text

    void removeHighlightedLineGraph();  ///< Remove highlighted line graph
    void createHighlightedLineGraph(const SegmentInfo& line);  ///< Create highlighted line graph
    GraphData prepareLineData(const SegmentInfo& line);  ///< Prepare line data for display

    void createDistanceLine(const DistanceInfo& dist);  ///< Create distance line element
    void createDistanceLabel(const DistanceInfo& dist);  ///< Create distance label

    void hideAllHighlights();  ///< Hide all highlight elements
    void clearDistanceLines();  ///< Clear distance lines
    void clearDistanceLabels();  ///< Clear distance labels
    void clearLineGraphs();  ///< Clear line graphs

private:
    QCustomPlot* m_plot;  ///< Pointer to the plot widget (non-owning)

    DisplayMode m_displayMode;  ///< Current display mode

    QCPGraph* m_mainGraph;      ///< Main graph for valid points
    QCPGraph* m_invalidGraph;   ///< Graph for invalid points

    QCPGraph* m_segmentGraph;   ///< Graph for highlighted segment
    QCPItemLine* m_segmentLine; ///< Segment approximation line
    QCPItemText* m_segmentLabel;///< Segment label

    QVector<QCPGraph*> m_lineGraphs;          ///< All line graphs
    QCPGraph* m_highlightedLineGraph;         ///< Currently highlighted line

    QVector<QCPItemLine*> m_distanceLines;    ///< Distance measurement lines
    QVector<QCPItemText*> m_distanceLabels;   ///< Distance labels
};
