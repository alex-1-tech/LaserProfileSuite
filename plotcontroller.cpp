#include "plotcontroller.h"
#include "qcustomplot.h"
#include <QFont>
#include <QPen>
#include <QDebug>

PlotController::PlotController(QCustomPlot* plot, QObject* parent)
    : QObject(parent)
    , m_plot(plot)
    , m_displayMode(DisplayMode::Lines)
    , m_mainGraph(nullptr)
    , m_invalidGraph(nullptr)
{
    Q_ASSERT(plot != nullptr);
    setupPlot();
}

PlotController::~PlotController()
{
    cleanup();
}

void PlotController::cleanup()
{
    // Удаляем все созданные элементы
    clearDistances();
    removeAllSegmentGraphs();
    removeAllLineGraphs();

    if (m_mainGraph) {
        m_plot->removeGraph(m_mainGraph);
        m_mainGraph = nullptr;
    }

    if (m_invalidGraph) {
        m_plot->removeGraph(m_invalidGraph);
        m_invalidGraph = nullptr;
    }
}

void PlotController::setupPlot()
{
    if (!m_plot) {
        qWarning() << "PlotController: plot is null!";
        return;
    }

    m_plot->xAxis->setLabel("X (мм)");
    m_plot->yAxis->setLabel("Z (мм)");

    m_plot->xAxis->setRange(-300, 300);
    m_plot->yAxis->setRange(-100, 100);

    m_plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);
    m_plot->axisRect()->setupFullAxesBox();

    // Оптимизация производительности
    m_plot->setNotAntialiasedElement(QCP::aeAll);
    m_plot->setNoAntialiasingOnDrag(true);
}

void PlotController::updateProfile(const QVector<ProfilePoint>& profile)
{
    if (profile.isEmpty()) {
        qWarning() << "PlotController: Empty profile data";
        return;
    }

    clearDistances();
    removeAllSegmentGraphs();
    removeAllLineGraphs();

    ProfileData data = splitProfileData(profile);
    updateMainGraph(data.validPoints);
    updateInvalidGraph(data.invalidPoints);

    updateGraphStyle();
    m_plot->replot();

    emit profileUpdated(data.validPoints.x.size(), profile.size());
}

PlotController::ProfileData PlotController::splitProfileData(const QVector<ProfilePoint>& profile)
{
    ProfileData data;
    data.validPoints.x.reserve(profile.size());
    data.validPoints.z.reserve(profile.size());
    data.invalidPoints.x.reserve(profile.size());

    for (const ProfilePoint& p : profile) {
        if (p.valid) {
            data.validPoints.x.append(p.x);
            data.validPoints.z.append(p.z);
        } else {
            data.invalidPoints.x.append(p.x);
            data.invalidPoints.z.append(p.z);
        }
    }

    return data;
}

void PlotController::updateMainGraph(const GraphData& data)
{
    if (!m_mainGraph) {
        m_mainGraph = m_plot->addGraph();
        m_mainGraph->setName("main_profile");
    }
    m_mainGraph->setData(data.x, data.z);
}

void PlotController::updateInvalidGraph(const GraphData& data)
{
    if (data.x.isEmpty()) {
        if (m_invalidGraph) {
            m_invalidGraph->setVisible(false);
        }
        return;
    }

    if (!m_invalidGraph) {
        m_invalidGraph = m_plot->addGraph();
        m_invalidGraph->setName("invalid_points");
    }

    m_invalidGraph->setData(data.x, data.z);
    m_invalidGraph->setVisible(true);
}

void PlotController::setDisplayMode(int mode)
{
    if (mode < 0 || mode > 1) {
        qWarning() << "PlotController: Invalid display mode:" << mode;
        return;
    }

    m_displayMode = static_cast<DisplayMode>(mode);
    updateGraphStyle();
    m_plot->replot();
}

void PlotController::updateGraphStyle()
{
    updateMainGraphStyle();
    updateInvalidGraphStyle();
    updateSegmentGraphStyle();
    updateLineGraphStyle();
    updateDistanceLinesStyle();
}

void PlotController::updateMainGraphStyle()
{
    if (!m_mainGraph) return;

    switch (m_displayMode) {
    case DisplayMode::Points:
        m_mainGraph->setLineStyle(QCPGraph::lsNone);
        m_mainGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCircle, 3, Qt::blue));
        break;
    case DisplayMode::Lines:
    default:
        m_mainGraph->setLineStyle(QCPGraph::lsLine);
        m_mainGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssNone));
        m_mainGraph->setPen(QPen(Qt::blue, 1.5));
        break;
    }
}

void PlotController::updateInvalidGraphStyle()
{
    if (m_invalidGraph) {
        m_invalidGraph->setLineStyle(QCPGraph::lsNone);
        m_invalidGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssCross, Qt::red, 4));
    }
}

void PlotController::updateSegmentGraphStyle()
{
    // Все сегменты одного цвета - желтый
    for (QCPGraph* segmentGraph : m_segmentGraphs) {
        if (segmentGraph) {
            segmentGraph->setLineStyle(QCPGraph::lsLine);
            segmentGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, Qt::yellow, 6));
            segmentGraph->setPen(QPen(Qt::yellow, 2));
        }
    }
}

void PlotController::updateLineGraphStyle()
{
    // Все линии одного цвета - желтый
    for (QCPGraph* lineGraph : m_lineGraphs) {
        if (lineGraph) {
            lineGraph->setLineStyle(QCPGraph::lsLine);
            lineGraph->setPen(QPen(Qt::yellow, 3));
        }
    }
}

void PlotController::updateDistanceLinesStyle()
{
    for (QCPItemLine* distLine : m_distanceLines) {
        if (distLine) {
            distLine->setPen(QPen(Qt::magenta, 2, Qt::DashLine));
        }
    }
}

void PlotController::highlightSegments(const QVector<SegmentInfo>& segments)
{
    removeAllSegmentGraphs();
    createSegmentGraphs(segments);
    updateSegmentGraphStyle();
    m_plot->replot();
}

void PlotController::createSegmentGraphs(const QVector<SegmentInfo>& segments)
{
    for (const SegmentInfo& segment : segments) {
        QVector<double> segX = QVector<double>() << segment.startX << segment.endX;
        QVector<double> segZ = QVector<double>() << segment.startZ << segment.endZ;

        QCPGraph* segmentGraph = m_plot->addGraph();
        segmentGraph->setData(segX, segZ);
        segmentGraph->setVisible(true);

        m_segmentGraphs.append(segmentGraph);
    }
}

void PlotController::highlightLines(const QVector<SegmentInfo>& lines)
{
    removeAllLineGraphs();
    createLineGraphs(lines);
    updateLineGraphStyle();
    m_plot->replot();
}

void PlotController::createLineGraphs(const QVector<SegmentInfo>& lines)
{
    for (const SegmentInfo& line : lines) {
        QCPGraph* lineGraph = m_plot->addGraph();

        GraphData lineData = prepareLineData(line);
        lineGraph->setData(lineData.x, lineData.z);
        lineGraph->setVisible(true);

        m_lineGraphs.append(lineGraph);
    }
}

PlotController::GraphData PlotController::prepareLineData(const SegmentInfo& line)
{
    GraphData data;

    if (!line.points.isEmpty()) {
        data.x.reserve(line.points.size());
        data.z.reserve(line.points.size());

        for (const QPointF& point : line.points) {
            data.x.append(point.x());
            data.z.append(point.y());
        }
    } else {
        data.x = QVector<double>() << line.startX << line.endX;
        data.z = QVector<double>() << line.startZ << line.endZ;
    }

    return data;
}

void PlotController::removeAllSegmentGraphs()
{
    for (QCPGraph* segmentGraph : m_segmentGraphs) {
        if (segmentGraph) {
            m_plot->removeGraph(segmentGraph);
        }
    }
    m_segmentGraphs.clear();
}

void PlotController::removeAllLineGraphs()
{
    for (QCPGraph* lineGraph : m_lineGraphs) {
        if (lineGraph) {
            m_plot->removeGraph(lineGraph);
        }
    }
    m_lineGraphs.clear();
}

void PlotController::showDistances(const QVector<DistanceInfo>& distances)
{
    clearDistances();

    for (const DistanceInfo& dist : distances) {
        createDistanceLine(dist);
        createDistanceLabel(dist);
    }

    m_plot->replot();
}

void PlotController::createDistanceLine(const DistanceInfo& dist)
{
    QCPItemLine* distLine = new QCPItemLine(m_plot);
    distLine->start->setCoords(dist.measurementStart.x(), dist.measurementStart.y());
    distLine->end->setCoords(dist.measurementEnd.x(), dist.measurementEnd.y());
    distLine->setPen(QPen(Qt::magenta, 2, Qt::DashLine));
    distLine->setVisible(true);

    m_distanceLines.append(distLine);
}

void PlotController::createDistanceLabel(const DistanceInfo& dist)
{
    double midX = (dist.measurementStart.x() + dist.measurementEnd.x()) / 2;
    double midY = (dist.measurementStart.y() + dist.measurementEnd.y()) / 2;

    QCPItemText* distLabel = new QCPItemText(m_plot);
    distLabel->setPositionAlignment(Qt::AlignCenter);
    distLabel->setTextAlignment(Qt::AlignCenter);
    distLabel->setFont(QFont("Arial", 10, QFont::Bold));
    distLabel->setColor(Qt::darkMagenta);
    distLabel->setPadding(QMargins(5, 5, 5, 5));
    distLabel->setText(QString("%1 мм").arg(QString::number(dist.distance, 'f', 3)));
    distLabel->position->setCoords(midX, midY);
    distLabel->setVisible(true);

    m_distanceLabels.append(distLabel);
}

void PlotController::clearHighlight()
{
    hideAllHighlights();
    m_plot->replot();
}

void PlotController::hideAllHighlights()
{
    removeAllSegmentGraphs();
    removeAllLineGraphs();
}

void PlotController::clearDistances()
{
    clearDistanceLines();
    clearDistanceLabels();

    if (m_plot) {
        m_plot->replot();
    }
}

void PlotController::clearDistanceLines()
{
    for (QCPItemLine* distLine : m_distanceLines) {
        if (distLine) {
            m_plot->removeItem(distLine);
        }
    }
    m_distanceLines.clear();
}

void PlotController::clearDistanceLabels()
{
    for (QCPItemText* label : m_distanceLabels) {
        if (label) {
            m_plot->removeItem(label);
        }
    }
    m_distanceLabels.clear();
}
