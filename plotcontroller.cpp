#include "plotcontroller.h"
#include "qcustomplot.h"
#include <QFont>
#include <QPen>
#include <QDebug>

PlotController::PlotController(QCustomPlot* plot, QObject* parent)
    : QObject(parent)
    , m_plot(plot)
    , m_displayMode(DisplayMode::Lines)
    , m_segmentGraph(nullptr)
    , m_segmentLine(nullptr)
    , m_segmentLabel(nullptr)
    , m_mainGraph(nullptr)
    , m_invalidGraph(nullptr)
    , m_highlightedLineGraph(nullptr)
{
    Q_ASSERT(plot != nullptr);
    setupPlot();
    initializeHighlightItems();
}

PlotController::~PlotController()
{
    cleanup();
}

void PlotController::cleanup()
{
    // Удаляем все созданные элементы
    clearDistances();

    if (m_mainGraph) {
        m_plot->removeGraph(m_mainGraph);
        m_mainGraph = nullptr;
    }

    if (m_invalidGraph) {
        m_plot->removeGraph(m_invalidGraph);
        m_invalidGraph = nullptr;
    }

    if (m_highlightedLineGraph) {
        m_plot->removeGraph(m_highlightedLineGraph);
        m_highlightedLineGraph = nullptr;
    }

    if (m_segmentGraph) {
        m_plot->removeGraph(m_segmentGraph);
        m_segmentGraph = nullptr;
    }

    if (m_segmentLine) {
        m_plot->removeItem(m_segmentLine);
        m_segmentLine = nullptr;
    }

    if (m_segmentLabel) {
        m_plot->removeItem(m_segmentLabel);
        m_segmentLabel = nullptr;
    }
}

void PlotController::initializeHighlightItems()
{
    // Создаем график для выделения сегментов
    m_segmentGraph = m_plot->addGraph();
    m_segmentGraph->setVisible(false);
    m_segmentGraph->setName("highlighted_segment");

    // Создаем элементы для выделения
    m_segmentLine = new QCPItemLine(m_plot);
    m_segmentLine->setPen(QPen(Qt::green, 2, Qt::DashLine));
    m_segmentLine->setVisible(false);

    m_segmentLabel = new QCPItemText(m_plot);
    m_segmentLabel->setPositionAlignment(Qt::AlignTop | Qt::AlignHCenter);
    m_segmentLabel->setTextAlignment(Qt::AlignCenter);
    m_segmentLabel->setFont(QFont("Arial", 10, QFont::Bold));
    m_segmentLabel->setColor(Qt::darkBlue);
    m_segmentLabel->setPadding(QMargins(5, 5, 5, 5));
    m_segmentLabel->setVisible(false);
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
    updateLineGraphsStyle();
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
    if (m_segmentGraph) {
        m_segmentGraph->setLineStyle(QCPGraph::lsLine);
        m_segmentGraph->setScatterStyle(QCPScatterStyle(QCPScatterStyle::ssDisc, Qt::red, 6));
        m_segmentGraph->setPen(QPen(Qt::red, 3));
    }
}

void PlotController::updateLineGraphsStyle()
{
    for (QCPGraph* lineGraph : m_lineGraphs) {
        if (lineGraph) {
            lineGraph->setLineStyle(QCPGraph::lsLine);
            lineGraph->setPen(QPen(Qt::green, 2));
        }
    }

    if (m_highlightedLineGraph) {
        m_highlightedLineGraph->setLineStyle(QCPGraph::lsLine);
        m_highlightedLineGraph->setPen(QPen(Qt::yellow, 4));
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

void PlotController::highlightSegment(const SegmentInfo& segment)
{
    hideAllHighlights();

    if (!m_segmentGraph) return;

    QVector<double> segX = QVector<double>() << segment.startX << segment.endX;
    QVector<double> segZ = QVector<double>() << segment.startZ << segment.endZ;
    m_segmentGraph->setData(segX, segZ);
    m_segmentGraph->setVisible(true);

    updateSegmentDisplay(segment);
    m_plot->replot();
}

void PlotController::updateSegmentDisplay(const SegmentInfo& segment)
{
    if (m_segmentLine) {
        m_segmentLine->start->setCoords(segment.startX, segment.startZ);
        m_segmentLine->end->setCoords(segment.endX, segment.endZ);
        m_segmentLine->setVisible(true);
    }

    if (m_segmentLabel) {
        double midX = (segment.startX + segment.endX) / 2;
        double midZ = (segment.startZ + segment.endZ) / 2;

        m_segmentLabel->setText(formatSegmentLabel(segment));
        m_segmentLabel->position->setCoords(midX, midZ);
        m_segmentLabel->setVisible(true);
    }
}

QString PlotController::formatSegmentLabel(const SegmentInfo& segment) const
{
    return QString("Сегмент %1\n%2 мм\n%3°")
        .arg(segment.id)
        .arg(QString::number(segment.length, 'f', 2))
        .arg(QString::number(segment.angle, 'f', 1));
}

void PlotController::highlightLine(const SegmentInfo& line)
{
    hideAllHighlights();

    removeHighlightedLineGraph();
    createHighlightedLineGraph(line);
    m_plot->replot();
}

void PlotController::removeHighlightedLineGraph()
{
    if (m_highlightedLineGraph) {
        m_plot->removeGraph(m_highlightedLineGraph);
        m_highlightedLineGraph = nullptr;
    }
}

void PlotController::createHighlightedLineGraph(const SegmentInfo& line)
{
    m_highlightedLineGraph = m_plot->addGraph();
    m_highlightedLineGraph->setName("highlighted_line");

    GraphData lineData = prepareLineData(line);
    m_highlightedLineGraph->setData(lineData.x, lineData.z);
    m_highlightedLineGraph->setPen(QPen(Qt::yellow, 4));
    m_highlightedLineGraph->setLineStyle(QCPGraph::lsLine);
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
    if (m_segmentGraph) m_segmentGraph->setVisible(false);
    if (m_segmentLine) m_segmentLine->setVisible(false);
    if (m_segmentLabel) m_segmentLabel->setVisible(false);
    if (m_highlightedLineGraph) m_highlightedLineGraph->setVisible(false);
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

void PlotController::clearLineGraphs()
{
    for (QCPGraph* lineGraph : m_lineGraphs) {
        if (lineGraph) {
            m_plot->removeGraph(lineGraph);
        }
    }
    m_lineGraphs.clear();
}
