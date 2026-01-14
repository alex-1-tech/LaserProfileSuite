#include "tablecontroller.h"
#include <QHeaderView>
#include <QDebug>

TableController::TableController(QTableWidget* segmentsTable,
                                 QTableWidget* longLinesTable,
                                 QObject* parent)
    : QObject(parent)
    , m_segmentsTable(segmentsTable)
    , m_longLinesTable(longLinesTable)
    , m_currentSegments()
    , m_currentLongLines()
{
    // Setup segments table
    m_segmentsTable->setColumnCount(8);
    QStringList segmentHeaders;
    segmentHeaders << "ID" << "Type" << "Length (mm)" << "Angle (°)"
                   << "Start.X" << "End.X" << "Start.Z" << "End.Z";
    m_segmentsTable->setHorizontalHeaderLabels(segmentHeaders);
    m_segmentsTable->verticalHeader()->setVisible(false);
    m_segmentsTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_segmentsTable->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // Setup long lines table
    m_longLinesTable->setColumnCount(10);
    QStringList longLinesHeaders;
    longLinesHeaders << "ID" << "Type" << "Length (mm)" << "Angle (°)"
                     << "Start.X" << "End.X" << "Start.Z" << "End.Z"
                     << "Segments" << "IsLongLine";
    m_longLinesTable->setHorizontalHeaderLabels(longLinesHeaders);
    m_longLinesTable->verticalHeader()->setVisible(false);
    m_longLinesTable->setSelectionBehavior(QAbstractItemView::SelectRows);
    m_longLinesTable->setEditTriggers(QAbstractItemView::NoEditTriggers);

    // Connect table signals
    connect(m_segmentsTable, &QTableWidget::cellClicked,
            this, [this](int row, int column) {
                Q_UNUSED(column);
                if (row >= 0 && row < m_currentSegments.size()) {
                    emit segmentSelected(m_currentSegments[row].id);
                }
            });

    connect(m_segmentsTable, &QTableWidget::cellDoubleClicked,
            this, [this](int row, int column) {
                Q_UNUSED(column);
                if (row >= 0 && row < m_currentSegments.size()) {
                    emit segmentDoubleClicked(m_currentSegments[row].id);
                }
            });

    connect(m_longLinesTable, &QTableWidget::cellClicked,
            this, [this](int row, int column) {
                Q_UNUSED(column);
                if (row >= 0 && row < m_currentLongLines.size()) {
                    emit longLineSelected(m_currentLongLines[row].id);
                }
            });
}

void TableController::showResultsTable(const GeometryResult& result)
{
    m_currentSegments = result.segments;
    m_segmentsTable->setRowCount(result.segments.size());

    for (int i = 0; i < result.segments.size(); i++) {
        const SegmentInfo& segment = result.segments[i];

        m_segmentsTable->setItem(i, 0, new QTableWidgetItem(QString::number(segment.id)));
        m_segmentsTable->setItem(i, 1, new QTableWidgetItem(segment.type));
        m_segmentsTable->setItem(i, 2, new QTableWidgetItem(QString::number(segment.length, 'f', 3)));
        m_segmentsTable->setItem(i, 3, new QTableWidgetItem(QString::number(segment.angle, 'f', 1)));
        m_segmentsTable->setItem(i, 4, new QTableWidgetItem(QString::number(segment.startX, 'f', 2)));
        m_segmentsTable->setItem(i, 5, new QTableWidgetItem(QString::number(segment.endX, 'f', 2)));
        m_segmentsTable->setItem(i, 6, new QTableWidgetItem(QString::number(segment.startZ, 'f', 2)));
        m_segmentsTable->setItem(i, 7, new QTableWidgetItem(QString::number(segment.endZ, 'f', 2)));

        // Make items non-editable
        for (int col = 0; col < 8; col++) {
            QTableWidgetItem* item = m_segmentsTable->item(i, col);
            if (item) {
                item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
            }
        }
    }

    m_segmentsTable->resizeColumnsToContents();
}

void TableController::showLongLinesTable(const GeometryResult& result)
{
    m_currentLongLines = result.longLines;
    m_longLinesTable->setRowCount(result.longLines.size());

    for (int i = 0; i < result.longLines.size(); i++) {
        const SegmentInfo& segment = result.longLines[i];

        m_longLinesTable->setItem(i, 0, new QTableWidgetItem(QString::number(segment.id)));
        m_longLinesTable->setItem(i, 1, new QTableWidgetItem(segment.type));
        m_longLinesTable->setItem(i, 2, new QTableWidgetItem(QString::number(segment.length, 'f', 3)));
        m_longLinesTable->setItem(i, 3, new QTableWidgetItem(QString::number(segment.angle, 'f', 1)));
        m_longLinesTable->setItem(i, 4, new QTableWidgetItem(QString::number(segment.startX, 'f', 2)));
        m_longLinesTable->setItem(i, 5, new QTableWidgetItem(QString::number(segment.endX, 'f', 2)));
        m_longLinesTable->setItem(i, 6, new QTableWidgetItem(QString::number(segment.startZ, 'f', 2)));
        m_longLinesTable->setItem(i, 7, new QTableWidgetItem(QString::number(segment.endZ, 'f', 2)));

        // Additional columns for long lines
        m_longLinesTable->setItem(i, 8, new QTableWidgetItem(QString::number(segment.mergedSegmentIds.size())));
        m_longLinesTable->setItem(i, 9, new QTableWidgetItem(segment.isLongLine ? "Yes" : "No"));

        // Make items non-editable
        for (int col = 0; col < 10; col++) {
            QTableWidgetItem* item = m_longLinesTable->item(i, col);
            if (item) {
                item->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
            }
        }
    }

    m_longLinesTable->resizeColumnsToContents();
}

void TableController::clearTables()
{
    m_segmentsTable->setRowCount(0);
    m_longLinesTable->setRowCount(0);
    m_currentSegments.clear();
    m_currentLongLines.clear();
}

int TableController::getSegmentIdAtRow(int row) const
{
    if (row >= 0 && row < m_currentSegments.size()) {
        return m_currentSegments[row].id;
    }
    return -1;
}

int TableController::getLongLineIdAtRow(int row) const
{
    if (row >= 0 && row < m_currentLongLines.size()) {
        return m_currentLongLines[row].id;
    }
    return -1;
}

SegmentInfo TableController::getSegmentAtRow(int row) const
{
    if (row >= 0 && row < m_currentSegments.size()) {
        return m_currentSegments[row];
    }
    return SegmentInfo();
}

SegmentInfo TableController::getLongLineAtRow(int row) const
{
    if (row >= 0 && row < m_currentLongLines.size()) {
        return m_currentLongLines[row];
    }
    return SegmentInfo();
}
