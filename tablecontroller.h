#pragma once

/**
 * @file    tablecontroller.h
 * @brief   Controller class for managing results tables
 * @author  alex-1-tech
 * @date    2026
 */

#include <QObject>
#include <QTableWidget>
#include <QSet>
#include "segmentinfo.h"

/**
 * @class   TableController
 * @brief   Manages display and interaction with analysis results tables
 *
 * Handles two tables:
 * 1. Segments table - individual line segments from analysis
 * 2. Long lines table - merged long lines from multiple segments
 */
class TableController : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param segmentsTable Table widget for individual segments
     * @param longLinesTable Table widget for long lines
     * @param parent Parent QObject
     */
    explicit TableController(QTableWidget* segmentsTable,
                             QTableWidget* longLinesTable,
                             QObject* parent = nullptr);

    /**
     * @brief Display segments in the segments table
     * @param result Geometry analysis result
     */
    void showResultsTable(const GeometryResult& result);

    /**
     * @brief Display long lines in the long lines table
     * @param result Geometry analysis result
     */
    void showLongLinesTable(const GeometryResult& result);

    /**
     * @brief Clear all table data
     */
    void clearTables();

    /**
     * @brief Get segment ID from selected row in segments table
     * @param row Row index
     * @return Segment ID or -1 if invalid
     */
    int getSegmentIdAtRow(int row) const;

    /**
     * @brief Get long line ID from selected row in long lines table
     * @param row Row index
     * @return Line ID or -1 if invalid
     */
    int getLongLineIdAtRow(int row) const;

    /**
     * @brief Get segment information from segments table row
     * @param row Row index
     * @return SegmentInfo structure
     */
    SegmentInfo getSegmentAtRow(int row) const;

    /**
     * @brief Get long line information from long lines table row
     * @param row Row index
     * @return SegmentInfo structure
     */
    SegmentInfo getLongLineAtRow(int row) const;

    /**
     * @brief Get all selected segments
     * @return Vector of selected SegmentInfo
     */
    QVector<SegmentInfo> getSelectedSegments() const;

    /**
     * @brief Get all selected long lines
     * @return Vector of selected SegmentInfo for long lines
     */
    QVector<SegmentInfo> getSelectedLongLines() const;

signals:
    /**
     * @brief Emitted when segment selection changes
     * @param segmentIds Set of selected segment IDs
     */
    void segmentsSelected(const QSet<int>& segmentIds);

    /**
     * @brief Emitted when long line selection changes
     * @param lineIds Set of selected long line IDs
     */
    void longLinesSelected(const QSet<int>& lineIds);

private slots:
    void onSegmentsTableSelectionChanged();
    void onLongLinesTableSelectionChanged();

private:
    QTableWidget* m_segmentsTable;      ///< Table for individual segments
    QTableWidget* m_longLinesTable;     ///< Table for merged long lines

    QVector<SegmentInfo> m_currentSegments;   ///< Current displayed segments
    QVector<SegmentInfo> m_currentLongLines;  ///< Current displayed long lines
};
