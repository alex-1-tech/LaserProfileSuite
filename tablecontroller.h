#pragma once

/**
 * @file    tablecontroller.h
 * @brief   Controller class for managing results tables
 * @author  alex-1-tech
 * @date    2026
 */

#include <QObject>
#include <QTableWidget>
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

signals:
    /**
     * @brief Emitted when segment is selected in table
     * @param segmentId ID of selected segment
     */
    void segmentSelected(int segmentId);

    /**
     * @brief Emitted when long line is selected in table
     * @param lineId ID of selected long line
     */
    void longLineSelected(int lineId);

    /**
     * @brief Emitted when segment is double-clicked in table
     * @param segmentId ID of double-clicked segment
     */
    void segmentDoubleClicked(int segmentId);

private:
    QTableWidget* m_segmentsTable;      ///< Table for individual segments
    QTableWidget* m_longLinesTable;     ///< Table for merged long lines

    QVector<SegmentInfo> m_currentSegments;   ///< Current displayed segments
    QVector<SegmentInfo> m_currentLongLines;  ///< Current displayed long lines
};
