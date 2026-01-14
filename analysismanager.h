#ifndef ANALYSISMANAGER_H
#define ANALYSISMANAGER_H

#include <QObject>
#include <QFutureWatcher>
#include "segmentinfo.h"
#include "analysisparams.h"

/**
 * @class AnalysisManager
 * @brief Manages analysis workflow including parameter validation and async execution
 *
 * This class separates analysis logic from UI, making it testable and reusable.
 * It handles parameter validation, method selection, and asynchronous execution.
 */
class AnalysisManager : public QObject
{
    Q_OBJECT

public:
    /**
     * @brief Constructor
     * @param parent Parent QObject
     */
    explicit AnalysisManager(QObject* parent = nullptr);

    /**
     * @brief Destructor
     */
    ~AnalysisManager() = default;

    /**
     * @brief Start analysis with given parameters
     * @param profile Filtered profile data to analyze
     * @param method Analysis method index (0=LinearFit, 1=Hough, 2=RANSAC)
     * @param params Analysis parameters
     * @return Future containing analysis result
     */
    QFuture<GeometryResult> analyze(const QVector<ProfilePoint>& profile,
                                    int method,
                                    const AnalysisParams& params);

    /**
     * @brief Cancel current analysis if running
     */
    void cancelCurrentAnalysis();

signals:
    /** @brief Emitted when analysis starts */
    void analysisStarted();

    /**
     * @brief Emitted when analysis completes successfully
     * @param result Analysis results
     */
    void analysisFinished(const GeometryResult& result);

    /**
     * @brief Emitted when analysis fails
     * @param error Error message
     */
    void analysisFailed(const QString& error);

private:
    QFutureWatcher<GeometryResult> m_watcher; ///< Watcher for async analysis
};

#endif // ANALYSISMANAGER_H
