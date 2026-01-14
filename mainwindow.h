#pragma once

/**
 * @file    mainwindow.h
 * @brief   Main application window for profile analysis system
 * @author  alex-1-tech
 * @date    2025
 */

#include <QMainWindow>
#include <QFutureWatcher>
#include <QThread>
#include <QDoubleSpinBox>

#include "segmentinfo.h"
#include "plotcontroller.h"
#include "tablecontroller.h"
#include "resultsexporter.h"
#include "fileprofileworker.h"
#include "analysisparamsdialog.h"

// Forward declarations
class QCustomPlot;
class QComboBox;
class QPushButton;
class QTableWidget;
class QLabel;
class QSplitter;
class LaserWorker;
class QTabWidget;
class QGroupBox;

/**
 * @class   MainWindow
 * @brief   Main application window managing profile analysis workflow
 *
 * Integrates all components:
 * - Profile visualization via PlotController
 * - Geometric analysis (RANSAC/Hough/LinearFit)
 * - Results display in tables
 * - Parameter configuration in separate dialog
 * - Data export functionality
 */
class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr); ///< Constructor
    ~MainWindow();                                  ///< Destructor

private slots:
    void onAnalyzeGeometry();                                       ///< Trigger geometric analysis
    void onAnalysisFinished();                                      ///< Handle analysis completion
    void onContinueUpdating();                                      ///< Toggle continuous profile updates
    void onNewProfile(const QVector<ProfilePoint>& newProfile);     ///< Process incoming profile data
    void onProfileUpdated(int validPoints, int totalPoints);        ///< Update profile statistics
    void onExportProfile();                                         ///< Export current profile data
    void onClearSelection();                                        ///< Clear all selections
    void onLaserError(const QString& message);                      ///< Handle laser/worker errors
    void onShowParamsDialog();                                      ///< Show parameters configuration dialog

    // Table controller slots
    void onSegmentSelected(int row, int column);                    ///< Handle segment table selection
    void onLongLineSelected(int row, int column);                   ///< Handle line table selection
    void onSegmentDoubleClicked(int row, int column);               ///< Handle segment double-click
private:
    // Initialization methods
    void createUI();                                        ///< Create all UI widgets and layouts
    void createToolbar();                                   ///< Create toolbar with controls
    void createPlotArea();                                  ///< Create plot widget and area
    void createTables();                                    ///< Create results tables
    void createConnections();                               ///< Setup signal-slot connections
    void initializeParameters();                            ///< Initialize analysis parameters with defaults
    void loadSettings();                                    ///< Load application settings

    // UI state management
    void setUIStateAnalyzing();                             ///< Set UI state to analyzing
    void setUIStateReady();                                 ///< Set UI state to ready
    void setUIStateUpdating();                              ///< Set UI state to updating

    // Analysis helpers
    QVector<ProfilePoint> filterProfileByXRange() const;    ///< Filter profile by X range
    bool validateAnalysisPrerequisites() const;             ///< Check if analysis can proceed

    void startLaserUpdating();                              ///< Start continuous profile updates
    void stopLaserUpdating();                               ///< Stop profile updates

    RANSACParams getRANSACParams() const;                   ///< Get current RANSAC parameters
    LinearFitParams getLinearFitParams() const;             ///< Get current LinearFit parameters
    HoughParams getHoughParams() const;                     ///< Get current Hough parameters

    /// @brief Check if point lies on segment within tolerance
    /// @param px, pz Point coordinates
    /// @param x1, z1 Segment start point
    /// @param x2, z2 Segment end point
    /// @param tolerance Maximum allowed distance
    /// @return true if point is on segment
    bool isPointOnSegment(double px, double pz,
                          double x1, double z1,
                          double x2, double z2,
                          double tolerance = 1.0);

private:
    // UI Components
    QCustomPlot* plot;                  ///< Main plotting widget
    QComboBox* modeBox;                 ///< Display mode selector
    QPushButton* analyzeButton;         ///< Analysis trigger button
    QPushButton* exportButton;          ///< Results export button
    QPushButton* continueButton;        ///< Continuous update toggle
    QPushButton* exportProfileButton;   ///< Profile data export button
    QPushButton* clearSelectionButton;  ///< Clear selection button
    QPushButton* paramsButton;          ///< Parameters dialog button
    QTableWidget* resultsTable;         ///< Segments table widget
    QTableWidget* longLinesTable;       ///< Lines table widget
    QLabel* statusLabel;                ///< Status display label
    QSplitter* splitter;                ///< Main layout splitter
    QTabWidget* tabWidget;              ///< Analysis results tabs
    QDoubleSpinBox* minXSpin;           ///< X-axis minimum bound
    QDoubleSpinBox* maxXSpin;           ///< X-axis maximum bound
    QComboBox* methodBox;               ///< Analysis method selector

    // Layouts
    QVBoxLayout* mainLayout;            ///< Main vertical layout
    QHBoxLayout* toolbarLayout;         ///< Toolbar horizontal layout

    // Parameters dialog
    AnalysisParamsDialog* paramsDialog; ///< Parameters configuration dialog
    LinearFitParams linearFitParams;    ///< Current Linear Fit parameters
    HoughParams houghParams;            ///< Current Hough parameters
    RANSACParams ransacParams;          ///< Current RANSAC parameters

    // Controllers
    PlotController* plotController;     ///< Plot visualization
    TableController* tableController;   ///< Table management controller
    ResultsExporter* resultsExporter;   ///< Results export handler

    // Data acquisition
    FileProfileWorker* fileWorker;      ///< File-based profile worker
    QThread* laserThread;               ///< Worker thread for laser
    LaserWorker* laserWorker;           ///< Laser data acquisition worker
    QVector<ProfilePoint> profile;      ///< Current profile data

    // Async processing
    QFutureWatcher<GeometryResult> futureWatcher; ///< Async analysis watcher
    GeometryResult currentResult;       ///< Last analysis results
    bool isLaserUpdating;               ///< Continuous update flag
    int selectedSegmentId;              ///< Currently selected segment ID
};
