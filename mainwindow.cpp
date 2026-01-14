#include "mainwindow.h"

#include <QPushButton>
#include <QTableWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QMessageBox>
#include <QLabel>
#include <QSplitter>
#include <QGroupBox>
#include <QFont>
#include <QTabWidget>
#include <QTextEdit>
#include <QFileDialog>

#include <iostream>

#include <QtConcurrent/QtConcurrent>
#include <QFuture>

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , plot(nullptr)
    , modeBox(nullptr)
    , analyzeButton(nullptr)
    , exportButton(nullptr)
    , continueButton(nullptr)
    , resultsTable(nullptr)
    , longLinesTable(nullptr)
    , statusLabel(nullptr)
    , splitter(nullptr)
    , tabWidget(nullptr)
    , minXSpin(nullptr)
    , maxXSpin(nullptr)
    , methodBox(nullptr)
    , mainLayout(nullptr)
    , toolbarLayout(nullptr)
    , paramsDialog(nullptr)
    , plotController(nullptr)
    , resultsExporter(nullptr)
    , fileWorker(nullptr)
    , laserThread(nullptr)
    , laserWorker(nullptr)
    , isLaserUpdating(true)
    , selectedSegmentId(-1)
{
    std::cout << "[INFO] Initializing main window\n";

    createUI();              // Step 1: Create UI
    createConnections();     // Step 2: Connect signals/slots
    initializeParameters();  // Step 3: Set default parameters
    loadSettings();          // Step 4: Load saved settings
    startLaserUpdating();    // Step 5: Start laser updating

    resize(1400, 900);
}

MainWindow::~MainWindow()
{
    stopLaserUpdating();

    delete paramsDialog;
    delete plotController;
    delete resultsExporter;

    if (laserThread) {
        laserThread->quit();
        laserThread->wait();
        delete laserThread;
    }
}


void MainWindow::createUI(){
    QWidget* central = new QWidget(this);
    mainLayout = new QVBoxLayout(central);

    createToolbar();
    createPlotArea();
    createTables();

    mainLayout->addLayout(toolbarLayout);
    mainLayout->addWidget(splitter);

    mainLayout->setStretch(0, 0);
    mainLayout->setStretch(1, 1);
    setCentralWidget(central);
}

void MainWindow::createToolbar()
{
    toolbarLayout = new QHBoxLayout();

    QGroupBox* controlGroup = new QGroupBox("Управление");
    QHBoxLayout* controlLayout = new QHBoxLayout(controlGroup);

    modeBox = new QComboBox();
    modeBox->addItem("Points");
    modeBox->addItem("Lines");
    modeBox->setCurrentIndex(1);

    analyzeButton = new QPushButton("Analyze geometry");
    continueButton = new QPushButton("Continue updating");
    continueButton->setEnabled(false);

    methodBox = new QComboBox();
    methodBox->addItem("Linear Fit");
    methodBox->addItem("Hough Transform");
    methodBox->addItem("RANSAC");
    methodBox->setCurrentIndex(0);

    exportButton = new QPushButton("Export results");
    exportButton->setEnabled(false);

    exportProfileButton = new QPushButton("Export profile (CSV)");
    exportProfileButton->setEnabled(false);

    clearSelectionButton = new QPushButton("Clear selection");
    clearSelectionButton->setEnabled(false);

    paramsButton = new QPushButton("Parameters...");
    paramsButton->setToolTip("Configure analysis parameters");

    // Range selectors
    minXSpin = new QDoubleSpinBox();
    minXSpin->setRange(-1e6, 1e6);
    minXSpin->setDecimals(3);
    minXSpin->setSingleStep(0.1);
    minXSpin->setValue(-100.0);

    maxXSpin = new QDoubleSpinBox();
    maxXSpin->setRange(-1e6, 1e6);
    maxXSpin->setDecimals(3);
    maxXSpin->setSingleStep(0.1);
    maxXSpin->setValue(100.0);

    controlLayout->addWidget(new QLabel("Mode:"));
    controlLayout->addWidget(modeBox);
    controlLayout->addWidget(new QLabel("Method:"));
    controlLayout->addWidget(methodBox);
    controlLayout->addWidget(paramsButton);
    controlLayout->addWidget(analyzeButton);
    controlLayout->addWidget(continueButton);
    controlLayout->addWidget(exportButton);
    controlLayout->addWidget(exportProfileButton);
    controlLayout->addWidget(clearSelectionButton);
    controlLayout->addWidget(new QLabel("Min X:"));
    controlLayout->addWidget(minXSpin);
    controlLayout->addWidget(new QLabel("Max X:"));
    controlLayout->addWidget(maxXSpin);
    controlLayout->addStretch();

    statusLabel = new QLabel("Real-time updating");

    toolbarLayout->addWidget(controlGroup);
    toolbarLayout->addStretch();
    toolbarLayout->addWidget(statusLabel);
}

void MainWindow::createPlotArea()
{
    plot = new QCustomPlot();

    plotController = new PlotController(plot, this);
    resultsExporter = new ResultsExporter(this);

    splitter = new QSplitter(Qt::Vertical);
    splitter->addWidget(plot);
}

void MainWindow::createTables()
{
    resultsTable = new QTableWidget();
    longLinesTable = new QTableWidget();

    tabWidget = new QTabWidget();
    tabWidget->addTab(resultsTable, "Segments");
    tabWidget->addTab(longLinesTable, "Lines");

    tableController = new TableController(resultsTable, longLinesTable, this);

    if (splitter) {
        splitter->addWidget(tabWidget);
        splitter->setStretchFactor(0, 4);
        splitter->setStretchFactor(1, 1);
    }
}

void MainWindow::createConnections()
{
    // Plot connections
    connect(modeBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            plotController, &PlotController::setDisplayMode);

    connect(plotController, &PlotController::profileUpdated,
            this, &MainWindow::onProfileUpdated);

    // Analysis connections
    connect(analyzeButton, &QPushButton::clicked,
            this, &MainWindow::onAnalyzeGeometry);

    connect(continueButton, &QPushButton::clicked,
            this, &MainWindow::onContinueUpdating);

    connect(&futureWatcher, &QFutureWatcher<GeometryResult>::finished,
            this, &MainWindow::onAnalysisFinished);

    // Export connections
    connect(exportButton, &QPushButton::clicked,
            resultsExporter, &ResultsExporter::exportToFile);

    connect(exportProfileButton, &QPushButton::clicked,
            this, &MainWindow::onExportProfile);

    // Table controller connections
    connect(tableController, &TableController::segmentSelected,
            this, [this](int segmentId) {
                auto segment = tableController->getSegmentAtRow(
                    resultsTable->currentRow());
                plotController->highlightSegment(segment);
            });

    connect(tableController, &TableController::longLineSelected,
            this, [this](int lineId) {
                auto line = tableController->getLongLineAtRow(
                    longLinesTable->currentRow());
                plotController->highlightLine(line);
            });

    connect(tableController, &TableController::segmentDoubleClicked,
            this, [this](int segmentId) {
                selectedSegmentId = segmentId;
                plotController->highlightSegment(
                    tableController->getSegmentAtRow(resultsTable->currentRow()));
                clearSelectionButton->setEnabled(true);
                statusLabel->setText(QString("Selected segment %1").arg(selectedSegmentId));
            });


    // Utility connections
    connect(clearSelectionButton, &QPushButton::clicked,
            this, &MainWindow::onClearSelection);

    connect(paramsButton, &QPushButton::clicked,
            this, &MainWindow::onShowParamsDialog);

    // Laser/worker connections
    if (fileWorker) {
        connect(fileWorker, &FileProfileWorker::profileReady,
                this, &MainWindow::onNewProfile);
        connect(fileWorker, &FileProfileWorker::error,
                this, &MainWindow::onLaserError);
    }
}


void MainWindow::initializeParameters()
{
    // Linear Fit parameters
    linearFitParams.minPointsPerSegment = 3;
    linearFitParams.residualTolerance = 0.08;
    linearFitParams.mergeAngleTolerance = 4.5;
    linearFitParams.mergeDistanceTolerance = 0.4;

    // Hough parameters
    houghParams.dTheta = 1.5;
    houghParams.thetaBins = 120;
    houghParams.dRho = 0.15;
    houghParams.minVotesPercent = 15.0;
    houghParams.minPointsPerLine = 3;
    houghParams.mergeAngleTolerance = 3.0;
    houghParams.mergeDistanceTolerance = 0.5;

    // RANSAC parameters
    ransacParams.maxIterations = 600;
    ransacParams.distanceThreshold = 0.15;
    ransacParams.minInliers = 3;
    ransacParams.minLineLength = 0.5;
    ransacParams.maxLinesToFind = 25;
    ransacParams.mergeAngleTolerance = 6.0;
    ransacParams.mergeDistanceTolerance = 0.3;
}

void MainWindow::loadSettings()
{
    // ** //
    setWindowTitle("Profile Analysis System");
}

void MainWindow::setUIStateAnalyzing()
{
    analyzeButton->setEnabled(false);
    continueButton->setEnabled(false);
    exportButton->setEnabled(false);
    statusLabel->setText("Analyzing geometry...");
}

void MainWindow::setUIStateReady()
{
    analyzeButton->setEnabled(true);
    continueButton->setEnabled(true);
    exportButton->setEnabled(true);
    statusLabel->setText("Analysis complete");
}

void MainWindow::setUIStateUpdating()
{
    analyzeButton->setEnabled(true);
    continueButton->setEnabled(false);
    exportButton->setEnabled(false);
    statusLabel->setText("Real-time updating");
}

void MainWindow::onShowParamsDialog()
{
    if (!paramsDialog) {
        paramsDialog = new AnalysisParamsDialog(this);

        // Устанавливаем текущие параметры в диалог
        paramsDialog->setLinearFitParams(linearFitParams);
        paramsDialog->setHoughParams(houghParams);
        paramsDialog->setRANSACParams(ransacParams);
        paramsDialog->setCurrentMethod(methodBox->currentIndex());
    }

    // Показываем диалог
    if (paramsDialog->exec() == QDialog::Accepted) {
        // Сохраняем новые параметры
        linearFitParams = paramsDialog->getLinearFitParams();
        houghParams = paramsDialog->getHoughParams();
        ransacParams = paramsDialog->getRANSACParams();

        statusLabel->setText("Parameters updated");
    }
}

QVector<ProfilePoint> MainWindow::filterProfileByXRange() const
{
    double minX = minXSpin->value();
    double maxX = maxXSpin->value();

    QVector<ProfilePoint> filtered;
    filtered.reserve(profile.size());

    for (const auto& p : profile) {
        if (p.valid && p.x >= minX && p.x <= maxX) {
            filtered.append(p);
        }
    }

    return filtered;
}

bool MainWindow::validateAnalysisPrerequisites() const
{
    if (profile.isEmpty()) {
        QMessageBox::warning(const_cast<MainWindow*>(this),
                             "Error",
                             "No profile data for analysis");
        return false;
    }

    double minX = minXSpin->value();
    double maxX = maxXSpin->value();

    if (minX > maxX) {
        QMessageBox::warning(const_cast<MainWindow*>(this),
                             "Error",
                             "Min X is greater than Max X");
        return false;
    }

    auto filtered = filterProfileByXRange();
    if (filtered.size() < 2) {
        QMessageBox::warning(const_cast<MainWindow*>(this),
                             "Error",
                             "Not enough points in the selected X range");
        return false;
    }

    return true;
}

void MainWindow::startLaserUpdating()
{
    // Останавливаем предыдущий поток, если он существует
    if (laserThread) {
        laserThread->quit();
        laserThread->wait();
        delete laserThread;
    }

    laserThread = new QThread(this);

    // ---- FROM CSV PROFILE ----
    fileWorker = new FileProfileWorker();
    fileWorker->moveToThread(laserThread);

    connect(laserThread, &QThread::started,
            fileWorker, &FileProfileWorker::start);

    connect(laserThread, &QThread::finished,
            fileWorker, &QObject::deleteLater);

    connect(fileWorker, &FileProfileWorker::profileReady,
            this, &MainWindow::onNewProfile);

    connect(fileWorker, &FileProfileWorker::error,
            this, &MainWindow::onLaserError);

    laserThread->start();
    isLaserUpdating = true;
}

void MainWindow::stopLaserUpdating()
{
    if (laserThread) {
        if (fileWorker) {
            fileWorker->stop();
        }
        laserThread->quit();
        laserThread->wait();
        delete laserThread;
        laserThread = nullptr;
        fileWorker = nullptr;
    }
    isLaserUpdating = false;
}

void MainWindow::onLaserError(const QString& message)
{
    QMessageBox::warning(this, "Profile read error", message);
    statusLabel->setText("Error: " + message);
}

void MainWindow::onNewProfile(const QVector<ProfilePoint>& newProfile)
{
    if (!isLaserUpdating) {
        return; // Не обновляем, если лазер остановлен
    }

    profile = newProfile;
    plotController->updateProfile(newProfile);
}

void MainWindow::onProfileUpdated(int validPoints, int totalPoints)
{
    statusLabel->setText(QString("Points: %1 (%2 valid)").arg(totalPoints).arg(validPoints));
}

void MainWindow::onExportProfile()
{
    if (profile.isEmpty()) {
        QMessageBox::warning(this, "Error", "No profile data to export");
        return;
    }

    QString fileName = QFileDialog::getSaveFileName(this,
                                                    "Export profile to CSV",
                                                    "",
                                                    "CSV files (*.csv)");

    if (fileName.isEmpty()) {
        return;
    }

    if (!fileName.endsWith(".csv", Qt::CaseInsensitive)) {
        fileName += ".csv";
    }

    try {
        QFile file(fileName);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(this, "Error",
                                  QString("Cannot open file for writing:\n%1").arg(file.errorString()));
            return;
        }

        QTextStream out(&file);
        out.setCodec("UTF-8");

        // Header
        out << "X_mm;Z_mm;Valid\n";

        for (const ProfilePoint& point : profile) {
            out << QString::number(point.x, 'f', 6) << ";"
                << QString::number(point.z, 'f', 6) << ";"
                << (point.valid ? "1" : "0") << "\n";
        }

        file.close();

        QMessageBox::information(this, "Success",
                                 QString("Profile exported to:\n%1").arg(fileName));

    } catch (const std::exception& e) {
        QMessageBox::critical(this, "Error",
                              QString("Export error: %1").arg(e.what()));
    }
}

void MainWindow::onClearSelection()
{
    selectedSegmentId = -1;
    clearSelectionButton->setEnabled(false);
    plotController->clearHighlight();
    statusLabel->setText("Selection cleared");
}

bool MainWindow::isPointOnSegment(double px, double pz,
                                  double x1, double z1,
                                  double x2, double z2,
                                  double tolerance)
{
    double cross = (px - x1) * (z2 - z1) - (pz - z1) * (x2 - x1);
    if (std::abs(cross) > tolerance) {
        return false;
    }
    double dot = (px - x1) * (x2 - x1) + (pz - z1) * (z2 - z1);
    if (dot < 0 || dot > ((x2 - x1)*(x2 - x1) + (z2 - z1)*(z2 - z1))) {
        return false;
    }
    return true;
}

void MainWindow::onAnalyzeGeometry()
{
    if (!validateAnalysisPrerequisites()) {
        return;
    }

    // Stop live updating
    stopLaserUpdating();

    // Get filtered profile
    auto filtered = filterProfileByXRange();

    setUIStateAnalyzing();

    // Cancel any running analysis
    if (futureWatcher.isRunning()) {
        futureWatcher.cancel();
        futureWatcher.waitForFinished();
    }

    // Disconnect any previous connections
    disconnect(&futureWatcher, nullptr, nullptr, nullptr);
    connect(&futureWatcher, &QFutureWatcher<GeometryResult>::finished,
            this, &MainWindow::onAnalysisFinished);

    // Start analysis based on selected method
    if (methodBox->currentIndex() == 0) {
        LinearFitParams params = getLinearFitParams();
        futureWatcher.setFuture(QtConcurrent::run([filtered, params]() {
            return ProfileAnalyzer::analyzeGeometry(filtered, params);
        }));
    } else if (methodBox->currentIndex() == 1) {
        HoughParams params = getHoughParams();
        futureWatcher.setFuture(QtConcurrent::run([filtered, params]() {
            return ProfileAnalyzer::analyzeGeometryHough(filtered, params);
        }));
    } else {
        RANSACParams params = getRANSACParams();
        futureWatcher.setFuture(QtConcurrent::run([filtered, params]() {
            return ProfileAnalyzer::analyzeGeometryRANSAC(filtered, params);
        }));
    }

}

void MainWindow::onAnalysisFinished()
{
    try {
        GeometryResult result = futureWatcher.result();

        qDebug() << "Result status:" << result.status;
        qDebug() << "Segments count:" << result.segments.size();
        qDebug() << "Long lines count:" << result.longLines.size();

        currentResult = result;

        resultsExporter->setCurrentResult(result);
        tableController->showResultsTable(result);
        tableController->showLongLinesTable(result);

        exportButton->setEnabled(true);
        statusLabel->setText(result.status);

        plotController->clearHighlight();
        plotController->clearDistances();

        for (const SegmentInfo& line : result.longLines) {
            plotController->highlightLine(line);
        }

        if (!result.segments.isEmpty()) {
            resultsTable->selectRow(0);
            plotController->highlightSegment(result.segments.first());
        }
    } catch (const std::exception& e) {
        statusLabel->setText(QString("Analysis error: %1").arg(e.what()));
        QMessageBox::critical(this, "Error",
                              QString("Geometry analysis failed: %1").arg(e.what()));
    }

    analyzeButton->setEnabled(true);
}

void MainWindow::onContinueUpdating()
{
    tableController->clearTables();
    plotController->clearHighlight();
    plotController->clearDistances();
    exportButton->setEnabled(false);

    analyzeButton->setEnabled(true);
    continueButton->setEnabled(false);

    startLaserUpdating();
    statusLabel->setText("Real-time updating");
}

void MainWindow::onSegmentSelected(int row, int column)
{
    Q_UNUSED(column);

    if (row >= 0 && row < currentResult.segments.size()) {
        plotController->highlightSegment(currentResult.segments[row]);
    } else {
        plotController->clearHighlight();
    }
}

void MainWindow::onLongLineSelected(int row, int column)
{
    Q_UNUSED(column);

    if (row >= 0 && row < currentResult.longLines.size()) {
        plotController->highlightLine(currentResult.longLines[row]);
    } else {
        plotController->clearHighlight();
    }
}

void MainWindow::onSegmentDoubleClicked(int row, int column)
{
    Q_UNUSED(column);

    if (row >= 0 && row < currentResult.segments.size()) {
        selectedSegmentId = currentResult.segments[row].id;

        plotController->highlightSegment(currentResult.segments[row]);

        clearSelectionButton->setEnabled(true);

        statusLabel->setText(QString("Selected segment %1").arg(selectedSegmentId));
    }
}

RANSACParams MainWindow::getRANSACParams() const {
    return ransacParams;
}

LinearFitParams MainWindow::getLinearFitParams() const {
    return linearFitParams;
}

HoughParams MainWindow::getHoughParams() const {
    return houghParams;
}
