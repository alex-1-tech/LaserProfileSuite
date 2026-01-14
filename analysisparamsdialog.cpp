#include "analysisparamsdialog.h"
#include <QPushButton>
#include <QVBoxLayout>
#include <QLabel>
#include <QSpacerItem>

AnalysisParamsDialog::AnalysisParamsDialog(QWidget *parent)
    : QDialog(parent)
{
    setupUI();
    loadDefaultValues();

    setWindowTitle("Analysis Parameters");
    setMinimumWidth(500);
    resize(600, 500);
}

void AnalysisParamsDialog::setupUI()
{
    QVBoxLayout* mainLayout = new QVBoxLayout(this);

    // Method selection
    QGroupBox* methodGroup = new QGroupBox("Analysis Method");
    QHBoxLayout* methodLayout = new QHBoxLayout(methodGroup);

    QLabel* methodLabel = new QLabel("Method:");
    methodComboBox = new QComboBox();
    methodComboBox->addItem("Linear Fit");
    methodComboBox->addItem("Hough Transform");
    methodComboBox->addItem("RANSAC");

    methodLayout->addWidget(methodLabel);
    methodLayout->addWidget(methodComboBox);
    methodLayout->addStretch();

    mainLayout->addWidget(methodGroup);

    // Parameters stack
    paramsStack = new QStackedWidget();
    paramsStack->addWidget(createLinearFitPage());   // index 0
    paramsStack->addWidget(createHoughPage());       // index 1
    paramsStack->addWidget(createRANSACPage());      // index 2

    connect(methodComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &AnalysisParamsDialog::onMethodChanged);

    mainLayout->addWidget(paramsStack, 1);

    // Buttons
    buttonBox = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    defaultsButton = new QPushButton("Load Defaults");

    QHBoxLayout* buttonLayout = new QHBoxLayout();
    buttonLayout->addWidget(defaultsButton);
    buttonLayout->addStretch();
    buttonLayout->addWidget(buttonBox);

    connect(buttonBox, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttonBox, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(defaultsButton, &QPushButton::clicked, this, &AnalysisParamsDialog::onDefaultsButtonClicked);

    mainLayout->addLayout(buttonLayout);
}

QWidget* AnalysisParamsDialog::createLinearFitPage()
{
    QWidget* page = new QWidget();
    QVBoxLayout* pageLayout = new QVBoxLayout(page);

    // Basic parameters group
    QGroupBox* basicGroup = new QGroupBox("Basic Parameters");
    QFormLayout* basicLayout = new QFormLayout(basicGroup);

    linearFitMinPointsSpin = new QSpinBox();
    linearFitMinPointsSpin->setRange(2, 100);
    linearFitMinPointsSpin->setToolTip("Minimum number of points to form a segment");
    basicLayout->addRow("Min points per segment:", linearFitMinPointsSpin);

    linearFitResidualSpin = new QDoubleSpinBox();
    linearFitResidualSpin->setRange(0.001, 10.0);
    linearFitResidualSpin->setDecimals(3);
    linearFitResidualSpin->setToolTip("Maximum residual error for point to be included in segment");
    basicLayout->addRow("Residual tolerance (mm):", linearFitResidualSpin);

    pageLayout->addWidget(basicGroup);

    // Merge parameters group
    QGroupBox* mergeGroup = new QGroupBox("Merge Parameters");
    QFormLayout* mergeLayout = new QFormLayout(mergeGroup);

    linearFitMergeAngleSpin = new QDoubleSpinBox();
    linearFitMergeAngleSpin->setRange(0.1, 90.0);
    linearFitMergeAngleSpin->setDecimals(1);
    linearFitMergeAngleSpin->setToolTip("Maximum angle difference for merging segments");
    mergeLayout->addRow("Angle tolerance (°):", linearFitMergeAngleSpin);

    linearFitMergeDistSpin = new QDoubleSpinBox();
    linearFitMergeDistSpin->setRange(0.01, 50.0);
    linearFitMergeDistSpin->setDecimals(2);
    linearFitMergeDistSpin->setToolTip("Maximum distance between segment endpoints for merging");
    mergeLayout->addRow("Distance tolerance (mm):", linearFitMergeDistSpin);

    pageLayout->addWidget(mergeGroup);

    // Add stretch at the bottom
    pageLayout->addStretch();

    return page;
}

QWidget* AnalysisParamsDialog::createHoughPage()
{
    QWidget* page = new QWidget();
    QVBoxLayout* pageLayout = new QVBoxLayout(page);

    // Hough Transform parameters group
    QGroupBox* houghGroup = new QGroupBox("Hough Transform Parameters");
    QFormLayout* houghLayout = new QFormLayout(houghGroup);

    houghDThetaSpin = new QDoubleSpinBox();
    houghDThetaSpin->setRange(0.1, 10.0);
    houghDThetaSpin->setDecimals(1);
    houghDThetaSpin->setToolTip("Angle resolution in degrees");
    houghLayout->addRow("Angle step (°):", houghDThetaSpin);

    houghThetaBinsSpin = new QSpinBox();
    houghThetaBinsSpin->setRange(10, 360);
    houghThetaBinsSpin->setToolTip("Number of angle bins");
    houghLayout->addRow("Angle bins:", houghThetaBinsSpin);

    houghDRhoSpin = new QDoubleSpinBox();
    houghDRhoSpin->setRange(0.01, 10.0);
    houghDRhoSpin->setDecimals(2);
    houghDRhoSpin->setToolTip("Distance resolution in mm");
    houghLayout->addRow("Distance step (mm):", houghDRhoSpin);

    houghMinVotesSpin = new QDoubleSpinBox();
    houghMinVotesSpin->setRange(0.1, 100.0);
    houghMinVotesSpin->setDecimals(1);
    houghMinVotesSpin->setToolTip("Minimum votes as percentage of points");
    houghLayout->addRow("Min votes (%):", houghMinVotesSpin);

    houghMinPointsSpin = new QSpinBox();
    houghMinPointsSpin->setRange(2, 100);
    houghMinPointsSpin->setToolTip("Minimum points to form a line");
    houghLayout->addRow("Min points per line:", houghMinPointsSpin);

    pageLayout->addWidget(houghGroup);

    // Merge parameters group
    QGroupBox* mergeGroup = new QGroupBox("Merge Parameters");
    QFormLayout* mergeLayout = new QFormLayout(mergeGroup);

    houghMergeAngleSpin = new QDoubleSpinBox();
    houghMergeAngleSpin->setRange(0.1, 90.0);
    houghMergeAngleSpin->setDecimals(1);
    mergeLayout->addRow("Angle tolerance (°):", houghMergeAngleSpin);

    houghMergeDistSpin = new QDoubleSpinBox();
    houghMergeDistSpin->setRange(0.01, 50.0);
    houghMergeDistSpin->setDecimals(2);
    mergeLayout->addRow("Distance tolerance (mm):", houghMergeDistSpin);

    pageLayout->addWidget(mergeGroup);

    // Add stretch at the bottom
    pageLayout->addStretch();

    return page;
}

QWidget* AnalysisParamsDialog::createRANSACPage()
{
    QWidget* page = new QWidget();
    QVBoxLayout* pageLayout = new QVBoxLayout(page);

    // RANSAC parameters group
    QGroupBox* ransacGroup = new QGroupBox("RANSAC Parameters");
    QFormLayout* ransacLayout = new QFormLayout(ransacGroup);

    ransacMaxIterSpin = new QSpinBox();
    ransacMaxIterSpin->setRange(10, 10000);
    ransacMaxIterSpin->setToolTip("Maximum number of RANSAC iterations");
    ransacLayout->addRow("Max iterations:", ransacMaxIterSpin);

    ransacDistThreshSpin = new QDoubleSpinBox();
    ransacDistThreshSpin->setRange(0.01, 10.0);
    ransacDistThreshSpin->setDecimals(3);
    ransacDistThreshSpin->setToolTip("Distance threshold for inliers");
    ransacLayout->addRow("Distance threshold (mm):", ransacDistThreshSpin);

    ransacMinInliersSpin = new QSpinBox();
    ransacMinInliersSpin->setRange(2, 1000);
    ransacMinInliersSpin->setToolTip("Minimum number of inliers");
    ransacLayout->addRow("Min inliers:", ransacMinInliersSpin);

    ransacMinLineLengthSpin = new QDoubleSpinBox();
    ransacMinLineLengthSpin->setRange(0.1, 100.0);
    ransacMinLineLengthSpin->setDecimals(2);
    ransacMinLineLengthSpin->setToolTip("Minimum line length");
    ransacLayout->addRow("Min line length (mm):", ransacMinLineLengthSpin);

    ransacMaxLinesSpin = new QSpinBox();
    ransacMaxLinesSpin->setRange(1, 100);
    ransacMaxLinesSpin->setToolTip("Maximum lines to find");
    ransacLayout->addRow("Max lines:", ransacMaxLinesSpin);

    pageLayout->addWidget(ransacGroup);

    // Merge parameters group
    QGroupBox* mergeGroup = new QGroupBox("Merge Parameters");
    QFormLayout* mergeLayout = new QFormLayout(mergeGroup);

    ransacMergeAngleSpin = new QDoubleSpinBox();
    ransacMergeAngleSpin->setRange(0.1, 90.0);
    ransacMergeAngleSpin->setDecimals(1);
    mergeLayout->addRow("Angle tolerance (°):", ransacMergeAngleSpin);

    ransacMergeDistSpin = new QDoubleSpinBox();
    ransacMergeDistSpin->setRange(0.01, 50.0);
    ransacMergeDistSpin->setDecimals(2);
    mergeLayout->addRow("Distance tolerance (mm):", ransacMergeDistSpin);

    pageLayout->addWidget(mergeGroup);

    // Add stretch at the bottom
    pageLayout->addStretch();

    return page;
}

void AnalysisParamsDialog::loadDefaultValues()
{
    // Linear Fit defaults
    linearFitMinPointsSpin->setValue(3);
    linearFitResidualSpin->setValue(0.08);
    linearFitMergeAngleSpin->setValue(4.5);
    linearFitMergeDistSpin->setValue(0.4);

    // Hough defaults
    houghDThetaSpin->setValue(1.5);
    houghThetaBinsSpin->setValue(120);
    houghDRhoSpin->setValue(0.15);
    houghMinVotesSpin->setValue(15.0);
    houghMinPointsSpin->setValue(3);
    houghMergeAngleSpin->setValue(3.0);
    houghMergeDistSpin->setValue(0.5);

    // RANSAC defaults
    ransacMaxIterSpin->setValue(600);
    ransacDistThreshSpin->setValue(0.15);
    ransacMinInliersSpin->setValue(3);
    ransacMinLineLengthSpin->setValue(0.5);
    ransacMaxLinesSpin->setValue(25);
    ransacMergeAngleSpin->setValue(6.0);
    ransacMergeDistSpin->setValue(0.3);
}

void AnalysisParamsDialog::onMethodChanged(int index)
{
    paramsStack->setCurrentIndex(index);
}

void AnalysisParamsDialog::onDefaultsButtonClicked()
{
    loadDefaultValues();
}

LinearFitParams AnalysisParamsDialog::getLinearFitParams() const
{
    LinearFitParams params;
    params.minPointsPerSegment = linearFitMinPointsSpin->value();
    params.residualTolerance = linearFitResidualSpin->value();
    params.mergeAngleTolerance = linearFitMergeAngleSpin->value();
    params.mergeDistanceTolerance = linearFitMergeDistSpin->value();
    return params;
}

HoughParams AnalysisParamsDialog::getHoughParams() const
{
    HoughParams params;
    params.dTheta = houghDThetaSpin->value();
    params.thetaBins = houghThetaBinsSpin->value();
    params.dRho = houghDRhoSpin->value();
    params.minVotesPercent = houghMinVotesSpin->value();
    params.minPointsPerLine = houghMinPointsSpin->value();
    params.mergeAngleTolerance = houghMergeAngleSpin->value();
    params.mergeDistanceTolerance = houghMergeDistSpin->value();
    return params;
}

RANSACParams AnalysisParamsDialog::getRANSACParams() const
{
    RANSACParams params;
    params.maxIterations = ransacMaxIterSpin->value();
    params.distanceThreshold = ransacDistThreshSpin->value();
    params.minInliers = ransacMinInliersSpin->value();
    params.minLineLength = ransacMinLineLengthSpin->value();
    params.maxLinesToFind = ransacMaxLinesSpin->value();
    params.mergeAngleTolerance = ransacMergeAngleSpin->value();
    params.mergeDistanceTolerance = ransacMergeDistSpin->value();
    return params;
}

void AnalysisParamsDialog::setLinearFitParams(const LinearFitParams &params)
{
    linearFitMinPointsSpin->setValue(params.minPointsPerSegment);
    linearFitResidualSpin->setValue(params.residualTolerance);
    linearFitMergeAngleSpin->setValue(params.mergeAngleTolerance);
    linearFitMergeDistSpin->setValue(params.mergeDistanceTolerance);
}

void AnalysisParamsDialog::setHoughParams(const HoughParams &params)
{
    houghDThetaSpin->setValue(params.dTheta);
    houghThetaBinsSpin->setValue(params.thetaBins);
    houghDRhoSpin->setValue(params.dRho);
    houghMinVotesSpin->setValue(params.minVotesPercent);
    houghMinPointsSpin->setValue(params.minPointsPerLine);
    houghMergeAngleSpin->setValue(params.mergeAngleTolerance);
    houghMergeDistSpin->setValue(params.mergeDistanceTolerance);
}

void AnalysisParamsDialog::setRANSACParams(const RANSACParams &params)
{
    ransacMaxIterSpin->setValue(params.maxIterations);
    ransacDistThreshSpin->setValue(params.distanceThreshold);
    ransacMinInliersSpin->setValue(params.minInliers);
    ransacMinLineLengthSpin->setValue(params.minLineLength);
    ransacMaxLinesSpin->setValue(params.maxLinesToFind);
    ransacMergeAngleSpin->setValue(params.mergeAngleTolerance);
    ransacMergeDistSpin->setValue(params.mergeDistanceTolerance);
}

void AnalysisParamsDialog::setCurrentMethod(int methodIndex)
{
    if (methodIndex >= 0 && methodIndex < 3) {
        methodComboBox->setCurrentIndex(methodIndex);
        paramsStack->setCurrentIndex(methodIndex);
    }
}

int AnalysisParamsDialog::getCurrentMethod() const
{
    return methodComboBox->currentIndex();
}
