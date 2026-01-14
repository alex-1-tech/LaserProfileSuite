#ifndef ANALYSISPARAMSDIALOG_H
#define ANALYSISPARAMSDIALOG_H

#include <QDialog>
#include <QStackedWidget>
#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QGroupBox>
#include <QFormLayout>
#include <QDialogButtonBox>
#include "segmentinfo.h"

/**
 * @class AnalysisParamsDialog
 * @brief Dialog for configuring analysis parameters
 *
 * Provides UI for setting parameters for all analysis methods:
 * - Linear Fit
 * - Hough Transform
 * - RANSAC
 */
class AnalysisParamsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit AnalysisParamsDialog(QWidget *parent = nullptr);

    // Getters for parameters
    LinearFitParams getLinearFitParams() const;
    HoughParams getHoughParams() const;
    RANSACParams getRANSACParams() const;

    // Setters for parameters
    void setLinearFitParams(const LinearFitParams &params);
    void setHoughParams(const HoughParams &params);
    void setRANSACParams(const RANSACParams &params);

    // Method selection
    void setCurrentMethod(int methodIndex);
    int getCurrentMethod() const;

private slots:
    void onMethodChanged(int index);
    void onDefaultsButtonClicked();

private:
    void setupUI();
    QWidget* createLinearFitPage();
    QWidget* createHoughPage();
    QWidget* createRANSACPage();
    void loadDefaultValues();

    // UI Components
    QComboBox* methodComboBox;
    QStackedWidget* paramsStack;
    QDialogButtonBox* buttonBox;
    QPushButton* defaultsButton;

    // Linear Fit widgets
    QSpinBox* linearFitMinPointsSpin;
    QDoubleSpinBox* linearFitResidualSpin;
    QDoubleSpinBox* linearFitMergeAngleSpin;
    QDoubleSpinBox* linearFitMergeDistSpin;

    // Hough widgets
    QDoubleSpinBox* houghDThetaSpin;
    QSpinBox* houghThetaBinsSpin;
    QDoubleSpinBox* houghDRhoSpin;
    QDoubleSpinBox* houghMinVotesSpin;
    QSpinBox* houghMinPointsSpin;
    QDoubleSpinBox* houghMergeAngleSpin;
    QDoubleSpinBox* houghMergeDistSpin;

    // RANSAC widgets
    QSpinBox* ransacMaxIterSpin;
    QDoubleSpinBox* ransacDistThreshSpin;
    QSpinBox* ransacMinInliersSpin;
    QDoubleSpinBox* ransacMinLineLengthSpin;
    QSpinBox* ransacMaxLinesSpin;
    QDoubleSpinBox* ransacMergeAngleSpin;
    QDoubleSpinBox* ransacMergeDistSpin;
};

#endif // ANALYSISPARAMSDIALOG_H
