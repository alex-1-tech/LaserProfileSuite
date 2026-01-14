#include "mainwindow.h"
#include "profileanalyzer.h"

void AnalysisWorker::analyze()
{
    GeometryResult result = ProfileAnalyzer::analyzeGeometry(m_profile);
    emit analysisFinished(result);
}
