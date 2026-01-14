#include "resultsexporter.h"
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>

ResultsExporter::ResultsExporter(QObject* parent)
    : QObject(parent), parentObj(parent)
{
}

void ResultsExporter::setCurrentResult(const GeometryResult& res)
{
    currentResult = res;
}

void ResultsExporter::exportToFile()
{
    if (currentResult.segments.isEmpty() && currentResult.longLines.isEmpty()) {
        QMessageBox::warning(nullptr, "Export", "No results to export");
        return;
    }

    QString baseName = QFileDialog::getSaveFileName(nullptr,
                                                    "Save results (base name)",
                                                    "",
                                                    "CSV files (*.csv)");
    if (baseName.isEmpty()) return;

    // Ensure baseName has no extension or strip it
    if (baseName.endsWith(".csv", Qt::CaseInsensitive)) {
        baseName.chop(4);
    }

    QString segFile = baseName + "_segments.csv";
    QString lineFile = baseName + "_lines.csv";

    // Export segments
    {
        QFile file(segFile);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(nullptr, "Export error", QString("Cannot open %1 for writing").arg(segFile));
            return;
        }
        QTextStream out(&file);
        out.setCodec("UTF-8");

        // English header
        out << "ID,Type,Length_mm,Angle_deg,StartX,EndX,StartZ,EndZ,PointCount,IsLongLine\n";

        for (const auto& s : currentResult.segments) {
            out << s.id << ","
                << s.type << ","
                << QString::number(s.length, 'f', 6) << ","
                << QString::number(s.angle, 'f', 3) << ","
                << QString::number(s.startX, 'f', 6) << ","
                << QString::number(s.endX, 'f', 6) << ","
                << QString::number(s.startZ, 'f', 6) << ","
                << QString::number(s.endZ, 'f', 6) << ","
                << s.pointCount << ","
                << (s.isLongLine ? "Yes" : "No") << "\n";
        }
        file.close();
    }

    // Export lines (merged)
    {
        QFile file(lineFile);
        if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
            QMessageBox::critical(nullptr, "Export error", QString("Cannot open %1 for writing").arg(lineFile));
            return;
        }
        QTextStream out(&file);
        out.setCodec("UTF-8");

        out << "ID,Type,Length_mm,Angle_deg,StartX,EndX,StartZ,EndZ,SegmentsMerged,PointCount\n";

        for (const auto& s : currentResult.longLines) {
            out << s.id << ","
                << s.type << ","
                << QString::number(s.length, 'f', 6) << ","
                << QString::number(s.angle, 'f', 3) << ","
                << QString::number(s.startX, 'f', 6) << ","
                << QString::number(s.endX, 'f', 6) << ","
                << QString::number(s.startZ, 'f', 6) << ","
                << QString::number(s.endZ, 'f', 6) << ","
                << "\"" << QString::number(s.mergedSegmentIds.size()) << "\","
                << s.pointCount << "\n";
        }
        file.close();
    }

    QMessageBox::information(nullptr, "Export", QString("Exported:\n%1\n%2").arg(segFile).arg(lineFile));
}
