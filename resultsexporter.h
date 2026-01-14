#ifndef RESULTSEXPORTER_H
#define RESULTSEXPORTER_H

#include <QObject>
#include <QString>
#include "profileanalyzer.h"

class ResultsExporter : public QObject
{
    Q_OBJECT
public:
    explicit ResultsExporter(QObject* parent = nullptr);

    void setCurrentResult(const GeometryResult& res);

public slots:
    void exportToFile();

private:
    GeometryResult currentResult;
    QObject* parentObj;
};

#endif // RESULTSEXPORTER_H
