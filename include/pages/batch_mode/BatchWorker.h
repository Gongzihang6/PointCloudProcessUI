/*
 * 文件说明：声明批处理后台线程 `BatchWorker`。
 */
#pragma once
#include <QMap>
#include <QThread>
#include <QString>
#include <QStringList>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <atomic>
#include "pages/batch_mode/BatchModeTypes.h"
class BatchWorker : public QThread {
    Q_OBJECT
public:
    BatchWorker(const QStringList& folders, const BatchParams& params, QObject* parent = nullptr);
    ~BatchWorker() override;
    void stop();
signals:
    void progressUpdated(int current, int total);
    void logMessage(const QString& msg, const QString& type);
    void batchFinished(int successCount, int totalCount);
protected:
    void run() override;
private:
    QMap<QString, Eigen::Matrix4d> getDefaultTransforms();
    void getCameraColor(const QString& camName, int& r, int& g, int& b);
    QStringList m_folders;
    BatchParams m_params;
    std::atomic<bool> m_stopFlag{false};
};
