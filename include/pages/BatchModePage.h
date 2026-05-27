/*
 * 文件说明：声明批处理页面 `BatchModePage`，仅保留页面层接口与界面成员。
 */
#pragma once
#include <QWidget>
#include "pages/batch_mode/BatchModeTypes.h"
#include "pages/batch_mode/BatchWorker.h"
class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLineEdit;
class QProgressBar;
class QPushButton;
class QSpinBox;
class QTextEdit;
class BatchModePage : public QWidget {
    Q_OBJECT
public:
    explicit BatchModePage(QWidget *parent = nullptr);
    ~BatchModePage() override;
private slots:
    void onBrowseInput();
    void onBrowseOutput();
    void onStartBatch();
    void onStopBatch();
    void onWorkerProgress(int current, int total);
    void onWorkerLog(const QString& msg, const QString& type);
    void onWorkerFinished(int successCount, int totalCount);
private:
    void initUI();
    BatchParams collectParams();
    QLineEdit *m_leInput = nullptr;
    QLineEdit *m_leOutput = nullptr;
    QDoubleSpinBox *m_spinLeaf = nullptr;
    QDoubleSpinBox *m_spinStd = nullptr;
    QSpinBox *m_spinMeanK = nullptr;
    QDoubleSpinBox *m_spinClip = nullptr;
    QComboBox *m_comboRegAlgo = nullptr;
    QSpinBox *m_spinIcpIter = nullptr;
    QDoubleSpinBox *m_spinIcpDist = nullptr;
    QDoubleSpinBox *m_spinNdtRes = nullptr;
    QDoubleSpinBox *m_spinNdtStep = nullptr;
    QSpinBox *m_spinNdtIter = nullptr;
    QDoubleSpinBox *m_spinGicpDist = nullptr;
    QSpinBox *m_spinGicpIter = nullptr;
    QDoubleSpinBox *m_spinBoxMinX = nullptr;
    QDoubleSpinBox *m_spinBoxMinY = nullptr;
    QDoubleSpinBox *m_spinBoxMinZ = nullptr;
    QDoubleSpinBox *m_spinBoxMaxX = nullptr;
    QDoubleSpinBox *m_spinBoxMaxY = nullptr;
    QDoubleSpinBox *m_spinBoxMaxZ = nullptr;
    QDoubleSpinBox *m_spinBoxRotZ = nullptr;
    QSpinBox *m_spinExtMinPts = nullptr;
    QComboBox *m_comboExtMethod = nullptr;
    QDoubleSpinBox *m_spinEuclideanTol = nullptr;
    QSpinBox *m_spinRgNeighbors = nullptr;
    QDoubleSpinBox *m_spinRgSmoothness = nullptr;
    QCheckBox *m_chkUseRansac = nullptr;
    QDoubleSpinBox *m_spinRansacDist = nullptr;
    QCheckBox *m_chkOnlyExtract = nullptr;
    QCheckBox *m_chkUseMls = nullptr;
    QDoubleSpinBox *m_spinMlsSearchRadius = nullptr;
    QDoubleSpinBox *m_spinMlsUpsampleRadius = nullptr;
    QDoubleSpinBox *m_spinMlsUpsampleStep = nullptr;
    QDoubleSpinBox *m_spinGirthThick = nullptr;
    QDoubleSpinBox *m_spinSkelStep = nullptr;
    QDoubleSpinBox *m_spinSkelRadius = nullptr;
    QDoubleSpinBox *m_spinHeightAngle = nullptr;
    QPushButton *m_btnStart = nullptr;
    QPushButton *m_btnStop = nullptr;
    QProgressBar *m_progressBar = nullptr;
    QTextEdit *m_console = nullptr;
    BatchWorker *m_worker = nullptr;
};
