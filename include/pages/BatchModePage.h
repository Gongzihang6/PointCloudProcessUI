#pragma once

#include <QWidget>
#include <QString>
#include <QThread>
#include <QMap>
#include <QCheckBox>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "core/PointCloudAlgo.h" // 包含底层的算法库

class QLineEdit;
class QComboBox;
class QDoubleSpinBox;
class QSpinBox;
class QProgressBar;
class QTextEdit;
class QPushButton;
class QCheckBox;
// ==========================================================
// 1. 全局参数结构体：统一管理所有批处理参数
// ==========================================================
struct BatchParams {
    // IO 路径
    QString inputDir;
    QString outputDir;

    // 预处理参数
    float leafSize = 10.0f;
    double stdDev = 2.0;
    int meanK = 50;
    float clipRadius = 2500.0f;

    // 配准参数
    int regMethod = 0; // 0: 手动矩阵, 1: ICP(点到点), 2: ICP(点到面), 3: NDT
    // ICP 参数
    int icpIter = 60;
    double icpDist = 100.0;
    // NDT 参数
    float ndtRes = 100.0f;
    float ndtStep = 0.1f;
    int ndtIter = 35;
    // G-ICP 参数
    int gicpIter = 50;
    double gicpDist = 50.0;
    double gicpEps = 1e-8;


    // 主体提取参数
    float boxMinX = -1200.0f, boxMinY = -460.0f, boxMinZ = -500.0f;
    float boxMaxX = 500.0f,  boxMaxY = 170.0f,  boxMaxZ = 2100.0f;
    float boxRotZ = 33.0f;
    int minClusterSize = 5000;
    int extMethodIndex = 0; // 0: 欧式, 1: 区域生长
    double extEuclideanTol = 40.0;
    int extRgNeighbors = 30;
    double extRgSmoothness = 7.0;
    bool useRansac = false;         // RANSAC 开关
    double ransacDistThresh = 20.0; // RANSAC 阈值
    bool onlyExtractBody = false;   // [新增功能] 仅提取主体模式开关
    bool useMlsUpsampling = true;
    double mlsSearchRadius = 80.0;
    double mlsUpsamplingRadius = 25.0;
    double mlsUpsamplingStep = 25.0;

    // 测量参数
    float girthThick = 10.0f;
    float skelStep = 20.0f;
    float skelRadius = 30.0f;
    float heightAngle = 15.0f;
    
    // AI 服务地址
    QString aiEndpoint = "http://127.0.0.1:8000/predict";
};

// ==========================================================
// 2. 后台工作线程：防止 UI 卡死
// ==========================================================
class BatchWorker : public QThread {
    Q_OBJECT
public:
    BatchWorker(const QStringList& folders, const BatchParams& params, QObject* parent = nullptr);
    ~BatchWorker();

    void stop(); // 提供中断接口

signals:
    void progressUpdated(int current, int total);
    void logMessage(const QString& msg, const QString& type);
    void batchFinished(int successCount, int totalCount);

protected:
    void run() override;

private:
    QStringList m_folders;
    BatchParams m_params;
    // [修改] 使用原子变量，防止多线程下的缓存不一致导致标志位读取失败
    std::atomic<bool> m_stopFlag{false};

    // 辅助函数
    QMap<QString, Eigen::Matrix4d> getDefaultTransforms();
    void getCameraColor(const QString& camName, int& r, int& g, int& b);
};

// ==========================================================
// 3. UI 页面类
// ==========================================================
class BatchModePage : public QWidget {
    Q_OBJECT
public:
    explicit BatchModePage(QWidget *parent = nullptr);
    ~BatchModePage();

private slots:
    void onBrowseInput();
    void onBrowseOutput();
    void onStartBatch();
    void onStopBatch(); // [新增] 停止批处理槽函数
    void onWorkerProgress(int current, int total);
    void onWorkerLog(const QString& msg, const QString& type);
    void onWorkerFinished(int successCount, int totalCount);

private:
    void initUI();
    BatchParams collectParams(); // 从界面收集参数到结构体

    // UI 控件
    QLineEdit *m_leInput;       // 输入文件夹路径
    QLineEdit *m_leOutput;      // 输出文件夹路径
    
    // 预处理参数控件
    QDoubleSpinBox *m_spinLeaf;     // 体素下采样参数
    QDoubleSpinBox *m_spinStd;      // 统计滤波参数
    QSpinBox       *m_spinMeanK;    // 统计滤波参数
    QDoubleSpinBox *m_spinClip;     // 距离裁剪参数

    // 配准参数控件
    QComboBox      *m_comboRegAlgo;
    QSpinBox       *m_spinIcpIter;
    QDoubleSpinBox *m_spinIcpDist;
    QDoubleSpinBox *m_spinNdtRes;
    QDoubleSpinBox *m_spinNdtStep;
    QSpinBox       *m_spinNdtIter;
    QDoubleSpinBox *m_spinGicpDist;
    QSpinBox       *m_spinGicpIter;

    // 主体提取参数控件
    QDoubleSpinBox *m_spinBoxMinX, *m_spinBoxMinY, *m_spinBoxMinZ;
    QDoubleSpinBox *m_spinBoxMaxX, *m_spinBoxMaxY, *m_spinBoxMaxZ;
    QDoubleSpinBox *m_spinBoxRotZ;     // [新增]
    QSpinBox       *m_spinExtMinPts;
    QComboBox      *m_comboExtMethod;
    QDoubleSpinBox *m_spinEuclideanTol;
    QSpinBox       *m_spinRgNeighbors;
    QDoubleSpinBox *m_spinRgSmoothness;
    QCheckBox      *m_chkUseRansac;    // [新增]
    QDoubleSpinBox *m_spinRansacDist;  // [新增]
    QCheckBox      *m_chkOnlyExtract;  // [新增功能] 模式选择框
    QCheckBox *m_chkUseMls;
    QDoubleSpinBox *m_spinMlsSearchRadius;
    QDoubleSpinBox *m_spinMlsUpsampleRadius;
    QDoubleSpinBox *m_spinMlsUpsampleStep;

    // 体尺测量参数控件
    QDoubleSpinBox *m_spinGirthThick;
    QDoubleSpinBox *m_spinSkelStep;
    QDoubleSpinBox *m_spinSkelRadius;
    QDoubleSpinBox *m_spinHeightAngle;

    // 流程控制按钮和显示控件
    QPushButton  *m_btnStart;
    QPushButton  *m_btnStop; // [新增] 停止按钮指针
    QProgressBar *m_progressBar;
    QTextEdit    *m_console;

    BatchWorker  *m_worker = nullptr;
};