#include "pages/BatchModePage.h"
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QComboBox>
#include <QProgressBar>
#include <QTextEdit>
#include <QFileDialog>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <QMessageBox>
#include <QScrollBar>
#include <QScrollArea>

// 网络与多线程模块
#include <QNetworkAccessManager>
#include <QNetworkRequest>
#include <QNetworkReply>
#include <QHttpMultiPart>
#include <QEventLoop>
#include <QtConcurrent>
#include <QFuture>

// PCL 相关
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d_omp.h>

#include <array>
#include <exception>
#include <atomic>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// ==========================================================
// BatchModePage (UI 界面实现)
// ==========================================================

BatchModePage::BatchModePage(QWidget *parent) : QWidget(parent) {
    initUI();
}

BatchModePage::~BatchModePage() {
    if (m_worker && m_worker->isRunning()) {
        m_worker->stop();
        m_worker->wait();
    }
}

void BatchModePage::initUI() {
    QVBoxLayout *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(20, 20, 20, 20);
    mainLayout->setSpacing(15);

    // --- 1. 顶部 IO 配置区 ---
    QGroupBox *grpIO = new QGroupBox("📂 批处理配置 (IO Configuration)");
    QGridLayout *ioLay = new QGridLayout(grpIO);
    
    m_leInput = new QLineEdit(); m_leInput->setPlaceholderText("请选择包含时间戳文件夹的 Input 目录...");
    QPushButton *btnInput = new QPushButton("选择...");
    connect(btnInput, &QPushButton::clicked, this, &BatchModePage::onBrowseInput);
    
    m_leOutput = new QLineEdit(); m_leOutput->setPlaceholderText("请选择处理结果保存的 Output 目录...");
    QPushButton *btnOutput = new QPushButton("选择...");
    connect(btnOutput, &QPushButton::clicked, this, &BatchModePage::onBrowseOutput);

    ioLay->addWidget(new QLabel("输入目录:"), 0, 0); ioLay->addWidget(m_leInput, 0, 1); ioLay->addWidget(btnInput, 0, 2);
    ioLay->addWidget(new QLabel("输出目录:"), 1, 0); ioLay->addWidget(m_leOutput, 1, 1); ioLay->addWidget(btnOutput, 1, 2);
    // [新增功能] 仅提取模式开关
    m_chkOnlyExtract = new QCheckBox("⚡ 仅批量提取主体与融合 (跳过后续 AI 关键点与体尺测量)");
    m_chkOnlyExtract->setStyleSheet("font-weight: bold; color: #E6A23C; margin-top: 5px;");
    ioLay->addWidget(m_chkOnlyExtract, 2, 0, 1, 3); // 放在第 2 行


    m_btnStart = new QPushButton("🚀 开始批量处理");
    m_btnStart->setMinimumHeight(40);
    m_btnStart->setStyleSheet("font-weight: bold; font-size: 14px; background-color: #409EFF; color: white; border-radius: 4px;");
    connect(m_btnStart, &QPushButton::clicked, this, &BatchModePage::onStartBatch);
    
    // [新增] 停止按钮
    m_btnStop = new QPushButton("🛑 停止处理");
    m_btnStop->setMinimumHeight(40);
    m_btnStop->setStyleSheet("font-weight: bold; font-size: 14px; background-color: #F56C6C; color: white; border-radius: 4px;");
    m_btnStop->setEnabled(false); // 初始状态为禁用，只有开始运行后才启用
    connect(m_btnStop, &QPushButton::clicked, this, &BatchModePage::onStopBatch);

    // [新增] 将两个按钮放在水平布局中，占据底部的空间
    QHBoxLayout *btnLay = new QHBoxLayout();
    btnLay->addWidget(m_btnStart);
    btnLay->addWidget(m_btnStop);
    ioLay->addLayout(btnLay, 3, 0, 1, 3);
    mainLayout->addWidget(grpIO, 0);

    // --- 2. 中间全局参数配置区 (放入滚动区) ---
    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    QWidget *scrollContent = new QWidget();
    QHBoxLayout *paramLayout = new QHBoxLayout(scrollContent); // 横向分栏
    
    // 栏目 A: 预处理与配准
    QVBoxLayout *colA = new QVBoxLayout();
    QGroupBox *grpPre = new QGroupBox("1. 预处理参数");
    QGridLayout *preLay = new QGridLayout(grpPre);
    m_spinLeaf = new QDoubleSpinBox(); m_spinLeaf->setValue(10.0); preLay->addWidget(new QLabel("下采样(mm):"), 0,0); preLay->addWidget(m_spinLeaf, 0,1);
    m_spinStd = new QDoubleSpinBox(); m_spinStd->setValue(2.0); preLay->addWidget(new QLabel("SOR Std倍数:"), 1,0); preLay->addWidget(m_spinStd, 1,1);
    m_spinMeanK = new QSpinBox(); m_spinMeanK->setValue(50); m_spinMeanK->setMaximum(500); preLay->addWidget(new QLabel("SOR 邻近点数:"), 2,0); preLay->addWidget(m_spinMeanK, 2,1);
    m_spinClip = new QDoubleSpinBox(); m_spinClip->setRange(100, 10000); m_spinClip->setValue(2500); preLay->addWidget(new QLabel("背景裁剪(mm):"), 3,0); preLay->addWidget(m_spinClip, 3,1);
    colA->addWidget(grpPre);

    QGroupBox *grpReg = new QGroupBox("2. 配准参数");
    QGridLayout *regLay = new QGridLayout(grpReg);
    m_comboRegAlgo = new QComboBox(); m_comboRegAlgo->addItems({"手动矩阵", "ICP (P2Point)", "ICP (P2Plane)", "NDT 微调", "G-ICP"});
    regLay->addWidget(new QLabel("算法:"), 0,0); regLay->addWidget(m_comboRegAlgo, 0,1);
    
    m_spinIcpIter = new QSpinBox(); m_spinIcpIter->setValue(60); m_spinIcpIter->setMaximum(500); regLay->addWidget(new QLabel("ICP 迭代次数:"), 1,0); regLay->addWidget(m_spinIcpIter, 1,1);
    m_spinIcpDist = new QDoubleSpinBox(); m_spinIcpDist->setValue(100.0); m_spinIcpDist->setMaximum(1000); regLay->addWidget(new QLabel("ICP 对应距离:"), 2,0); regLay->addWidget(m_spinIcpDist, 2,1);
    
    m_spinNdtRes = new QDoubleSpinBox(); m_spinNdtRes->setValue(100.0); m_spinNdtRes->setMaximum(500); regLay->addWidget(new QLabel("NDT 分辨率:"), 3,0); regLay->addWidget(m_spinNdtRes, 3,1);
    
    // =========================================================
    // [新增修复] 补全 NDT 步长和迭代次数的 UI 实例化
    // =========================================================
    m_spinNdtStep = new QDoubleSpinBox(); 
    m_spinNdtStep->setValue(0.1); 
    m_spinNdtStep->setSingleStep(0.1); 
    regLay->addWidget(new QLabel("NDT 步长:"), 4, 0); 
    regLay->addWidget(m_spinNdtStep, 4, 1);

    m_spinNdtIter = new QSpinBox(); 
    m_spinNdtIter->setValue(35); 
    m_spinNdtIter->setMaximum(500); 
    regLay->addWidget(new QLabel("NDT 迭代次数:"), 5, 0); 
    regLay->addWidget(m_spinNdtIter, 5, 1);
    // =========================================================

    // =========================================================
    // [核心修复] 实例化缺失的 G-ICP 控件，终结闪退！
    // =========================================================
    m_spinGicpIter = new QSpinBox(); 
    m_spinGicpIter->setValue(50); 
    m_spinGicpIter->setMaximum(500); 
    regLay->addWidget(new QLabel("G-ICP 迭代次数:"), 6, 0); 
    regLay->addWidget(m_spinGicpIter, 6, 1);

    m_spinGicpDist = new QDoubleSpinBox(); 
    m_spinGicpDist->setValue(50.0); 
    m_spinGicpDist->setMaximum(1000.0); 
    regLay->addWidget(new QLabel("G-ICP 对应距离:"), 7, 0); 
    regLay->addWidget(m_spinGicpDist, 7, 1);
    // =========================================================

    colA->addWidget(grpReg);
    paramLayout->addLayout(colA);

    // 栏目 B: 提取与测量
    QVBoxLayout *colB = new QVBoxLayout();
    QGroupBox *grpExt = new QGroupBox("3. 提取参数");
    QGridLayout *extLay = new QGridLayout(grpExt);
    int r = 0;

    // 共用参数: 包围盒
    QHBoxLayout* minLay = new QHBoxLayout;
    m_spinBoxMinX = new QDoubleSpinBox(); m_spinBoxMinX->setRange(-3000,3000); m_spinBoxMinX->setValue(-1200); m_spinBoxMinX->setDecimals(0);
    m_spinBoxMinY = new QDoubleSpinBox(); m_spinBoxMinY->setRange(-3000,3000); m_spinBoxMinY->setValue(-460); m_spinBoxMinY->setDecimals(0);
    m_spinBoxMinZ = new QDoubleSpinBox(); m_spinBoxMinZ->setRange(-3000,3000); m_spinBoxMinZ->setValue(-500); m_spinBoxMinZ->setDecimals(0);
    minLay->addWidget(m_spinBoxMinX); minLay->addWidget(m_spinBoxMinY); minLay->addWidget(m_spinBoxMinZ);
    extLay->addWidget(new QLabel("Box Min(X/Y/Z):"), r, 0); extLay->addLayout(minLay, r, 1); r++;

    QHBoxLayout* maxLay = new QHBoxLayout;
    m_spinBoxMaxX = new QDoubleSpinBox(); m_spinBoxMaxX->setRange(-3000,3000); m_spinBoxMaxX->setValue(600); m_spinBoxMaxX->setDecimals(0);
    m_spinBoxMaxY = new QDoubleSpinBox(); m_spinBoxMaxY->setRange(-3000,3000); m_spinBoxMaxY->setValue(170); m_spinBoxMaxY->setDecimals(0);
    m_spinBoxMaxZ = new QDoubleSpinBox(); m_spinBoxMaxZ->setRange(-3000,3000); m_spinBoxMaxZ->setValue(2100); m_spinBoxMaxZ->setDecimals(0);
    maxLay->addWidget(m_spinBoxMaxX); maxLay->addWidget(m_spinBoxMaxY); maxLay->addWidget(m_spinBoxMaxZ);
    extLay->addWidget(new QLabel("Box Max(X/Y/Z):"), r, 0); extLay->addLayout(maxLay, r, 1); r++;

    // [新增] Z轴旋转
    m_spinBoxRotZ = new QDoubleSpinBox(); m_spinBoxRotZ->setRange(-180, 180); m_spinBoxRotZ->setValue(33.0);
    extLay->addWidget(new QLabel("Z轴旋转(度):"), r, 0); extLay->addWidget(m_spinBoxRotZ, r, 1); r++;

    m_spinExtMinPts = new QSpinBox(); m_spinExtMinPts->setRange(100, 100000); m_spinExtMinPts->setValue(5000);
    extLay->addWidget(new QLabel("最小簇点数:"), r, 0); extLay->addWidget(m_spinExtMinPts, r, 1); r++;

    m_comboExtMethod = new QComboBox(); m_comboExtMethod->addItems({"欧式聚类", "区域生长"});
    extLay->addWidget(new QLabel("聚类算法:"), r, 0); extLay->addWidget(m_comboExtMethod, r, 1); r++;

    m_spinEuclideanTol = new QDoubleSpinBox(); m_spinEuclideanTol->setRange(1, 200); m_spinEuclideanTol->setValue(40.0);
    extLay->addWidget(new QLabel("欧式容差(mm):"), r, 0); extLay->addWidget(m_spinEuclideanTol, r, 1); r++;

    m_spinRgNeighbors = new QSpinBox(); m_spinRgNeighbors->setRange(5, 100); m_spinRgNeighbors->setValue(30);
    extLay->addWidget(new QLabel("RG邻居数:"), r, 0); extLay->addWidget(m_spinRgNeighbors, r, 1); r++;

    m_spinRgSmoothness = new QDoubleSpinBox(); m_spinRgSmoothness->setRange(1, 45); m_spinRgSmoothness->setValue(7.0);
    extLay->addWidget(new QLabel("RG平滑度(度):"), r, 0); extLay->addWidget(m_spinRgSmoothness, r, 1); r++;

    // [新增] RANSAC 面板
    m_chkUseRansac = new QCheckBox("开启 RANSAC 剔除平面");
    m_spinRansacDist = new QDoubleSpinBox(); m_spinRansacDist->setRange(1.0, 100.0); m_spinRansacDist->setValue(20.0); m_spinRansacDist->setEnabled(false);
    connect(m_chkUseRansac, &QCheckBox::toggled, m_spinRansacDist, &QDoubleSpinBox::setEnabled);
    QHBoxLayout* ransacLay = new QHBoxLayout; ransacLay->addWidget(m_chkUseRansac); ransacLay->addWidget(new QLabel("阈值:")); ransacLay->addWidget(m_spinRansacDist);
    extLay->addLayout(ransacLay, r, 0, 1, 2); r++;

    colB->addWidget(grpExt);

    // 在 initUI() 的 colB 布局中，测量参数组之前添加
    QGroupBox *grpMls = new QGroupBox("3.5 MLS 平滑与上采样");
    QGridLayout *mlsLay = new QGridLayout(grpMls);

    m_chkUseMls = new QCheckBox("开启 MLS 上采样补孔");
    m_chkUseMls->setChecked(true);
    mlsLay->addWidget(m_chkUseMls, 0, 0, 1, 2);

    m_spinMlsSearchRadius = new QDoubleSpinBox(); m_spinMlsSearchRadius->setRange(1, 500); m_spinMlsSearchRadius->setValue(80.0);
    mlsLay->addWidget(new QLabel("搜索半径(mm):"), 1, 0); mlsLay->addWidget(m_spinMlsSearchRadius, 1, 1);

    m_spinMlsUpsampleRadius = new QDoubleSpinBox(); m_spinMlsUpsampleRadius->setRange(1, 500); m_spinMlsUpsampleRadius->setValue(25.0);
    mlsLay->addWidget(new QLabel("补孔半径(mm):"), 2, 0); mlsLay->addWidget(m_spinMlsUpsampleRadius, 2, 1);

    m_spinMlsUpsampleStep = new QDoubleSpinBox(); m_spinMlsUpsampleStep->setRange(1, 100); m_spinMlsUpsampleStep->setValue(25.0);
    mlsLay->addWidget(new QLabel("补孔步长(mm):"), 3, 0); mlsLay->addWidget(m_spinMlsUpsampleStep, 3, 1);

    // 联动禁用
    connect(m_chkUseMls, &QCheckBox::toggled, [this](bool checked){
        m_spinMlsUpsampleRadius->setEnabled(checked);
        m_spinMlsUpsampleStep->setEnabled(checked);
    });

    colB->addWidget(grpMls);



    QGroupBox *grpMeas = new QGroupBox("4. 测量参数");
    QGridLayout *measLay = new QGridLayout(grpMeas);
    m_spinGirthThick = new QDoubleSpinBox(); m_spinGirthThick->setValue(10.0); measLay->addWidget(new QLabel("周长切片厚度:"), 0,0); measLay->addWidget(m_spinGirthThick, 0,1);
    m_spinSkelStep = new QDoubleSpinBox(); m_spinSkelStep->setValue(20.0); measLay->addWidget(new QLabel("骨架采样步长:"), 1,0); measLay->addWidget(m_spinSkelStep, 1,1);
    m_spinSkelRadius = new QDoubleSpinBox(); m_spinSkelRadius->setValue(30.0); measLay->addWidget(new QLabel("骨架搜索半径:"), 2,0); measLay->addWidget(m_spinSkelRadius, 2,1);
    m_spinHeightAngle = new QDoubleSpinBox(); m_spinHeightAngle->setValue(15.0); measLay->addWidget(new QLabel("地面法向容差:"), 3,0); measLay->addWidget(m_spinHeightAngle, 3,1);
    colB->addWidget(grpMeas);
    paramLayout->addLayout(colB);

    scroll->setWidget(scrollContent);
    mainLayout->addWidget(scroll, 1);

    // --- 3. 底部状态区 ---
    m_progressBar = new QProgressBar();
    m_progressBar->setValue(0);
    m_progressBar->setAlignment(Qt::AlignCenter);
    mainLayout->addWidget(m_progressBar, 0);

    m_console = new QTextEdit();
    m_console->setReadOnly(true);
    m_console->setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas; font-size: 12px;");
    mainLayout->addWidget(m_console, 2);
}

void BatchModePage::onBrowseInput() {
    QString dir = QFileDialog::getExistingDirectory(this, "选择 Input 文件夹");
    if (!dir.isEmpty()) m_leInput->setText(dir);
}

void BatchModePage::onBrowseOutput() {
    QString dir = QFileDialog::getExistingDirectory(this, "选择 Output 文件夹");
    if (!dir.isEmpty()) m_leOutput->setText(dir);
}

BatchParams BatchModePage::collectParams() {
    BatchParams p;

    // 输入输出目录
    p.inputDir = m_leInput->text();
    p.outputDir = m_leOutput->text();

    // 预处理参数
    p.leafSize = m_spinLeaf->value();
    p.stdDev = m_spinStd->value();
    p.meanK = m_spinMeanK->value();
    p.clipRadius = m_spinClip->value();

    // 配准参数
    p.regMethod = m_comboRegAlgo->currentIndex();
    p.icpIter = m_spinIcpIter->value();
    p.icpDist = m_spinIcpDist->value();
    p.ndtRes = m_spinNdtRes->value();
    p.ndtStep = m_spinNdtStep->value();
    p.ndtIter = m_spinNdtIter->value();
    p.gicpIter = m_spinGicpIter->value();
    p.gicpDist = m_spinGicpDist->value();
    p.gicpEps  = 1e-8; 

    // 主体提取参数
    p.boxMinX = m_spinBoxMinX->value();
    p.boxMinY = m_spinBoxMinY->value();
    p.boxMinZ = m_spinBoxMinZ->value();
    p.boxMaxX = m_spinBoxMaxX->value();
    p.boxMaxY = m_spinBoxMaxY->value();
    p.boxMaxZ = m_spinBoxMaxZ->value();
    p.boxRotZ = m_spinBoxRotZ->value();
    p.minClusterSize = m_spinExtMinPts->value();
    p.extMethodIndex = m_comboExtMethod->currentIndex();
    p.extEuclideanTol = m_spinEuclideanTol->value();
    p.extRgNeighbors = m_spinRgNeighbors->value();
    p.extRgSmoothness = m_spinRgSmoothness->value();
    p.useRansac = m_chkUseRansac->isChecked();       // 新增
    p.ransacDistThresh = m_spinRansacDist->value();  // 新增
    p.onlyExtractBody = m_chkOnlyExtract->isChecked(); // 新增
    p.useMlsUpsampling = m_chkUseMls->isChecked();
    p.mlsSearchRadius = m_spinMlsSearchRadius->value();
    p.mlsUpsamplingRadius = m_spinMlsUpsampleRadius->value();
    p.mlsUpsamplingStep = m_spinMlsUpsampleStep->value();


    p.girthThick = m_spinGirthThick->value();
    p.skelStep = m_spinSkelStep->value();
    p.skelRadius = m_spinSkelRadius->value();
    p.heightAngle = m_spinHeightAngle->value();
    return p;
}

void BatchModePage::onStartBatch() {
    BatchParams params = collectParams();
    
    if (params.inputDir.isEmpty() || params.outputDir.isEmpty()) {
        QMessageBox::warning(this, "警告", "请先设置输入和输出目录！");
        return;
    }

    // 扫描输入目录下的所有子文件夹
    QDir inDir(params.inputDir);
    QStringList subDirs = inDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    
    if (subDirs.isEmpty()) {
        QMessageBox::information(this, "提示", "输入目录中未找到任何子文件夹。");
        return;
    }

    QStringList folderPaths;
    for (const QString& d : subDirs) {
        folderPaths << inDir.absoluteFilePath(d);
    }

    // 初始化 UI 状态
    m_btnStart->setEnabled(false);
    m_btnStart->setText("⏳ 批处理运行中...");
    // [新增] 运行批处理时，激活停止按钮
    m_btnStop->setEnabled(true);
    m_btnStop->setText("🛑 停止处理");
    m_progressBar->setMaximum(folderPaths.size());
    m_progressBar->setValue(0);
    m_console->clear();
    onWorkerLog(QString("扫描到 %1 个待处理文件夹。").arg(folderPaths.size()), "INFO");

    // 启动后台线程
    m_worker = new BatchWorker(folderPaths, params, this);
    connect(m_worker, &BatchWorker::progressUpdated, this, &BatchModePage::onWorkerProgress);
    connect(m_worker, &BatchWorker::logMessage, this, &BatchModePage::onWorkerLog);
    connect(m_worker, &BatchWorker::batchFinished, this, &BatchModePage::onWorkerFinished);
    
    m_worker->start();
}

void BatchModePage::onStopBatch() {
    if (m_worker && m_worker->isRunning()) {
        m_worker->stop(); // 通知后台线程停止
        
        m_btnStop->setEnabled(false); // 防止用户疯狂连点
        m_btnStop->setText("⏳ 正在安全中断...");
        
        // 打印提示给用户
        onWorkerLog("🚨 收到中止指令！正在等待当前文件夹处理完毕后安全退出...", "WARN");
    }
}


void BatchModePage::onWorkerProgress(int current, int total) {
    m_progressBar->setValue(current);
}

void BatchModePage::onWorkerLog(const QString& msg, const QString& type) {
    QString timeStr = QDateTime::currentDateTime().toString("[HH:mm:ss]");
    QString color = "#d4d4d4"; 
    if (type == "WARN") color = "#e5c07b";
    else if (type == "ERROR") color = "#e06c75";
    else if (type == "SUCCESS") color = "#98c379";
    else if (type == "ALGO") color = "#61afef";

    QString html = QString("<span style='color:#5c6370;'>%1</span> <span style='color:%2; font-weight:bold;'>[%3]</span> <span style='color:#d4d4d4;'>%4</span>")
                    .arg(timeStr).arg(color).arg(type).arg(msg);
    m_console->append(html);
    QScrollBar *sb = m_console->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void BatchModePage::onWorkerFinished(int successCount, int totalCount) {
    // 恢复开始按钮
    m_btnStart->setEnabled(true);
    m_btnStart->setText("🚀 开始批量处理");
    
    // [新增] 重置停止按钮
    m_btnStop->setEnabled(false);
    m_btnStop->setText("🛑 停止处理");

    QMessageBox::information(this, "批处理完成", QString("批处理结束！\n成功: %1\n总计(含跳过/中断): %2").arg(successCount).arg(totalCount));
    
    m_worker->deleteLater();
    m_worker = nullptr;
}



// ==========================================================
// BatchWorker (后台多线程核心逻辑)
// ==========================================================

// 用于 QtConcurrent 的局部并行任务结构
// [核心修复 1] 彻底规避 Eigen 内存对齐崩溃
struct WorkerRegTask {
    QString camId;
    PointCloudT::Ptr srcCloud;
    PointCloudT::Ptr targetCloud;
    
    // 绝对不要在这里放 Eigen::Matrix4d，改用标准原生数组！
    std::array<double, 16> guessArray; 
    
    BatchParams params;
    PointCloudT::Ptr resultCloud;
    bool valid = false;
    QString errorMsg; // 捕获异常信息
};

// 并行配准核心函数
WorkerRegTask processSingleCloud(WorkerRegTask task) {
    task.valid = false;
    if (!task.srcCloud || task.srcCloud->empty()) {
        task.errorMsg = "输入点云为空";
        return task;
    }

    // [核心修复 2] 增加 try-catch 拦截 PCL 底层抛出的异常，防止程序闪退
    try {
        // 1. 预处理
        auto p1 = PointCloudAlgo::downsample(task.srcCloud, task.params.leafSize);
        if(!p1) throw std::runtime_error("降采样失败");
        
        auto p2 = PointCloudAlgo::statisticalOutlierRemoval(p1, task.params.meanK, task.params.stdDev);
        if(!p2) throw std::runtime_error("SOR滤波失败");
        
        auto p3 = PointCloudAlgo::distanceClip(p2, task.params.clipRadius);
        if (!p3 || p3->empty()) throw std::runtime_error("裁剪后点云为空");

        // 2. 配准 (Top 视角不需要配准)
        if (task.camId == "Top") {
            task.resultCloud = p3;
            task.valid = true;
            return task;
        }

        // 将安全数组还原为 Eigen 矩阵 (Eigen 默认是列主序，直接映射内存即可)
        Eigen::Matrix4d guessMat(task.guessArray.data());
        Eigen::Matrix4d finalMat = guessMat;

        if (task.params.regMethod == 1 || task.params.regMethod == 2) {
            int algoType = (task.params.regMethod == 2) ? PointCloudAlgo::P2Plane : PointCloudAlgo::P2Point;
            auto res = PointCloudAlgo::alignICP(p3, task.targetCloud, guessMat, task.params.icpIter, task.params.icpDist, algoType);
            task.resultCloud = res.first;
        } else if (task.params.regMethod == 3) {
            Eigen::Matrix4f guess_f = guessMat.cast<float>();
            Eigen::Matrix4f final_f = PointCloudAlgo::refineRegistrationNDT(p3, task.targetCloud, guess_f, task.params.ndtRes, task.params.ndtStep, task.params.ndtIter);
            task.resultCloud = PointCloudAlgo::transformCloud(p3, final_f.cast<double>());
        } else if (task.params.regMethod == 4) {
            auto res = PointCloudAlgo::alignGICP(p3, task.targetCloud, guessMat, task.params.gicpIter, task.params.gicpDist, task.params.gicpEps);
            task.resultCloud = res.first;
        } else {
            task.resultCloud = PointCloudAlgo::transformCloud(p3, guessMat);
        }
        
        if(task.resultCloud) task.valid = true;
        
    } catch (const std::exception& e) {
        task.errorMsg = QString("算法异常: %1").arg(e.what());
    } catch (...) {
        task.errorMsg = "发生未知算法崩溃";
    }

    return task;
}

BatchWorker::BatchWorker(const QStringList& folders, const BatchParams& params, QObject* parent) 
    : QThread(parent), m_folders(folders), m_params(params) {}

BatchWorker::~BatchWorker() {}

void BatchWorker::stop() {
    m_stopFlag = true;
}

void BatchWorker::run() {
    int successCount = 0;
    int total = m_folders.size();
    
    auto defaultMats = getDefaultTransforms();

    for (int i = 0; i < total; ++i) {
        if (m_stopFlag) {
            emit logMessage("批处理已被用户手动中止！", "WARN");
            break;
        }

        QString folderPath = m_folders[i];
        QFileInfo folderInfo(folderPath);
        QString folderName = folderInfo.fileName(); // e.g., 20251102_115330_818
        
        emit logMessage(QString("==================== 开始处理 [%1] ====================").arg(folderName), "INFO");

        // 1. 创建输出目录
        QString outPath = m_params.outputDir + "/" + folderName;
        QDir().mkpath(outPath);


        // 2. 加载当前文件夹下的点云
        QDir currentDir(folderPath);
        // [修改] 过滤器同时支持提取 .pcd 和 .raw 文件
        QStringList nameFilters;
        nameFilters << "*_d_pc.pcd" << "*_depth_raw.raw";
        QStringList targetFiles = currentDir.entryList(nameFilters, QDir::Files);
        
        QMap<QString, PointCloudT::Ptr> clouds;
        QMap<QString, QString> keyMap = { {"005J","Top"}, {"00YA","LB"}, {"00X6","LT"}, {"00SE","RB"}, {"003W","RT"} };
        
        for (const QString& file : targetFiles) {
            for (auto it = keyMap.begin(); it != keyMap.end(); ++it) {
                // 匹配设备编号
                if (file.contains(it.key(), Qt::CaseInsensitive)) {
                    QString absPath = currentDir.absoluteFilePath(file);
                    QString camKey = it.value();
                    PointCloudT::Ptr cloud(new PointCloudT);
                    
                    // =======================================================
                    // [核心修改] 根据文件后缀名分支，动态解析
                    // =======================================================
                    if (file.endsWith(".raw", Qt::CaseInsensitive)) {
                        // [新增] 极致优雅的查表调用，一行代码拿到包含畸变系数在内的全套精准参数！
                        CameraIntrinsics intr = PointCloudAlgo::getCameraIntrinsics(camKey, SensorType::DEPTH, 512, 512);
                        // 调用刚才写好的底层静态库，使用对应的相机内参解析深度图
                        cloud = PointCloudAlgo::convertRawDepthToPointCloud(absPath, intr);
                        if (cloud && !cloud->empty()) {
                            clouds[camKey] = cloud;
                        } else {
                            emit logMessage(QString("RAW 解析失败或为空: %1").arg(file), "WARN");
                        }
                    } else {
                        // 传统的 PCD 加载方式
                        if (pcl::io::loadPCDFile<PointT>(absPath.toStdString(), *cloud) == 0) {
                            clouds[camKey] = cloud;
                        }
                    }
                    
                    break; // 匹配到当前相机编号后，跳出内层循环，继续处理下一个文件
                }
            }
        }

        if (!clouds.contains("Top")) {
            emit logMessage(QString("[%1] 缺少 Top 视角点云，跳过！").arg(folderName), "ERROR");
            emit progressUpdated(i + 1, total);
            continue;
        }

        // 3. 构建多线程配准任务
        // 注意：先处理 Top，作为后续配准的目标
        // 为防止后续由于异常崩溃，这里包一层 try-catch
        try {
            PointCloudT::Ptr topCloud = clouds["Top"];
            auto p1 = PointCloudAlgo::downsample(topCloud, m_params.leafSize);
            auto p2 = PointCloudAlgo::statisticalOutlierRemoval(p1, m_params.meanK, m_params.stdDev);
            PointCloudT::Ptr topProcessed = PointCloudAlgo::distanceClip(p2, m_params.clipRadius);

            QList<WorkerRegTask> tasks;
            QStringList sideCams = {"LB", "LT", "RB", "RT"};
            for (const QString& cam : sideCams) {
                if (clouds.contains(cam)) {
                    WorkerRegTask t;
                    t.camId = cam;
                    t.srcCloud = clouds[cam];
                    t.targetCloud = topProcessed;
                    t.params = m_params;
                    
                    // [核心修复] 将矩阵解构成安全数组
                    Eigen::Matrix4d mat = defaultMats[cam];
                    const double* matData = mat.data();
                    for(int k = 0; k < 16; ++k) t.guessArray[k] = matData[k];
                    
                    tasks.append(t);
                }
            }

            emit logMessage("正在并行执行预处理与配准...", "ALGO");
            QList<WorkerRegTask> results = QtConcurrent::mapped(tasks, processSingleCloud).results();

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedRGB(new pcl::PointCloud<pcl::PointXYZRGB>);
            PointCloudT::Ptr mergedGeom(new PointCloudT);

            auto appendCloud = [&](PointCloudT::Ptr cloud, const QString& cam) {
                if (!cloud) return;
                int r, g, b; getCameraColor(cam, r, g, b);
                for (const auto& pt : cloud->points) {
                    mergedGeom->points.push_back(pt);
                    pcl::PointXYZRGB ptRGB;
                    ptRGB.x = pt.x; ptRGB.y = pt.y; ptRGB.z = pt.z;
                    ptRGB.r = r; ptRGB.g = g; ptRGB.b = b;
                    mergedRGB->points.push_back(ptRGB);
                }
            };

            appendCloud(topProcessed, "Top");
            for (const auto& res : results) {
                if (res.valid) {
                    appendCloud(res.resultCloud, res.camId);
                } else {
                    emit logMessage(QString("相机 [%1] 处理失败: %2").arg(res.camId).arg(res.errorMsg), "WARN");
                }
            }
            mergedGeom->width = mergedGeom->size(); mergedGeom->height = 1;

            emit logMessage("提取猪只主体...", "ALGO");
    
            // [核心修复] 组装主体提取的全局参数
            ExtractionParams extParams;
            extParams.boxMinX = m_params.boxMinX; extParams.boxMinY = m_params.boxMinY; extParams.boxMinZ = m_params.boxMinZ;
            extParams.boxMaxX = m_params.boxMaxX; extParams.boxMaxY = m_params.boxMaxY; extParams.boxMaxZ = m_params.boxMaxZ;
            extParams.boxRotZ = m_params.boxRotZ;
            extParams.minClusterSize = m_params.minClusterSize;
            extParams.methodIndex = m_params.extMethodIndex;
            extParams.euclideanTolerance = m_params.extEuclideanTol;
            extParams.rgNeighbors = m_params.extRgNeighbors;
            extParams.rgSmoothness = m_params.extRgSmoothness;
            extParams.useRansac = m_params.useRansac;
            extParams.ransacDistThresh = m_params.ransacDistThresh;
            extParams.useMlsUpsampling = m_params.useMlsUpsampling;
            extParams.mlsSearchRadius = m_params.mlsSearchRadius;
            extParams.mlsUpsamplingRadius = m_params.mlsUpsamplingRadius;
            extParams.mlsUpsamplingStep = m_params.mlsUpsamplingStep;

            // [注意：此处日志回调可以传 nullptr，防止批处理期间大量日志刷屏]
            PointCloudT::Ptr bodyCloud = PointCloudAlgo::extractLargestCluster(mergedGeom, extParams, nullptr);

            
            if (!bodyCloud || bodyCloud->empty()) {
                emit logMessage("主体提取失败，跳过该文件夹。", "ERROR");
                emit progressUpdated(i + 1, total);
                continue;
            }

            // ==========================================================
            // [新增功能] 如果用户勾选了“仅提取”，则在此处直接保存并跳到下一个文件夹
            // ==========================================================
            if (m_params.onlyExtractBody) {
                QString pcdMerged = outPath + "/Merged_Cloud.pcd";
                QString pcdBody   = outPath + "/Pig_Body.pcd";
                pcl::io::savePCDFileBinary(pcdMerged.toStdString(), *mergedRGB);
                pcl::io::savePCDFileBinary(pcdBody.toStdString(), *bodyCloud);
                
                emit logMessage(QString("[%1] 【仅提取模式】融合与主体点云已保存！").arg(folderName), "SUCCESS");
                successCount++;
                emit progressUpdated(i + 1, total);
                continue; // 直接 Continue，跳过后面的 AI 预测和体尺测量
            }
            // ==========================================================


            emit logMessage("正在执行 AI 关键点预测...", "ALGO");
    
            // [核心修复] 利用刚才的包围盒组装 backCloud 的提取参数
            ExtractionParams backParams = extParams;
            backParams.methodIndex = 0; // 强制用欧式聚类
            backParams.euclideanTolerance = 50.0;
            backParams.minClusterSize = 1000;
            backParams.useRansac = false;
            PointCloudT::Ptr backCloud = PointCloudAlgo::extractLargestCluster(topProcessed, backParams, nullptr);

            if(!backCloud) backCloud = topProcessed; 

            pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
            pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
            ne.setSearchMethod(tree); ne.setInputCloud(backCloud); ne.setRadiusSearch(50.0);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
            ne.compute(*cloud_normals);

            QByteArray postData;
            int numPoints = backCloud->size();
            postData.resize(numPoints * 7 * sizeof(float));
            float* ptr = reinterpret_cast<float*>(postData.data());
            for (int j = 0; j < numPoints; ++j) {
                *ptr++ = backCloud->points[j].x; *ptr++ = backCloud->points[j].y; *ptr++ = backCloud->points[j].z;
                *ptr++ = cloud_normals->points[j].normal_x; *ptr++ = cloud_normals->points[j].normal_y; *ptr++ = cloud_normals->points[j].normal_z;
                *ptr++ = cloud_normals->points[j].curvature;
            }

            // HTTP 请求
            QNetworkAccessManager manager;
            QNetworkRequest request(QUrl(m_params.aiEndpoint));
            QHttpMultiPart *multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);
            QHttpPart filePart;
            filePart.setHeader(QNetworkRequest::ContentDispositionHeader, QVariant("form-data; name=\"file\"; filename=\"pointcloud.bin\""));
            filePart.setBody(postData);
            multiPart->append(filePart);

            QNetworkReply *reply = manager.post(request, multiPart);
            multiPart->setParent(reply);

            QEventLoop loop;
            connect(reply, &QNetworkReply::finished, &loop, &QEventLoop::quit);
            loop.exec(); 

            std::vector<Eigen::Vector3f> keypoints;
            if (reply->error() == QNetworkReply::NoError) {
                QByteArray resData = reply->readAll();
                int numKps = resData.size() / (3 * sizeof(float));
                const float* outPtr = reinterpret_cast<const float*>(resData.constData());
                for (int k = 0; k < numKps; ++k) {
                    keypoints.push_back(Eigen::Vector3f(outPtr[k*3], outPtr[k*3+1], outPtr[k*3+2]));
                }
            } else {
                emit logMessage("AI 服务连接失败: " + reply->errorString(), "ERROR");
            }
            
            // [核心修复 3] 在局部事件循环后，必须直接 delete，绝不能用 deleteLater
            delete reply;

            if (keypoints.size() != 6) {
                emit logMessage(QString("AI 检测失败，仅检测到 %1 个关键点。").arg(keypoints.size()), "ERROR");
                emit progressUpdated(i + 1, total);
                continue;
            }

            emit logMessage("计算体尺参数...", "ALGO");
            BodySizeResults measRes = PointCloudAlgo::calculateAllMeasurements(
                bodyCloud, mergedGeom, keypoints, 
                m_params.girthThick, m_params.skelStep, m_params.skelRadius, m_params.heightAngle
            );

            if (measRes.aligned_cloud && !measRes.aligned_cloud->empty()) {
                QString pcdMerged = outPath + "/Merged_Cloud.pcd";
                QString pcdBody   = outPath + "/Pig_Body.pcd";
                QString pcdVisual = outPath + "/Measurement_Visual.pcd"; // [新增]
                QString txtReport = outPath + "/Measurement_Report.txt";

                pcl::io::savePCDFileBinary(pcdMerged.toStdString(), *mergedRGB);
                pcl::io::savePCDFileBinary(pcdBody.toStdString(), *bodyCloud);

                // ==========================================================
                // [新增] 自动构建体尺测量的可视化彩色点云
                // ==========================================================
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
                for (const auto& p : measRes.aligned_cloud->points) {
                    pcl::PointXYZRGB pt; pt.x = p.x; pt.y = p.y; pt.z = p.z;
                    pt.r = 200; pt.g = 200; pt.b = 200; viz_cloud->push_back(pt);
                }

                // 辅助 Lambda：在两点之间进行插值，并生成物理“加粗”的点云管道
                auto drawLine = [&](const PointT& p1, const PointT& p2, uint8_t r, uint8_t g, uint8_t b, float thickness = 3.0f) {
                    Eigen::Vector3f v1 = p1.getVector3fMap();
                    Eigen::Vector3f v2 = p2.getVector3fMap();
                    float dist = (v2 - v1).norm();
                    float forward_step = 1.0f; // 沿线段前进的步长 1mm
                    int steps = std::max(1, static_cast<int>(dist / forward_step));

                    for (int i = 0; i <= steps; ++i) {
                        Eigen::Vector3f p = v1 + (v2 - v1) * (static_cast<float>(i) / steps);
                        
                        // 为了加粗，在中心点周围的立体空间内撒点，形成有厚度的管道
                        // thickness 为管道的物理半径 (mm)
                        for (float dx = -thickness; dx <= thickness; dx += 1.5f) {
                            for (float dy = -thickness; dy <= thickness; dy += 1.5f) {
                                for (float dz = -thickness; dz <= thickness; dz += 1.5f) {
                                    // 利用勾股定理，只保留圆柱/球截面内的点，使线条圆润
                                    if (dx*dx + dy*dy + dz*dz <= thickness*thickness) {
                                        pcl::PointXYZRGB pt;
                                        pt.x = p.x() + dx; 
                                        pt.y = p.y() + dy; 
                                        pt.z = p.z() + dz;
                                        pt.r = r; pt.g = g; pt.b = b;
                                        viz_cloud->push_back(pt);
                                    }
                                }
                            }
                        }
                    }
                };

                auto drawSphere = [&](const PointT& center, float radius, uint8_t r, uint8_t g, uint8_t b) {
                    for (float dx = -radius; dx <= radius; dx += 1.5f) {
                        for (float dy = -radius; dy <= radius; dy += 1.5f) {
                            for (float dz = -radius; dz <= radius; dz += 1.5f) {
                                if (dx*dx + dy*dy + dz*dz <= radius*radius) {
                                    pcl::PointXYZRGB pt; pt.x = center.x+dx; pt.y = center.y+dy; pt.z = center.z+dz;
                                    pt.r = r; pt.g = g; pt.b = b; viz_cloud->push_back(pt);
                                }
                            }
                        }
                    }
                };
                auto drawContour = [&](PointCloudT::Ptr contour, uint8_t r, uint8_t g, uint8_t b) {
                    if (!contour || contour->size() < 3) return;
                    for (size_t i = 0; i < contour->size() - 1; ++i) drawLine(contour->points[i], contour->points[i+1], r, g, b);
                    drawLine(contour->points.back(), contour->points.front(), r, g, b);
                };

                for (const auto& kp : measRes.aligned_keypoints) drawSphere(kp, 15.0f, 255, 0, 0);
                if (measRes.skeleton_cloud && measRes.skeleton_cloud->size() > 1) {
                    for (size_t i = 0; i < measRes.skeleton_cloud->size() - 1; ++i)
                        drawLine(measRes.skeleton_cloud->points[i], measRes.skeleton_cloud->points[i+1], 0, 255, 0);
                }
                drawLine(measRes.height_top, measRes.height_bottom, 0, 0, 255);
                drawLine(measRes.width_p1, measRes.width_p2, 255, 255, 0);
                drawContour(measRes.chest_contour, 255, 128, 0);
                drawContour(measRes.waist_contour, 0, 255, 255);
                drawContour(measRes.hip_contour, 255, 0, 255);
                if (measRes.ground_polygon && measRes.ground_polygon->size() == 4) drawContour(measRes.ground_polygon, 0, 150, 0);

                viz_cloud->width = viz_cloud->size(); viz_cloud->height = 1; viz_cloud->is_dense = true;
                pcl::io::savePCDFileBinary(pcdVisual.toStdString(), *viz_cloud);
                // ==========================================================

                QFile file(txtReport);
                if (file.open(QIODevice::WriteOnly | QIODevice::Text)) {
                    QTextStream out(&file);
                    out << "\xEF\xBB\xBF"; 
                    out << "============================================================\n";
                    out << "                     猪 只 体 尺 测 量 报 告                   \n";
                    out << "============================================================\n";
                    out << QString(" ID   : %1\n").arg(folderName);
                    out << "------------------------------------------------------------\n";
                    auto writeLine = [&out](const QString& name, double value) {
                        out << " " << QString("%1").arg(name, -25) << " :\t" << QString("%1").arg(value, 8, 'f', 2) << " mm\n";
                    };
                    writeLine("体长 (Body Length)", measRes.body_length);
                    writeLine("体高 (Body Height)", measRes.body_height);
                    writeLine("体宽 (Body Width)",  measRes.body_width);
                    writeLine("胸围 (Chest Girth)", measRes.chest_girth);
                    writeLine("腰围 (Waist Girth)", measRes.waist_girth);
                    writeLine("臀围 (Hip Girth)",   measRes.hip_girth);
                    file.close();
                }

                emit logMessage(QString("[%1] 处理成功！结果(含可视化)已保存。").arg(folderName), "SUCCESS");
                successCount++;
            } else {
                emit logMessage(QString("[%1] 体尺测量失败，可能是主体提取不完整。").arg(folderName), "ERROR");
            }

        } catch (const std::exception& e) {
            emit logMessage(QString("[%1] 发生致命异常跳过该目录: %2").arg(folderName).arg(e.what()), "ERROR");
        } catch (...) {
            emit logMessage(QString("[%1] 发生未知错误跳过该目录").arg(folderName), "ERROR");
        }

        emit progressUpdated(i + 1, total);
    }

    emit batchFinished(successCount, total);
}

// 辅助方法：返回默认校准矩阵
QMap<QString, Eigen::Matrix4d> BatchWorker::getDefaultTransforms() {
    QMap<QString, Eigen::Matrix4d> mats;
    Eigen::Matrix4d lb; lb << 
        -0.926086, -0.052355, -0.373662, 719.549438,
        -0.368365, -0.088922, 0.925419, -1532.871094,
        -0.081677, 0.994662, 0.063064, 1826.347290,
        0.000000, 0.000000, 0.000000, 1.000000;
    Eigen::Matrix4d lt; lt << 
        -0.893402, 0.129827, -0.430091, 715.208496,
        -0.431722, -0.512967, 0.741944, -1244.757813,
        -0.124298, 0.848534, 0.514334, 853.710327,
        0.000000, 0.000000, 0.000000, 1.000000;
    Eigen::Matrix4d rb; rb << 
        0.843684, 0.019195, 0.536497, -845.013184,
        0.527601, -0.214250, -0.822030, 1084.744995,
        0.099166, 0.976590, -0.190886, 2075.820557,
        0.000000, 0.000000, 0.000000, 1.000000;
    Eigen::Matrix4d rt; rt << 
        0.881307, -0.310379, 0.356319, -767.480042,
        0.451075, 0.327860, -0.830084, 1185.616089,
        0.140818, 0.892285, 0.428950, 1194.222656,
        0.000000, 0.000000, 0.000000, 1.000000;
    mats["LB"] = lb; mats["LT"] = lt; mats["RB"] = rb; mats["RT"] = rt;
    return mats;
}

void BatchWorker::getCameraColor(const QString& camName, int& r, int& g, int& b) {
    if      (camName == "Top") { r = 255; g = 0;   b = 0;   }
    else if (camName == "LB")  { r = 0;   g = 255; b = 0;   }
    else if (camName == "LT")  { r = 0;   g = 0;   b = 255; }
    else if (camName == "RB")  { r = 255; g = 215; b = 0;   }
    else if (camName == "RT")  { r = 0;   g = 255; b = 255; }
    else                       { r = 255; g = 255; b = 255; }
}