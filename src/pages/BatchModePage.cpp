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
    ioLay->addLayout(btnLay, 2, 0, 1, 3);
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
    m_comboRegAlgo = new QComboBox(); m_comboRegAlgo->addItems({"手动矩阵", "ICP (P2Point)", "ICP (P2Plane)", "NDT 微调"});
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

    colA->addWidget(grpReg);
    paramLayout->addLayout(colA);

    // 栏目 B: 提取与测量
    QVBoxLayout *colB = new QVBoxLayout();
    QGroupBox *grpExt = new QGroupBox("3. 提取参数");
    QGridLayout *extLay = new QGridLayout(grpExt);
    m_spinRansac = new QDoubleSpinBox(); m_spinRansac->setValue(20.0); extLay->addWidget(new QLabel("地面滤除厚度:"), 0,0); extLay->addWidget(m_spinRansac, 0,1);
    m_spinTol = new QDoubleSpinBox(); m_spinTol->setValue(40.0); extLay->addWidget(new QLabel("聚类容差:"), 1,0); extLay->addWidget(m_spinTol, 1,1);
    m_spinMinSize = new QSpinBox(); m_spinMinSize->setMaximum(100000); m_spinMinSize->setValue(5000); extLay->addWidget(new QLabel("最小簇点数:"), 2,0); extLay->addWidget(m_spinMinSize, 2,1);
    colB->addWidget(grpExt);

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
    p.inputDir = m_leInput->text();
    p.outputDir = m_leOutput->text();
    p.leafSize = m_spinLeaf->value();
    p.stdDev = m_spinStd->value();
    p.meanK = m_spinMeanK->value();
    p.clipRadius = m_spinClip->value();
    p.regMethod = m_comboRegAlgo->currentIndex();
    p.icpIter = m_spinIcpIter->value();
    p.icpDist = m_spinIcpDist->value();
    p.ndtRes = m_spinNdtRes->value();
    p.ndtStep = m_spinNdtStep->value();
    p.ndtIter = m_spinNdtIter->value();
    p.ransacThresh = m_spinRansac->value();
    p.clusterTol = m_spinTol->value();
    p.minClusterSize = m_spinMinSize->value();
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
        QStringList pcdFiles = currentDir.entryList({"*_d_pc.pcd"}, QDir::Files);
        
        QMap<QString, PointCloudT::Ptr> clouds;
        QMap<QString, QString> keyMap = { {"005J","Top"}, {"00SE","LB"}, {"003W","LT"}, {"00YA","RB"}, {"00X6","RT"} };
        
        for (const QString& file : pcdFiles) {
            for (auto it = keyMap.begin(); it != keyMap.end(); ++it) {
                if (file.contains(it.key())) {
                    PointCloudT::Ptr cloud(new PointCloudT);
                    if (pcl::io::loadPCDFile<PointT>(currentDir.absoluteFilePath(file).toStdString(), *cloud) == 0) {
                        clouds[it.value()] = cloud;
                    }
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
            PointCloudT::Ptr bodyCloud = PointCloudAlgo::extractLargestCluster(
                mergedGeom, m_params.clusterTol, m_params.minClusterSize, m_params.ransacThresh);
            
            if (!bodyCloud || bodyCloud->empty()) {
                emit logMessage("主体提取失败，跳过该文件夹。", "ERROR");
                emit progressUpdated(i + 1, total);
                continue;
            }

            emit logMessage("正在执行 AI 关键点预测...", "ALGO");
            PointCloudT::Ptr backCloud = PointCloudAlgo::extractLargestCluster(
                topProcessed, m_params.clusterTol, m_params.minClusterSize, 15.0);
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
    Eigen::Matrix4d lb; lb << 0.998144, 0.040452, 0.045531, 43.625172, 0.049100, -0.092113, -0.994537, 1393.481567, -0.036037, 0.994927, -0.093928, 2062.417725, 0, 0, 0, 1;
    Eigen::Matrix4d lt; lt << 0.991525, 0.015493, 0.128988, -49.530918, 0.125268, 0.149176, -0.980844, 1446.867676, -0.034438, 0.988689, 0.145971, 1454.759033, 0, 0, 0, 1;
    Eigen::Matrix4d rb; rb << -0.997376, 0.071718, 0.009833, 9.360316, 0.006674, -0.044152, 0.999003, -1420.248047, 0.072081, 0.996447, 0.043558, 1944.664185, 0, 0, 0, 1;
    Eigen::Matrix4d rt; rt << -0.993564, 0.102217, 0.048803, 11.873594, 0.016009, -0.299805, 0.953866, -1409.841309, 0.112133, 0.948509, 0.296239, 1245.285522, 0, 0, 0, 1;
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