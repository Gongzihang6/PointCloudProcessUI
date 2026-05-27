#include "pages/BatchModePage.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDateTime>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFrame>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QProgressBar>
#include <QPushButton>
#include <QScrollArea>
#include <QScrollBar>
#include <QSpinBox>
#include <QTextEdit>
#include <QVBoxLayout>

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

    QGroupBox *grpIO = new QGroupBox("Batch IO");
    QGridLayout *ioLay = new QGridLayout(grpIO);

    m_leInput = new QLineEdit();
    m_leInput->setPlaceholderText("Select input directory...");
    QPushButton *btnInput = new QPushButton("Browse...");
    connect(btnInput, &QPushButton::clicked, this, &BatchModePage::onBrowseInput);

    m_leOutput = new QLineEdit();
    m_leOutput->setPlaceholderText("Select output directory...");
    QPushButton *btnOutput = new QPushButton("Browse...");
    connect(btnOutput, &QPushButton::clicked, this, &BatchModePage::onBrowseOutput);

    ioLay->addWidget(new QLabel("Input:"), 0, 0);
    ioLay->addWidget(m_leInput, 0, 1);
    ioLay->addWidget(btnInput, 0, 2);
    ioLay->addWidget(new QLabel("Output:"), 1, 0);
    ioLay->addWidget(m_leOutput, 1, 1);
    ioLay->addWidget(btnOutput, 1, 2);

    m_chkOnlyExtract = new QCheckBox("Only extract merged/body clouds");
    m_chkOnlyExtract->setStyleSheet("font-weight: bold; color: #E6A23C; margin-top: 5px;");
    ioLay->addWidget(m_chkOnlyExtract, 2, 0, 1, 3);

    m_btnStart = new QPushButton("Start Batch");
    m_btnStart->setMinimumHeight(40);
    m_btnStart->setStyleSheet("font-weight: bold; font-size: 14px; background-color: #409EFF; color: white; border-radius: 4px;");
    connect(m_btnStart, &QPushButton::clicked, this, &BatchModePage::onStartBatch);

    m_btnStop = new QPushButton("Stop");
    m_btnStop->setMinimumHeight(40);
    m_btnStop->setStyleSheet("font-weight: bold; font-size: 14px; background-color: #F56C6C; color: white; border-radius: 4px;");
    m_btnStop->setEnabled(false);
    connect(m_btnStop, &QPushButton::clicked, this, &BatchModePage::onStopBatch);

    QHBoxLayout *btnLay = new QHBoxLayout();
    btnLay->addWidget(m_btnStart);
    btnLay->addWidget(m_btnStop);
    ioLay->addLayout(btnLay, 3, 0, 1, 3);
    mainLayout->addWidget(grpIO, 0);

    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    QWidget *scrollContent = new QWidget();
    QHBoxLayout *paramLayout = new QHBoxLayout(scrollContent);

    QVBoxLayout *colA = new QVBoxLayout();
    QGroupBox *grpPre = new QGroupBox("1. Preprocess");
    QGridLayout *preLay = new QGridLayout(grpPre);
    m_spinLeaf = new QDoubleSpinBox();
    m_spinLeaf->setValue(10.0);
    preLay->addWidget(new QLabel("Leaf size (mm):"), 0, 0);
    preLay->addWidget(m_spinLeaf, 0, 1);
    m_spinStd = new QDoubleSpinBox();
    m_spinStd->setValue(2.0);
    preLay->addWidget(new QLabel("SOR stddev:"), 1, 0);
    preLay->addWidget(m_spinStd, 1, 1);
    m_spinMeanK = new QSpinBox();
    m_spinMeanK->setValue(50);
    m_spinMeanK->setMaximum(500);
    preLay->addWidget(new QLabel("SOR mean K:"), 2, 0);
    preLay->addWidget(m_spinMeanK, 2, 1);
    m_spinClip = new QDoubleSpinBox();
    m_spinClip->setRange(100, 10000);
    m_spinClip->setValue(2500);
    preLay->addWidget(new QLabel("Clip radius (mm):"), 3, 0);
    preLay->addWidget(m_spinClip, 3, 1);
    colA->addWidget(grpPre);

    QGroupBox *grpReg = new QGroupBox("2. Registration");
    QGridLayout *regLay = new QGridLayout(grpReg);
    m_comboRegAlgo = new QComboBox();
    m_comboRegAlgo->addItems({"Manual Matrix", "ICP (P2Point)", "ICP (P2Plane)", "NDT", "G-ICP"});
    regLay->addWidget(new QLabel("Method:"), 0, 0);
    regLay->addWidget(m_comboRegAlgo, 0, 1);

    m_spinIcpIter = new QSpinBox();
    m_spinIcpIter->setValue(60);
    m_spinIcpIter->setMaximum(500);
    regLay->addWidget(new QLabel("ICP iterations:"), 1, 0);
    regLay->addWidget(m_spinIcpIter, 1, 1);

    m_spinIcpDist = new QDoubleSpinBox();
    m_spinIcpDist->setValue(100.0);
    m_spinIcpDist->setMaximum(1000);
    regLay->addWidget(new QLabel("ICP max dist:"), 2, 0);
    regLay->addWidget(m_spinIcpDist, 2, 1);

    m_spinNdtRes = new QDoubleSpinBox();
    m_spinNdtRes->setValue(100.0);
    m_spinNdtRes->setMaximum(500);
    regLay->addWidget(new QLabel("NDT resolution:"), 3, 0);
    regLay->addWidget(m_spinNdtRes, 3, 1);

    m_spinNdtStep = new QDoubleSpinBox();
    m_spinNdtStep->setValue(0.1);
    m_spinNdtStep->setSingleStep(0.1);
    regLay->addWidget(new QLabel("NDT step:"), 4, 0);
    regLay->addWidget(m_spinNdtStep, 4, 1);

    m_spinNdtIter = new QSpinBox();
    m_spinNdtIter->setValue(35);
    m_spinNdtIter->setMaximum(500);
    regLay->addWidget(new QLabel("NDT iterations:"), 5, 0);
    regLay->addWidget(m_spinNdtIter, 5, 1);

    m_spinGicpIter = new QSpinBox();
    m_spinGicpIter->setValue(50);
    m_spinGicpIter->setMaximum(500);
    regLay->addWidget(new QLabel("G-ICP iterations:"), 6, 0);
    regLay->addWidget(m_spinGicpIter, 6, 1);

    m_spinGicpDist = new QDoubleSpinBox();
    m_spinGicpDist->setValue(50.0);
    m_spinGicpDist->setMaximum(1000.0);
    regLay->addWidget(new QLabel("G-ICP max dist:"), 7, 0);
    regLay->addWidget(m_spinGicpDist, 7, 1);

    colA->addWidget(grpReg);
    paramLayout->addLayout(colA);

    QVBoxLayout *colB = new QVBoxLayout();
    QGroupBox *grpExt = new QGroupBox("3. Extraction");
    QGridLayout *extLay = new QGridLayout(grpExt);
    int r = 0;

    QHBoxLayout *minLay = new QHBoxLayout();
    m_spinBoxMinX = new QDoubleSpinBox();
    m_spinBoxMinX->setRange(-3000, 3000);
    m_spinBoxMinX->setValue(-1200);
    m_spinBoxMinX->setDecimals(0);
    m_spinBoxMinY = new QDoubleSpinBox();
    m_spinBoxMinY->setRange(-3000, 3000);
    m_spinBoxMinY->setValue(-460);
    m_spinBoxMinY->setDecimals(0);
    m_spinBoxMinZ = new QDoubleSpinBox();
    m_spinBoxMinZ->setRange(-3000, 3000);
    m_spinBoxMinZ->setValue(-500);
    m_spinBoxMinZ->setDecimals(0);
    minLay->addWidget(m_spinBoxMinX);
    minLay->addWidget(m_spinBoxMinY);
    minLay->addWidget(m_spinBoxMinZ);
    extLay->addWidget(new QLabel("Box min (X/Y/Z):"), r, 0);
    extLay->addLayout(minLay, r, 1);
    ++r;

    QHBoxLayout *maxLay = new QHBoxLayout();
    m_spinBoxMaxX = new QDoubleSpinBox();
    m_spinBoxMaxX->setRange(-3000, 3000);
    m_spinBoxMaxX->setValue(600);
    m_spinBoxMaxX->setDecimals(0);
    m_spinBoxMaxY = new QDoubleSpinBox();
    m_spinBoxMaxY->setRange(-3000, 3000);
    m_spinBoxMaxY->setValue(170);
    m_spinBoxMaxY->setDecimals(0);
    m_spinBoxMaxZ = new QDoubleSpinBox();
    m_spinBoxMaxZ->setRange(-3000, 3000);
    m_spinBoxMaxZ->setValue(2100);
    m_spinBoxMaxZ->setDecimals(0);
    maxLay->addWidget(m_spinBoxMaxX);
    maxLay->addWidget(m_spinBoxMaxY);
    maxLay->addWidget(m_spinBoxMaxZ);
    extLay->addWidget(new QLabel("Box max (X/Y/Z):"), r, 0);
    extLay->addLayout(maxLay, r, 1);
    ++r;

    m_spinBoxRotZ = new QDoubleSpinBox();
    m_spinBoxRotZ->setRange(-180, 180);
    m_spinBoxRotZ->setValue(33.0);
    extLay->addWidget(new QLabel("Rotate Z (deg):"), r, 0);
    extLay->addWidget(m_spinBoxRotZ, r, 1);
    ++r;

    m_spinExtMinPts = new QSpinBox();
    m_spinExtMinPts->setRange(100, 100000);
    m_spinExtMinPts->setValue(5000);
    extLay->addWidget(new QLabel("Min cluster size:"), r, 0);
    extLay->addWidget(m_spinExtMinPts, r, 1);
    ++r;

    m_comboExtMethod = new QComboBox();
    m_comboExtMethod->addItems({"Euclidean", "Region Growing"});
    extLay->addWidget(new QLabel("Method:"), r, 0);
    extLay->addWidget(m_comboExtMethod, r, 1);
    ++r;

    m_spinEuclideanTol = new QDoubleSpinBox();
    m_spinEuclideanTol->setRange(1, 200);
    m_spinEuclideanTol->setValue(40.0);
    extLay->addWidget(new QLabel("Euclidean tol (mm):"), r, 0);
    extLay->addWidget(m_spinEuclideanTol, r, 1);
    ++r;

    m_spinRgNeighbors = new QSpinBox();
    m_spinRgNeighbors->setRange(5, 100);
    m_spinRgNeighbors->setValue(30);
    extLay->addWidget(new QLabel("RG neighbors:"), r, 0);
    extLay->addWidget(m_spinRgNeighbors, r, 1);
    ++r;

    m_spinRgSmoothness = new QDoubleSpinBox();
    m_spinRgSmoothness->setRange(1, 45);
    m_spinRgSmoothness->setValue(7.0);
    extLay->addWidget(new QLabel("RG smoothness (deg):"), r, 0);
    extLay->addWidget(m_spinRgSmoothness, r, 1);
    ++r;

    m_chkUseRansac = new QCheckBox("Enable RANSAC plane removal");
    m_spinRansacDist = new QDoubleSpinBox();
    m_spinRansacDist->setRange(1.0, 100.0);
    m_spinRansacDist->setValue(20.0);
    m_spinRansacDist->setEnabled(false);
    connect(m_chkUseRansac, &QCheckBox::toggled, m_spinRansacDist, &QDoubleSpinBox::setEnabled);
    QHBoxLayout *ransacLay = new QHBoxLayout();
    ransacLay->addWidget(m_chkUseRansac);
    ransacLay->addWidget(new QLabel("Threshold:"));
    ransacLay->addWidget(m_spinRansacDist);
    extLay->addLayout(ransacLay, r, 0, 1, 2);
    ++r;

    colB->addWidget(grpExt);

    QGroupBox *grpMls = new QGroupBox("3.5 MLS");
    QGridLayout *mlsLay = new QGridLayout(grpMls);

    m_chkUseMls = new QCheckBox("Enable MLS upsampling");
    m_chkUseMls->setChecked(true);
    mlsLay->addWidget(m_chkUseMls, 0, 0, 1, 2);

    m_spinMlsSearchRadius = new QDoubleSpinBox();
    m_spinMlsSearchRadius->setRange(1, 500);
    m_spinMlsSearchRadius->setValue(80.0);
    mlsLay->addWidget(new QLabel("Search radius (mm):"), 1, 0);
    mlsLay->addWidget(m_spinMlsSearchRadius, 1, 1);

    m_spinMlsUpsampleRadius = new QDoubleSpinBox();
    m_spinMlsUpsampleRadius->setRange(1, 500);
    m_spinMlsUpsampleRadius->setValue(25.0);
    mlsLay->addWidget(new QLabel("Upsample radius (mm):"), 2, 0);
    mlsLay->addWidget(m_spinMlsUpsampleRadius, 2, 1);

    m_spinMlsUpsampleStep = new QDoubleSpinBox();
    m_spinMlsUpsampleStep->setRange(1, 100);
    m_spinMlsUpsampleStep->setValue(25.0);
    mlsLay->addWidget(new QLabel("Upsample step (mm):"), 3, 0);
    mlsLay->addWidget(m_spinMlsUpsampleStep, 3, 1);

    connect(m_chkUseMls, &QCheckBox::toggled, [this](bool checked) {
        m_spinMlsSearchRadius->setEnabled(checked);
        m_spinMlsUpsampleRadius->setEnabled(checked);
        m_spinMlsUpsampleStep->setEnabled(checked);
    });

    colB->addWidget(grpMls);

    QGroupBox *grpMeas = new QGroupBox("4. Measurement");
    QGridLayout *measLay = new QGridLayout(grpMeas);
    m_spinGirthThick = new QDoubleSpinBox();
    m_spinGirthThick->setValue(10.0);
    measLay->addWidget(new QLabel("Slice thickness:"), 0, 0);
    measLay->addWidget(m_spinGirthThick, 0, 1);
    m_spinSkelStep = new QDoubleSpinBox();
    m_spinSkelStep->setValue(20.0);
    measLay->addWidget(new QLabel("Skeleton step:"), 1, 0);
    measLay->addWidget(m_spinSkelStep, 1, 1);
    m_spinSkelRadius = new QDoubleSpinBox();
    m_spinSkelRadius->setValue(30.0);
    measLay->addWidget(new QLabel("Skeleton radius:"), 2, 0);
    measLay->addWidget(m_spinSkelRadius, 2, 1);
    m_spinHeightAngle = new QDoubleSpinBox();
    m_spinHeightAngle->setValue(15.0);
    measLay->addWidget(new QLabel("Ground angle tol:"), 3, 0);
    measLay->addWidget(m_spinHeightAngle, 3, 1);
    colB->addWidget(grpMeas);

    paramLayout->addLayout(colB);

    scroll->setWidget(scrollContent);
    mainLayout->addWidget(scroll, 1);

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
    QString dir = QFileDialog::getExistingDirectory(this, "Select input directory");
    if (!dir.isEmpty()) {
        m_leInput->setText(dir);
    }
}

void BatchModePage::onBrowseOutput() {
    QString dir = QFileDialog::getExistingDirectory(this, "Select output directory");
    if (!dir.isEmpty()) {
        m_leOutput->setText(dir);
    }
}

BatchParams BatchModePage::collectParams() {
    BatchParams p;
    p.inputDir = m_leInput->text();
    p.outputDir = m_leOutput->text();

    p.leafSize = static_cast<float>(m_spinLeaf->value());
    p.stdDev = m_spinStd->value();
    p.meanK = m_spinMeanK->value();
    p.clipRadius = static_cast<float>(m_spinClip->value());

    p.regMethod = m_comboRegAlgo->currentIndex();
    p.icpIter = m_spinIcpIter->value();
    p.icpDist = m_spinIcpDist->value();
    p.ndtRes = static_cast<float>(m_spinNdtRes->value());
    p.ndtStep = static_cast<float>(m_spinNdtStep->value());
    p.ndtIter = m_spinNdtIter->value();
    p.gicpIter = m_spinGicpIter->value();
    p.gicpDist = m_spinGicpDist->value();
    p.gicpEps = 1e-8;

    p.boxMinX = static_cast<float>(m_spinBoxMinX->value());
    p.boxMinY = static_cast<float>(m_spinBoxMinY->value());
    p.boxMinZ = static_cast<float>(m_spinBoxMinZ->value());
    p.boxMaxX = static_cast<float>(m_spinBoxMaxX->value());
    p.boxMaxY = static_cast<float>(m_spinBoxMaxY->value());
    p.boxMaxZ = static_cast<float>(m_spinBoxMaxZ->value());
    p.boxRotZ = static_cast<float>(m_spinBoxRotZ->value());
    p.minClusterSize = m_spinExtMinPts->value();
    p.extMethodIndex = m_comboExtMethod->currentIndex();
    p.extEuclideanTol = m_spinEuclideanTol->value();
    p.extRgNeighbors = m_spinRgNeighbors->value();
    p.extRgSmoothness = m_spinRgSmoothness->value();
    p.useRansac = m_chkUseRansac->isChecked();
    p.ransacDistThresh = m_spinRansacDist->value();
    p.onlyExtractBody = m_chkOnlyExtract->isChecked();
    p.useMlsUpsampling = m_chkUseMls->isChecked();
    p.mlsSearchRadius = m_spinMlsSearchRadius->value();
    p.mlsUpsamplingRadius = m_spinMlsUpsampleRadius->value();
    p.mlsUpsamplingStep = m_spinMlsUpsampleStep->value();

    p.girthThick = static_cast<float>(m_spinGirthThick->value());
    p.skelStep = static_cast<float>(m_spinSkelStep->value());
    p.skelRadius = static_cast<float>(m_spinSkelRadius->value());
    p.heightAngle = static_cast<float>(m_spinHeightAngle->value());
    return p;
}

void BatchModePage::onStartBatch() {
    BatchParams params = collectParams();

    if (params.inputDir.isEmpty() || params.outputDir.isEmpty()) {
        QMessageBox::warning(this, "Warning", "Please set both input and output directories.");
        return;
    }

    QDir inDir(params.inputDir);
    QStringList subDirs = inDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
    if (subDirs.isEmpty()) {
        QMessageBox::information(this, "Info", "No subfolders were found in the input directory.");
        return;
    }

    QStringList folderPaths;
    for (const QString &d : subDirs) {
        folderPaths << inDir.absoluteFilePath(d);
    }

    m_btnStart->setEnabled(false);
    m_btnStart->setText("Running...");
    m_btnStop->setEnabled(true);
    m_btnStop->setText("Stop");
    m_progressBar->setMaximum(folderPaths.size());
    m_progressBar->setValue(0);
    m_console->clear();
    onWorkerLog(QString("Found %1 folders to process").arg(folderPaths.size()), "INFO");

    m_worker = new BatchWorker(folderPaths, params, this);
    connect(m_worker, &BatchWorker::progressUpdated, this, &BatchModePage::onWorkerProgress);
    connect(m_worker, &BatchWorker::logMessage, this, &BatchModePage::onWorkerLog);
    connect(m_worker, &BatchWorker::batchFinished, this, &BatchModePage::onWorkerFinished);
    m_worker->start();
}

void BatchModePage::onStopBatch() {
    if (m_worker && m_worker->isRunning()) {
        m_worker->stop();
        m_btnStop->setEnabled(false);
        m_btnStop->setText("Stopping...");
        onWorkerLog("Stop requested. Waiting for the current folder to finish...", "WARN");
    }
}

void BatchModePage::onWorkerProgress(int current, int total) {
    Q_UNUSED(total);
    m_progressBar->setValue(current);
}

void BatchModePage::onWorkerLog(const QString& msg, const QString& type) {
    QString timeStr = QDateTime::currentDateTime().toString("[HH:mm:ss]");
    QString color = "#d4d4d4";
    if (type == "WARN") {
        color = "#e5c07b";
    } else if (type == "ERROR") {
        color = "#e06c75";
    } else if (type == "SUCCESS") {
        color = "#98c379";
    } else if (type == "ALGO") {
        color = "#61afef";
    }

    QString html = QString("<span style='color:#5c6370;'>%1</span> "
                           "<span style='color:%2; font-weight:bold;'>[%3]</span> "
                           "<span style='color:#d4d4d4;'>%4</span>")
                           .arg(timeStr, color, type, msg);
    m_console->append(html);
    QScrollBar *sb = m_console->verticalScrollBar();
    sb->setValue(sb->maximum());
}

void BatchModePage::onWorkerFinished(int successCount, int totalCount) {
    m_btnStart->setEnabled(true);
    m_btnStart->setText("Start Batch");
    m_btnStop->setEnabled(false);
    m_btnStop->setText("Stop");

    QMessageBox::information(
        this,
        "Batch Finished",
        QString("Batch processing completed.\nSuccess: %1\nTotal: %2").arg(successCount).arg(totalCount));

    m_worker->deleteLater();
    m_worker = nullptr;
}
