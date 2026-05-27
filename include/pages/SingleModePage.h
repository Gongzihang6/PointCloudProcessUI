/*
 * 文件说明：
 * 该文件声明单体模式页面 `SingleModePage`，负责组织点云加载、预处理、配准融合、
 * 主体提取、关键点检测、体尺测量与结果导出等完整业务流程。
 *
 * 重构说明：
 * 1. 保持对外类名、头文件路径与信号槽接口不变，确保页面集成方式不受影响；
 * 2. 将原先超大实现文件拆分为 UI、数据 IO、配准、提取、AI/测量、导出等多个实现文件；
 * 3. 将成员状态继续迁移到独立的 `SingleModePageMembers.h` 中，让本头文件更聚焦接口；
 * 4. 清理历史遗留且未使用的声明，统一 MLS 相关成员命名风格。
 */
#pragma once

#include <QString>
#include <QWidget>

class QResizeEvent;

#include "pages/single_mode/SingleModePageMembers.h"

class SingleModePage : public QWidget, private SingleModePageMembers {
    Q_OBJECT

public:
    explicit SingleModePage(QWidget *parent = nullptr);
    QString getFullPath(const QString& camKey) const;

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void onBrowseFile(const QString& key);
    void onLoadFolder();
    void onClearFiles();
    void onLayerToggle(const QString& layerId, bool checked);
    void onSetIntrinsics();

    void onRunPreprocess();

    void onMatrixTargetChanged(int index);
    void onMatrixTextChanged();
    void onExecuteRegistration();

    void onExtractBody();

    void onRunAIInference();
    void onRunAIInference2D();

    void onCalculateBodySize();

    void onExportMergedCloud();
    void onExportBodyCloud();
    void onExportReport();
    void onExportVizCloud();

private:
    void initLeftPanel();
    void initCenterView();
    void initRightPanel();

    void log(const QString& msg, const QString& type = "INFO");
    void update3DView();
    void getCameraColor(const QString& camName, int& r, int& g, int& b);

    void loadCloudToMemory(const QString& key, const QString& filePath);
    void initDefaultIntrinsics();

    void applyPreprocessToMemory();

    QString matrixToString(const Eigen::Matrix4d& mat);
    Eigen::Matrix4d stringToMatrix(const QString& text);
    void initDefaultMatrices();

    void applyBodyCloudResult(const PointCloudT::Ptr& bodyCloud, const QString& sourceLabel);
    void resetMeasurementState();
    QString writePointCloudToTempPcd(const PointCloudT::Ptr& cloud, QString& errorMessage) const;

    void updateBadgeStyle(int index, int state);
    void onManualPointPicked(double x, double y, double z);
    bool prepareKeypointsCloud();
    void drawKeypointsInViewer(const std::vector<Eigen::Vector3f>& kps);

    void drawMeasurements();
    void clearMeasurements();
};
