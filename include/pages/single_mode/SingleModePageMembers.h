/*
 * 文件说明：
 * 该文件定义 `SingleModePage` 的成员状态基类 `SingleModePageMembers`。
 *
 * 设计目的：
 * 1. 将页面类中数量众多的成员变量按模块集中收纳，减轻主头文件阅读压力；
 * 2. 保持原有成员命名不变，确保现有实现文件无需因头文件瘦身而大规模改写；
 * 3. 通过“页面接口类 + 成员状态基类”的方式，让 `SingleModePage.h` 主要承担接口声明职责。
 */
#pragma once

#include <QList>
#include <QMap>
#include <QNetworkAccessManager>
#include <QTimer>

#include <Eigen/Core>

#include <opencv2/core/mat.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vector>

#include "OpenPoseInferencer.h"
#include "core/PointCloudAlgo.h"

class QCheckBox;
class QComboBox;
class QDoubleSpinBox;
class QLabel;
class QLineEdit;
class QVTKOpenGLNativeWidget;
class QPushButton;
class QSpinBox;
class QStackedWidget;
class QTextEdit;
class QWidget;
class vtkGenericOpenGLRenderWindow;
class vtkRenderer;

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class SingleModePageMembers {
protected:
    // ==========================================================
    // --- 核心基础组件 (UI 布局、渲染器、日志) ---
    // ==========================================================
    QWidget *leftPanel = nullptr;
    QWidget *centerPanel = nullptr;
    QWidget *rightPanel = nullptr;
    QVTKOpenGLNativeWidget *m_vtkWidget = nullptr;
    QTextEdit *m_console = nullptr;
    vtkSmartPointer<vtkGenericOpenGLRenderWindow> m_vtkRenderWindow;
    vtkSmartPointer<vtkRenderer> m_vtkRenderer;
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    QTimer *m_refreshTimer = nullptr;

    // ==========================================================
    // --- 模块 0: 数据 IO 与底层状态 ---
    // ==========================================================
    QMap<QString, QLineEdit*> m_fileInputs;
    QMap<QString, QCheckBox*> m_layerChecks;
    QMap<QString, PointCloudT::Ptr> m_cloudData;
    cv::Mat m_topColorImage;
    cv::Mat m_topAlignedDepthImage;
    QMap<QString, CameraIntrinsics> m_intrinsicsMap;

    // ==========================================================
    // --- 模块 1: 点云预处理 ---
    // ==========================================================
    QDoubleSpinBox* m_spinLeafSize = nullptr;
    QDoubleSpinBox* m_spinStdDev = nullptr;
    QSpinBox* m_spinMeanK = nullptr;
    QDoubleSpinBox* m_spinClipRadius = nullptr;

    // ==========================================================
    // --- 模块 2: 配准与融合 ---
    // ==========================================================
    QMap<QString, Eigen::Matrix4d> m_transforms;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_mergedCloudRGB;

    QComboBox* m_comboRegMethod = nullptr;
    QComboBox* m_comboRegTarget = nullptr;
    QComboBox* m_comboMatrixView = nullptr;
    QTextEdit* m_textMatrix = nullptr;
    QPushButton *m_btnRunReg = nullptr;
    QMap<QString, QCheckBox*> m_sourceChecks;

    QWidget *m_icpParamsWidget = nullptr;
    QSpinBox *m_spinIcpIter = nullptr;
    QDoubleSpinBox *m_spinIcpDist = nullptr;

    QWidget *m_ndtParamsWidget = nullptr;
    QDoubleSpinBox *m_spinNdtRes = nullptr;
    QDoubleSpinBox *m_spinNdtStep = nullptr;
    QSpinBox *m_spinNdtIter = nullptr;

    QWidget *m_gicpParamsWidget = nullptr;
    QSpinBox *m_spinGicpIter = nullptr;
    QDoubleSpinBox *m_spinGicpDist = nullptr;
    QDoubleSpinBox *m_spinGicpEps = nullptr;

    // ==========================================================
    // --- 模块 3: 主体提取 ---
    // ==========================================================
    QComboBox *m_comboBodyExtractMode = nullptr;
    QStackedWidget *m_extractModeStack = nullptr;
    QLineEdit *m_leSegServiceUrl = nullptr;
    QPushButton *m_btnExtractBody = nullptr;

    QDoubleSpinBox *m_spinBoxMinX = nullptr;
    QDoubleSpinBox *m_spinBoxMinY = nullptr;
    QDoubleSpinBox *m_spinBoxMinZ = nullptr;
    QDoubleSpinBox *m_spinBoxMaxX = nullptr;
    QDoubleSpinBox *m_spinBoxMaxY = nullptr;
    QDoubleSpinBox *m_spinBoxMaxZ = nullptr;
    QDoubleSpinBox *m_spinBoxRotZ = nullptr;
    QSpinBox *m_spinExtMinPts = nullptr;

    QCheckBox *m_chkUseRansac = nullptr;
    QDoubleSpinBox *m_spinRansacDist = nullptr;

    QComboBox *m_comboExtMethod = nullptr;
    QWidget *m_euclideanParamsWidget = nullptr;
    QWidget *m_rgParamsWidget = nullptr;

    QCheckBox* m_chkMlsUpsampling = nullptr;
    QDoubleSpinBox* m_spinMlsRadius = nullptr;
    QDoubleSpinBox* m_spinMlsUpsampleRadius = nullptr;
    QDoubleSpinBox* m_spinMlsUpsampleStep = nullptr;

    QDoubleSpinBox *m_spinEuclideanTol = nullptr;
    QSpinBox *m_spinRgNeighbors = nullptr;
    QDoubleSpinBox *m_spinRgSmoothness = nullptr;

    // ==========================================================
    // --- 模块 4: AI 推理与关键点 ---
    // ==========================================================
    OpenPoseInferencer m_openpose;
    QNetworkAccessManager *m_networkManager = nullptr;

    QPushButton *m_btnRunAI3D = nullptr;
    QPushButton *m_btnRunAI2D = nullptr;
    QLineEdit *m_leModelPath = nullptr;
    QList<QLabel*> m_kpBadges;

    bool m_isManualPickingMode = false;
    int m_currentPickIndex = 0;
    std::vector<Eigen::Vector3f> m_keypoints;

    // ==========================================================
    // --- 模块 5: 体尺测量 ---
    // ==========================================================
    QDoubleSpinBox *m_spinGirthThick = nullptr;
    QDoubleSpinBox *m_spinSkelStep = nullptr;
    QDoubleSpinBox *m_spinSkelRadius = nullptr;
    QDoubleSpinBox *m_spinHeightAngle = nullptr;

    BodySizeResults m_latestResults;
    bool m_hasResults = false;
};
