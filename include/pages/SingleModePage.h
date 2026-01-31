#pragma once
#include <QWidget>
#include <QMap>
#include <QTimer>
#include "core/PointCloudAlgo.h"
// PCL 相关头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// 前置声明，减少头文件依赖
class QLineEdit;
class QPushButton;
class QCheckBox; // 新增
class QDoubleSpinBox;
class QSpinBox;

// 定义点云类型别名，方便书写
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class SingleModePage : public QWidget {
    Q_OBJECT // 务必保留这个宏，它是信号槽的基础
public:
    explicit SingleModePage(QWidget *parent = nullptr);
    // [新增] 获取某个相机对应的真实完整路径（供给算法使用）
    QString getFullPath(const QString& camKey) const;

protected:
    // [新增] 重写调整大小事件，确保 PCL 窗口跟随缩放
    void resizeEvent(QResizeEvent *event) override;

private slots:
    // --- 槽函数 (响应 UI 操作) ---
    void onBrowseFile(const QString& key); // 单个文件浏览
    void onLoadFolder();                   // 文件夹批量加载
    // [新增] 清空所有文件槽
    void onClearFiles();
    // [新增] 响应图层复选框状态改变
    void onLayerToggle(const QString& layerId, bool checked);
    // [新增] 执行预处理预览
    void onRunPreprocess();

private:
    void initLeftPanel();
    void initCenterView();
    void initRightPanel();

    // [新增] 加载点云到内存的辅助函数
    void loadCloudToMemory(const QString& key, const QString& filePath);
    // [新增] 更新 3D 视图的辅助函数
    void update3DView();

    // --- UI 控件指针 (方便后续访问) ---
    QWidget *leftPanel;
    QWidget *centerPanel;
    QWidget *rightPanel;

    // 使用 Map 管理 5 个输入框，key 为 "Top", "LB", "LT" 等
    QMap<QString, QLineEdit*> m_fileInputs; 

    // [新增] 管理复选框，方便后续代码控制勾选状态
    QMap<QString, QCheckBox*> m_layerChecks; 

    // [修改] 不再使用 QVTKWidget，改用普通 Widget 作为容器
    QWidget *m_vtkContainer; 
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    
    // [新增] 刷新定时器 (替代 QVTKWidget 的自动刷新)
    QTimer *m_refreshTimer;

    // [新增] 内存中缓存的点云数据 (Key: "Top", "LB", "Merged" 等)
    // 这样勾选时直接从内存拿数据，不用读硬盘，速度快
    QMap<QString, PointCloudT::Ptr> m_cloudData;

    // [新增] 预处理参数控件指针
    QDoubleSpinBox* m_spinLeafSize;   // 下采样
    QDoubleSpinBox* m_spinStdDev;     // SOR 标准差倍数
    QSpinBox* m_spinMeanK;      // SOR 邻近点数
    QDoubleSpinBox* m_spinClipRadius; // 半径裁剪

    void applyPreprocessToMemory();
};