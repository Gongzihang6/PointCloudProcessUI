#pragma once
#include <QWidget>
#include <QMap>
#include <QTimer>
#include <QNetworkAccessManager>
#include <QNetworkReply>
#include <QHttpMultiPart>
#include <QList>
#include <QFileDialog>
#include <QFile>
#include <QTextStream>
#include <QDateTime>
#include <QMessageBox>
#include "core/PointCloudAlgo.h"
// PCL 相关头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>

// 前置声明，减少头文件依赖
class QLineEdit;
class QPushButton;
class QCheckBox; 
class QDoubleSpinBox;
class QSpinBox;
class QTextEdit;
class QComboBox; 
class QLabel;

// 定义点云类型别名，方便书写
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class SingleModePage : public QWidget {
    Q_OBJECT // 务必保留这个宏，它是信号槽的基础
public:
    explicit SingleModePage(QWidget *parent = nullptr);
    // 获取某个相机对应的真实完整路径（供给算法使用）
    QString getFullPath(const QString& camKey) const;

protected:
    // 重写调整大小事件，确保 PCL 窗口跟随缩放
    void resizeEvent(QResizeEvent *event) override;

private slots:
    // --- 槽函数 (响应 UI 操作) ---
    void onBrowseFile(const QString& key); // 单个文件浏览
    void onLoadFolder();                   // 文件夹批量加载
    // 清空所有文件槽
    void onClearFiles();
    // 响应图层复选框状态改变，控制 3D 视图显示/隐藏对应点云
    void onLayerToggle(const QString& layerId, bool checked);
    // 执行预处理预览
    void onRunPreprocess();
    // 当下拉框改变相机时 (例如从 LB 切到 RT)，更新矩阵输入框显示的数值
    void onMatrixTargetChanged(int index);
    
    // 当矩阵输入框内容改变时，保存到内存
    void onMatrixTextChanged();

    // 执行配准与融合
    void onExecuteRegistration();

    // 执行主体提取的槽函数
    void onExtractBody();

    // 运行关键点预测的槽函数
    void onRunAIInference(); 

    // 计算体尺参数的槽函数
    void onCalculateBodySize();

    // 导出模块的三个槽函数
    void onExportMergedCloud();
    void onExportBodyCloud();
    void onExportReport();
private:
    // UI布局初始化函数，左侧面板、中心视图、右侧面板分别初始化
    void initLeftPanel();
    void initCenterView();
    void initRightPanel();

    QTextEdit *m_console; // 日志窗口指针

    // 辅助日志函数
    void log(const QString& msg, const QString& type = "INFO");

    // 加载点云到内存的辅助函数
    void loadCloudToMemory(const QString& key, const QString& filePath);
    // 更新 3D 视图的辅助函数
    void update3DView();

    // --- UI 控件指针 (方便后续访问) ---
    QWidget *leftPanel;
    QWidget *centerPanel;
    QWidget *rightPanel;

    // 使用 Map 管理 5 个输入框，key 为 "Top", "LB", "LT" 等，value为对应的 QLineEdit 输入框指针
    QMap<QString, QLineEdit*> m_fileInputs; 

    // 管理复选框，方便后续代码控制勾选状态
    QMap<QString, QCheckBox*> m_layerChecks; 

    // 不再使用 QVTKWidget，改用普通 Widget 作为容器
    QWidget *m_vtkContainer; 
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    
    // 刷新定时器 (替代 QVTKWidget 的自动刷新)
    QTimer *m_refreshTimer;

    // 内存中缓存的点云数据 (Key: "Top", "LB", "Merged" 等)
    // 这样勾选时直接从内存拿数据，不用读硬盘，速度快
    QMap<QString, PointCloudT::Ptr> m_cloudData;

    // 预处理参数控件指针
    QDoubleSpinBox* m_spinLeafSize;   // 下采样
    QDoubleSpinBox* m_spinStdDev;     // SOR 标准差倍数
    QSpinBox* m_spinMeanK;      // SOR 邻近点数
    QDoubleSpinBox* m_spinClipRadius; // 半径裁剪

    void applyPreprocessToMemory();

    // 存储每个相机的变换矩阵 (Key: "LB", "LT", "RB", "RT")
    // Top 相机不需要矩阵，因为它是基准 (Identity)
    QMap<QString, Eigen::Matrix4d> m_transforms;

    // 控件指针
    QComboBox* m_comboRegMethod;   // 算法选择 (Manual/ICP)
    QComboBox* m_comboMatrixView;  // 选择当前编辑哪个矩阵
    QTextEdit* m_textMatrix;       // 矩阵输入框
    
    // 勾选哪些源云参与配准
    QMap<QString, QCheckBox*> m_sourceChecks; // Key: "LB", "LT"...
    
    // 辅助函数：将矩阵转为字符串显示
    QString matrixToString(const Eigen::Matrix4d& mat);
    // 辅助函数：将字符串解析为矩阵
    Eigen::Matrix4d stringToMatrix(const QString& text);
    
    // 初始化默认矩阵 (硬编码你的参数)
    void initDefaultMatrices();

    QComboBox* m_comboRegTarget;   // 配准的目标 (Top, LB, LT...)

    // 专门用于存储带颜色的融合结果 (仅用于显示)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_mergedCloudRGB;
    
    // 辅助函数：根据相机名称获取 RGB 颜色
    void getCameraColor(const QString& camName, int& r, int& g, int& b);

    // 主体提取参数控件
    QDoubleSpinBox *m_spinExtractTol;
    QSpinBox *m_spinExtractMinSize;
    QDoubleSpinBox *m_spinRansacThresh; // 新增地面滤除阈值控件

    void drawKeypointsInViewer(const std::vector<Eigen::Vector3f>& kps);    // 用于可视化模型预测的关键点位置

    // 网络请求管理器 (用于发送点云到 Python 服务端)
    QNetworkAccessManager *m_networkManager;
    
    // 存储关键点状态标签的指针，方便后续变绿
    QList<QLabel*> m_kpBadges;

    // 运行 AI 按钮的指针
    QPushButton *m_btnRunAI;

    // 手动拾取状态控制
    bool m_isManualPickingMode = false;
    int m_currentPickIndex = 0;
    std::vector<Eigen::Vector3f> m_keypoints; // 存放当前的 6 个关键点 (AI预测的或手动拾取的)

    // UI 状态辅助函数
    void updateBadgeStyle(int index, int state); // state: 0=灰(未开始), 1=蓝(正在拾取), 2=绿(已完成)
    void onManualPointPicked(double x, double y, double z); // 拾取到点后的处理逻辑

    // 专门用于准备(或检查)关键点检测云的辅助函数
    bool prepareKeypointsCloud();

    // 体尺参数控件
    QDoubleSpinBox *m_spinGirthThick;  // 切片厚度
    QDoubleSpinBox *m_spinSkelStep;    // 骨架步长
    QDoubleSpinBox *m_spinSkelRadius;  // 骨架搜索半径
    QDoubleSpinBox *m_spinHeightAngle; // 地面法线角度阈值

    // 缓存最新的体尺计算结果，供导出 CSV 时使用
    BodySizeResults m_latestResults;
    bool m_hasResults = false; // 标记是否已经成功计算过
};