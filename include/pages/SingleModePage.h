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
#include <QFutureWatcher> // 异步监视器
#include <QtConcurrent>
#include <vtkCellPicker.h>
#include <QDialog>
#include <QTableWidget>
#include <QHeaderView>
#include <QGroupBox>
#include <QFormLayout>
// PCL 相关头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d_omp.h>

#include <opencv2/opencv.hpp>
#include "OpenPoseInferencer.h"

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

// 【严格统一声明】
static PointCloudT::Ptr extractLargestCluster(
    PointCloudT::Ptr input_cloud, 
    const ExtractionParams& params,
    std::function<void(const QString&, const QString&)> logger = nullptr);


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
    // ==========================================================
    // --- 槽函数区 (响应 UI 操作，按业务模块划分) ---
    // ==========================================================

    // 【模块 0: IO 与图层控制】
    void onBrowseFile(const QString& key); // 单个文件浏览
    void onLoadFolder();                   // 文件夹批量加载
    void onClearFiles();                   // 清空所有文件槽
    void onLayerToggle(const QString& layerId, bool checked); // 响应图层复选框状态改变，控制 3D 视图显示/隐藏对应点云
    void onSetIntrinsics();                // [新增] 弹出内参设置对话框

    // 【模块 1: 点云预处理】
    void onRunPreprocess();                // 执行预处理预览

    // 【模块 2: 配准与融合】
    void onMatrixTargetChanged(int index); // 当下拉框改变相机时 (例如从 LB 切到 RT)，更新矩阵输入框显示的数值
    void onMatrixTextChanged();            // 当矩阵输入框内容改变时，保存到内存
    void onExecuteRegistration();          // 执行配准与融合

    // 【模块 3: 主体精细提取】
    void onExtractBody();                  // 执行主体提取的槽函数

    // 【模块 4: 关键点与 AI 检测】
    void onRunAIInference();               // 运行关键点预测的槽函数 (3D C/S 服务端架构)
    void onRunAIInference2D();             // 2D 本地推理 (本地 ONNX 引擎)

    // 【模块 5: 体尺测量】
    void onCalculateBodySize();            // 计算体尺参数的槽函数

    // 【模块 6: 结果导出】
    void onExportMergedCloud();            // 导出模块的三个槽函数：导出融合点云
    void onExportBodyCloud();              // 导出主体点云
    void onExportReport();                 // 导出 CSV/TXT 测量报告
    void onExportVizCloud();               // [新增] 导出可视化点云


private:
    // ==========================================================
    // --- 核心基础组件 (UI 布局、渲染器、日志) ---
    // ==========================================================
    // UI布局初始化函数，左侧面板、中心视图、右侧面板分别初始化
    void initLeftPanel();
    void initCenterView();
    void initRightPanel();

    // --- UI 控件指针 (方便后续访问) ---
    QWidget *leftPanel;
    QWidget *centerPanel;
    QWidget *rightPanel;

    QTextEdit *m_console; // 日志窗口指针
    // 辅助日志函数
    void log(const QString& msg, const QString& type = "INFO");

    // 不再使用 QVTKWidget，改用普通 Widget 作为容器
    QWidget *m_vtkContainer; 
    pcl::visualization::PCLVisualizer::Ptr m_viewer;
    
    // 刷新定时器 (替代 QVTKWidget 的自动刷新)
    QTimer *m_refreshTimer;
    // 更新 3D 视图的辅助函数
    void update3DView();


    // ==========================================================
    // --- 模块 0: 数据 IO 与底层内存状态管理 ---
    // ==========================================================
    // 使用 Map 管理 5 个输入框，key 为 "Top", "LB", "LT" 等，value为对应的 QLineEdit 输入框指针
    QMap<QString, QLineEdit*> m_fileInputs; 

    // 管理复选框，方便后续代码控制勾选状态
    QMap<QString, QCheckBox*> m_layerChecks; 

    // 内存中缓存的点云数据 (Key: "Top", "LB", "Merged" 等)
    // 这样勾选时直接从内存拿数据，不用读硬盘，速度快
    QMap<QString, PointCloudT::Ptr> m_cloudData;
    
    // [新增] 用于存放 Top 相机的彩色图和对齐深度图
    cv::Mat m_topColorImage;
    cv::Mat m_topAlignedDepthImage;

    // [新增] 存储当前系统的相机内参
    QMap<QString, CameraIntrinsics> m_intrinsicsMap;

    // 辅助函数
    void loadCloudToMemory(const QString& key, const QString& filePath); // 加载点云到内存的辅助函数
    void initDefaultIntrinsics(); // 初始化硬编码的内参


    // ==========================================================
    // --- 模块 1: 点云预处理 (Preprocess) ---
    // ==========================================================
    // 预处理参数控件指针
    QDoubleSpinBox* m_spinLeafSize;   // 下采样
    QDoubleSpinBox* m_spinStdDev;     // SOR 标准差倍数
    QSpinBox* m_spinMeanK;            // SOR 邻近点数
    QDoubleSpinBox* m_spinClipRadius; // 半径裁剪

    void applyPreprocessToMemory();   // 应用预处理参数


    // ==========================================================
    // --- 模块 2: 配准与融合 (Registration) ---
    // ==========================================================
    // 存储每个相机的变换矩阵 (Key: "LB", "LT", "RB", "RT")
    // Top 相机不需要矩阵，因为它是基准 (Identity)
    QMap<QString, Eigen::Matrix4d> m_transforms;

    // 专门用于存储带颜色的融合结果 (仅用于显示)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_mergedCloudRGB;

    // 全局控制控件
    QComboBox* m_comboRegMethod;   // 算法选择 (Manual/ICP)
    QComboBox* m_comboRegTarget;   // 配准的目标 (Top, LB, LT...)
    QComboBox* m_comboMatrixView;  // 选择当前编辑哪个矩阵
    QTextEdit* m_textMatrix;       // 矩阵输入框
    QPushButton *m_btnRunReg;      // [新增] 将“执行配准”按钮提升为成员变量
    // 勾选哪些源云参与配准
    QMap<QString, QCheckBox*> m_sourceChecks; // Key: "LB", "LT"...

    // 动态参数面板控件: ICP参数面板
    QWidget *m_icpParamsWidget;       
    QSpinBox *m_spinIcpIter;          // ICP最大迭代次数
    QDoubleSpinBox *m_spinIcpDist;    // ICP最大对应距离

    // 动态参数面板控件: NDT参数面板
    QWidget *m_ndtParamsWidget;       
    QDoubleSpinBox *m_spinNdtRes;     // NDT网格分辨率
    QDoubleSpinBox *m_spinNdtStep;    // NDT搜索步长
    QSpinBox *m_spinNdtIter;          // NDT最大迭代次数

    // 动态参数面板控件: [新增] G-ICP 参数面板控件
    QWidget *m_gicpParamsWidget;
    QSpinBox *m_spinGicpIter;
    QDoubleSpinBox *m_spinGicpDist;
    QDoubleSpinBox *m_spinGicpEps;

    // 辅助函数
    QString matrixToString(const Eigen::Matrix4d& mat);     // 将矩阵转为字符串显示
    Eigen::Matrix4d stringToMatrix(const QString& text);    // 将字符串解析为矩阵
    void initDefaultMatrices();                             // 初始化默认矩阵 (硬编码你的参数)
    void getCameraColor(const QString& camName, int& r, int& g, int& b); // 根据相机名称获取 RGB 颜色


    // ==========================================================
    // --- 模块 3: 主体精细提取 (Extraction) ---
    // ==========================================================
    // [新增] 提取模块 UI 控件：共用参数 (包围盒与最小点数)
    QDoubleSpinBox *m_spinBoxMinX, *m_spinBoxMinY, *m_spinBoxMinZ;
    QDoubleSpinBox *m_spinBoxMaxX, *m_spinBoxMaxY, *m_spinBoxMaxZ;
    QDoubleSpinBox *m_spinBoxRotZ; // [新增] Z轴旋转角度控件
    QSpinBox *m_spinExtMinPts;

    // [新增] RANSAC 剔除控件
    QCheckBox *m_chkUseRansac;
    QDoubleSpinBox *m_spinRansacDist;

    // 算法选择与动态面板容器
    QComboBox *m_comboExtMethod;
    QWidget *m_euclideanParamsWidget;
    QWidget *m_rgParamsWidget;

    // [新增] 将 MLS UI 控件提升为类成员变量，以便跨函数读取
    QCheckBox* cbMlsUpsampling;
    QDoubleSpinBox* spinMlsRadius;
    QDoubleSpinBox* spinUpsampleRadius;
    QDoubleSpinBox* spinUpsampleStep;

    // 欧式聚类专属控件
    QDoubleSpinBox *m_spinEuclideanTol;

    // 区域生长专属控件
    QSpinBox *m_spinRgNeighbors;
    QDoubleSpinBox *m_spinRgSmoothness;

    // 历史遗留版本提取参数控件 (保留旧注释供兼容/备用参考)
    QDoubleSpinBox *m_spinExtractTol;
    QSpinBox *m_spinExtractMinSize;
    QDoubleSpinBox *m_spinRansacThresh; // 新增地面滤除阈值控件


    // ==========================================================
    // --- 模块 4: 关键点与测量准备 (AI & Picking) ---
    // ==========================================================
    // AI 引擎与服务端通讯组件
    OpenPoseInferencer m_openpose;
    QNetworkAccessManager *m_networkManager; // 网络请求管理器 (用于发送点云到 Python 服务端)

    // UI 控件
    QPushButton *m_btnRunAI3D;    // 运行 3D AI 按钮
    QPushButton *m_btnRunAI2D;    // 运行 2D AI 按钮
    QLineEdit   *m_leModelPath;   // ONNX 本地权重路径
    QList<QLabel*> m_kpBadges;    // 存储关键点状态标签的指针，方便后续变绿

    // 核心状态数据
    bool m_isManualPickingMode = false; // 手动拾取状态控制
    int m_currentPickIndex = 0;
    std::vector<Eigen::Vector3f> m_keypoints; // 存放当前的 6 个关键点 (AI预测的或手动拾取的)

    // 交互与渲染辅助函数
    void updateBadgeStyle(int index, int state); // state: 0=灰(未开始), 1=蓝(正在拾取), 2=绿(已完成)
    void onManualPointPicked(double x, double y, double z); // 拾取到点后的处理逻辑
    bool prepareKeypointsCloud(); // 专门用于准备(或检查)关键点检测云的辅助函数
    void drawKeypointsInViewer(const std::vector<Eigen::Vector3f>& kps); // 用于可视化模型预测的关键点位置


    // ==========================================================
    // --- 模块 5: 体尺计算核心 (Measurement) ---
    // ==========================================================
    // 体尺参数微调控件
    QDoubleSpinBox *m_spinGirthThick;  // 切片厚度
    QDoubleSpinBox *m_spinSkelStep;    // 骨架步长
    QDoubleSpinBox *m_spinSkelRadius;  // 骨架搜索半径
    QDoubleSpinBox *m_spinHeightAngle; // 地面法线角度阈值

    // 计算结果数据缓存
    BodySizeResults m_latestResults;   // 缓存最新的体尺计算结果，供导出 CSV 时使用
    bool m_hasResults = false;         // 标记是否已经成功计算过

    // 渲染控制
    void drawMeasurements();  // 绘制测量结果
    void clearMeasurements(); // 清除测量结果

};