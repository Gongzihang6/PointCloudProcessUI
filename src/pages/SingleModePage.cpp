#include "pages/SingleModePage.h"
#include "widgets/CollapsibleBox.h"
#include <QHBoxLayout>
#include <QVBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QCheckBox>
#include <QSpinBox>
#include <QPushButton>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QTextEdit>
#include <QScrollArea>
#include <QButtonGroup>  
#include <QFrame>        
#include <QStyle>
#include <QFileDialog> // 文件对话框
#include <QDir>        // 目录操作
#include <QDebug>      // 用于调试打印
#include <vtkRenderWindow.h> // VTK 标准渲染窗口
#include <vtkGenericOpenGLRenderWindow.h> // 必须包含这个
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkCommand.h>
#include <vtkPointPicker.h>
#include <vtkCamera.h>
#include <vtkRenderer.h>
#include <pcl/io/pcd_io.h>                // 用于读取 PCD
#include <QMessageBox>                    // 用于报错提示
#include <QPixmap>
#include <QIcon>
#include <QColor>
#include <QDoubleSpinBox>
#include <QSplitter>
#include <QDateTime>
#include <QScrollBar>


// ==========================================================
// [新增] 定义并行配准任务的数据结构
// ==========================================================
struct RegTaskInput {
    QString srcKey;
    QString targetKey;
    PointCloudT::Ptr cloudSrc;
    PointCloudT::Ptr cloudTarget;
    Eigen::Matrix4d initialGuess;
    int methodIndex;
    int algoType;
    int icpIter;
    double icpDist;
    float ndtRes;
    float ndtStep;
    int ndtIter;

    // [新增]
    int gicpIter;
    double gicpDist;
    double gicpEps;
};

struct RegTaskOutput {
    QString srcKey;
    PointCloudT::Ptr cloudAlignedLocal;
    Eigen::Matrix4d finalTransform;
    std::vector<std::pair<QString, QString>> logs; // 收集该线程产生的所有日志
    bool valid = false;
};

// ==========================================================
// [新增] 纯后台工作函数 (剥离 UI，完全在后台子线程中并行执行)
// ==========================================================
RegTaskOutput processRegistrationWorker(const RegTaskInput& input) {
    RegTaskOutput output;
    output.srcKey = input.srcKey;
    output.valid = true;
    output.finalTransform = input.initialGuess; // 默认使用初始矩阵
    
    // 伪装一个 logger，将日志存入缓存而不是直接操作 UI
    auto logBridge = [&output](const QString& msg, const QString& type) {
        output.logs.push_back({msg, type});
    };

    if (input.methodIndex == 0) { // 手动矩阵
        output.cloudAlignedLocal = PointCloudAlgo::transformCloud(input.cloudSrc, input.initialGuess);
    } 
    else if (input.methodIndex == 1 || input.methodIndex == 2) { // ICP
        auto result = PointCloudAlgo::alignICP(input.cloudSrc, input.cloudTarget, input.initialGuess, input.icpIter, input.icpDist, input.algoType, logBridge);
        output.cloudAlignedLocal = result.first;
        output.finalTransform = result.second;
    } 
    else if (input.methodIndex == 3) { // NDT
        Eigen::Matrix4f guess_f = input.initialGuess.cast<float>();
        Eigen::Matrix4f final_f = PointCloudAlgo::refineRegistrationNDT(input.cloudSrc, input.cloudTarget, guess_f, input.ndtRes, input.ndtStep, input.ndtIter, logBridge);
        output.finalTransform = final_f.cast<double>();
        output.cloudAlignedLocal = PointCloudAlgo::transformCloud(input.cloudSrc, output.finalTransform);
    }
    else if (input.methodIndex == 4) {  // G-ICP
        auto result = PointCloudAlgo::alignGICP(
            input.cloudSrc, input.cloudTarget, input.initialGuess, 
            input.gicpIter, input.gicpDist, input.gicpEps, logBridge
        );
        output.cloudAlignedLocal = result.first;
        output.finalTransform = result.second;
    }
    return output;
}

// 自定义 VTK 交互拦截器：模仿 CloudCompare
class CloudCompareMouseCallback : public vtkCommand {
public:
    static CloudCompareMouseCallback* New() { return new CloudCompareMouseCallback; }
    
    // 保存默认交互样式的指针
    vtkInteractorStyleTrackballCamera* style = nullptr;
    // 用于向 UI 打印日志的回调
    std::function<void(const QString&, const QString&)> logger;

    // 状态指针与回调函数
    bool* isManualMode = nullptr;
    std::function<void(double, double, double)> onPointPicked;

    void Execute(vtkObject* caller, unsigned long eventId, void* callData) override {
        vtkRenderWindowInteractor* iren = vtkRenderWindowInteractor::SafeDownCast(caller);
        if (!iren || !style) return;

        // 获取鼠标当前在屏幕上的 2D 像素坐标
        int x = iren->GetEventPosition()[0];
        int y = iren->GetEventPosition()[1];

        switch (eventId) {
            // ==========================================
            // 1. 右键 -> 改为平移 (Pan)
            // ==========================================
            case vtkCommand::RightButtonPressEvent:
                style->FindPokedRenderer(x, y);
                if (style->GetCurrentRenderer()) {
                    style->StartPan();   // 启动平移
                    this->AbortFlagOn(); // [关键] 拦截事件，阻止 VTK 默认的右键缩放
                }
                break;
            case vtkCommand::RightButtonReleaseEvent:
                style->EndPan();
                this->AbortFlagOn();
                break;

            // ==========================================
            // 2. 中键 -> 改为缩放 (Dolly) - 交换原本的右键功能
            // ==========================================
            case vtkCommand::MiddleButtonPressEvent:
                style->FindPokedRenderer(x, y);
                if (style->GetCurrentRenderer()) {
                    style->StartDolly(); // 启动缩放
                    this->AbortFlagOn(); // 阻止 VTK 默认的中键平移
                }
                break;
            case vtkCommand::MiddleButtonReleaseEvent:
                style->EndDolly();
                this->AbortFlagOn();
                break;

            // ==========================================
            // 3. 左键双击 -> 拾取点并设置为旋转中心
            // ==========================================
            case vtkCommand::LeftButtonDoubleClickEvent:
            {
                style->FindPokedRenderer(x, y);
                vtkRenderer* ren = style->GetCurrentRenderer();
                if (ren) {
                    // [核心修复] 使用 vtkCellPicker，专治点云拾取难题
                    vtkSmartPointer<vtkCellPicker> picker = vtkSmartPointer<vtkCellPicker>::New();
                    picker->SetTolerance(0.01); // 设置屏幕容差(0.01=1%)，相当于鼠标周围的一个小圆圈，点中更容易
                    picker->Pick(x, y, 0.0, ren);

                    if (picker->GetCellId() != -1) {
                        // 成功拾取到点云上的点
                        double* pickPos = picker->GetPickPosition();
                        
                        ren->GetActiveCamera()->SetFocalPoint(pickPos[0], pickPos[1], pickPos[2]);
                        iren->Render(); // 触发重绘
                        
                        if (logger) {
                            QString msg = QString("📍 已设置旋转中心: (X:%.1f, Y:%.1f, Z:%.1f)")
                                          .arg(pickPos[0]).arg(pickPos[1]).arg(pickPos[2]);
                            logger(msg, "INFO");
                        }
                    }
                }
                this->AbortFlagOn(); // 拦截 VTK 的默认双击事件
                break;
            }

            default:
                break; // 其他事件不处理，交给 VTK 默认逻辑
        }
    }
};

SingleModePage::SingleModePage(QWidget *parent) : QWidget(parent) {
    // 初始化网络管理器
    m_networkManager = new QNetworkAccessManager(this);

    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

    initDefaultIntrinsics();
    // 初始化三个面板
    initLeftPanel();
    initCenterView();
    initRightPanel();

    layout->addWidget(leftPanel, 0); // Flex: 0
    layout->addWidget(centerPanel, 1); // Flex: 1 (占用主要空间)
    layout->addWidget(rightPanel, 0);
}

void SingleModePage::initLeftPanel() {
    leftPanel = new QWidget(this);
    leftPanel->setFixedWidth(280);
    leftPanel->setStyleSheet("background: #ffffff; border-right: 1px solid #dcdfe6;");
    
    auto *layout = new QVBoxLayout(leftPanel);
    
    // --- 文件输入区 ---
    QLabel *headerInput = new QLabel("输入文件 (Input Files)", leftPanel);
    headerInput->setStyleSheet("font-weight: bold; color: #606266; padding: 10px 0; border-bottom: 1px solid #eee;");
    layout->addWidget(headerInput);

    // 定义相机标识符
    QStringList cams = {"Top", "LB", "LT", "RB", "RT"};
    
    for(const auto& cam : cams) {
        auto *row = new QHBoxLayout();
        
        QLabel *lbl = new QLabel(cam); 
        lbl->setFixedWidth(30);
        
        QLineEdit *edit = new QLineEdit(); 
        edit->setPlaceholderText(cam + "_Camera.pcd");
        // [关键]：将 edit 指针存入成员变量 m_fileInputs
        m_fileInputs[cam] = edit; 

        QPushButton *btn = new QPushButton("📂"); 
        btn->setFixedWidth(30);
        
        // [关键]：连接信号槽
        // 使用 Lambda 表达式捕获 cam 字符串，这样我们知道是哪个按钮被点了
        connect(btn, &QPushButton::clicked, this, [this, cam](){
            onBrowseFile(cam);
        });

        row->addWidget(lbl); 
        row->addWidget(edit); 
        row->addWidget(btn);
        layout->addLayout(row);
    }
    
    // 底部按钮组容器
    QHBoxLayout *bottomBtnLayout = new QHBoxLayout();
    bottomBtnLayout->setSpacing(5); // 按钮间距

    // 1. 加载文件夹按钮
    QPushButton *loadFolderBtn = new QPushButton("📥 加载文件夹...");
    loadFolderBtn->setToolTip("自动扫描文件夹下的 _d_pc.pcd 文件");
    connect(loadFolderBtn, &QPushButton::clicked, this, &SingleModePage::onLoadFolder);
    
    // 2. 清空按钮
    QPushButton *clearBtn = new QPushButton("🗑️ 清空");
    clearBtn->setToolTip("清空所有已加载的文件路径");
    // 设置一点样式，让它看起来像警告操作（可选）
    clearBtn->setStyleSheet("color: #f56c6c;"); 
    connect(clearBtn, &QPushButton::clicked, this, &SingleModePage::onClearFiles);

    // 3. 内参设置按钮
    QPushButton *btnIntrinsics = new QPushButton("⚙️ 内参");
    btnIntrinsics->setToolTip("修改 RAW 深度图转换为点云的相机内参");
    connect(btnIntrinsics, &QPushButton::clicked, this, &SingleModePage::onSetIntrinsics);

    // 将按钮加入水平布局
    bottomBtnLayout->addWidget(loadFolderBtn, 4); // 比例 3
    bottomBtnLayout->addWidget(btnIntrinsics, 2); 
    bottomBtnLayout->addWidget(clearBtn, 2);      // 比例 1

    // 将水平布局加入主垂直布局
    layout->addLayout(bottomBtnLayout);

    layout->addStretch();


    // --- 图层区 ---
    layout->addSpacing(20);
    QLabel *headerLayer = new QLabel("视图图层 (Layers)", leftPanel);
    headerLayer->setStyleSheet("font-weight: bold; color: #606266; padding: 10px 0; border-top: 1px solid #eee;");
    layout->addWidget(headerLayer);

    // 定义图层 ID 和 显示名称的映射
    // 注意：ID 必须和 m_fileInputs 的 Key ("Top", "LB"...) 保持一致，方便联动
    struct LayerInfo { QString id; QString name; QString color; };
    QList<LayerInfo> layers = {
        {"Top", "Top Camera", "red"},
        {"LB",  "Left-Bottom", "green"},
        {"LT",  "Left-Top", "blue"},
        {"RB",  "Right-Bottom", "gold"},
        {"RT",  "Right-Top", "cyan"},
        // 下面这俩暂时还没数据，先占位
        {"Merged", "✨ 融合后点云", "white"},
        {"Body",   "🐷 提取主体", "pink"},
        {"Keypoints", "🎯 关键点检测云", "orange"},
        {"Measurements", "📏 体尺测量结果", "#9c27b0"}
    };

    for(const auto& layer : layers) {
        QCheckBox *chk = new QCheckBox(); // 先不传文字
        
        // [修复 Bug 1] 使用 QPixmap 绘制颜色块，代替 HTML
        QPixmap pixmap(14, 14);           // 创建一个 14x14 的图片
        pixmap.fill(QColor(layer.color)); // 填充颜色
        chk->setIcon(QIcon(pixmap));      // 设置为图标
        chk->setText(layer.name);         // 设置纯文本名字
        
        chk->setChecked(false); 
        m_layerChecks[layer.id] = chk;

        connect(chk, &QCheckBox::toggled, this, [this, layer](bool checked){
            onLayerToggle(layer.id, checked);
        });

        layout->addWidget(chk);
    }
    
    layout->addStretch();
}

void SingleModePage::initCenterView() {
    centerPanel = new QWidget(this);
    // 使用垂直布局作为外壳
    auto *mainLayout = new QVBoxLayout(centerPanel);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // [核心修改] 创建垂直分割器
    QSplitter *splitter = new QSplitter(Qt::Vertical, centerPanel);
    splitter->setHandleWidth(2); // 分割线细一点，精致一些
    splitter->setStyleSheet("QSplitter::handle { background-color: #3e3e3e; }");

    // ==========================================
    // 上半部分：3D 视图容器 (m_vtkContainer)
    // 准备一个纯净的、直接与操作系统对话的画板，交给后面的 3D 渲染引擎（VTK/PCL）
    // WA_OpaquePaintEvent: 告诉 Qt 这个 Widget 的绘制事件会完全覆盖之前的内容，不需要保留任何背景信息，这样可以避免不必要的重绘和闪烁
    // WA_PaintOnScreen: 告诉 Qt 不要使用双缓冲机制，不要干预这个 Widget 的绘制过程，直接在屏幕上绘制
    // WA_NativeWindow: 强制 Qt 为这个 Widget 向操作系统申请一个真实的、独立的窗口句柄
    // ==========================================
    m_vtkContainer = new QWidget();
    m_vtkContainer->setAttribute(Qt::WA_OpaquePaintEvent);
    m_vtkContainer->setAttribute(Qt::WA_PaintOnScreen);
    m_vtkContainer->setAttribute(Qt::WA_NativeWindow);

    // 将 VTK 容器加入分割器 (Index 0)
    splitter->addWidget(m_vtkContainer);

    // --- VTK 初始化逻辑保持不变 ---
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->SetParentId(reinterpret_cast<void*>(m_vtkContainer->winId()));  // 把 Qt 的窗口强行“嫁接”给 VTK

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renWin->AddRenderer(renderer);

    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renWin, "viewer", true));
    if (m_viewer->getRenderWindow()->GetInteractor()) {
        m_viewer->getRenderWindow()->GetInteractor()->Initialize();
    }
    m_viewer->setBackgroundColor(0.1, 0.1, 0.1);    // 深灰色背景
    m_viewer->addCoordinateSystem(100.0);           // 添加坐标轴
    m_viewer->initCameraParameters();               // 初始化相机视角

    // =========================================================
    // [修复] 接入 PCL 工业级原生拾取事件 (默认绑定 Shift + 左键)
    // =========================================================
    m_viewer->registerPointPickingCallback([this](const pcl::visualization::PointPickingEvent& event) {
        // 如果没有选中任何有效点，直接返回
        if (event.getPointIndex() == -1) return;
        
        // 只有在用户点击了"手动拾取"按钮进入模式后，才处理点击事件
        if (!m_isManualPickingMode) return;

        float x, y, z;
        event.getPoint(x, y, z);
        
        // 调用我们之前写好的状态推进与渲染函数
        this->onManualPointPicked(x, y, z);
    });

    // =========================================================
    // 注入 CloudCompare 风格鼠标交互
    // =========================================================
    vtkRenderWindowInteractor* iren = m_viewer->getRenderWindow()->GetInteractor();
    // PCL 默认使用的是 vtkInteractorStyleTrackballCamera 的子类
    vtkInteractorStyleTrackballCamera* style = 
        vtkInteractorStyleTrackballCamera::SafeDownCast(iren->GetInteractorStyle());

    if (style) {
        vtkSmartPointer<CloudCompareMouseCallback> ccCallback = vtkSmartPointer<CloudCompareMouseCallback>::New();
        ccCallback->style = style;
        

        // 绑定手动拾取的变量和 Lambda 回调
        ccCallback->isManualMode = &m_isManualPickingMode;
        ccCallback->onPointPicked = [this](double x, double y, double z) {
            this->onManualPointPicked(x, y, z);
        };

        // AddObserver 的第三个参数 1.0 表示“高优先级”
        // 确保我们的代码在 VTK 默认代码之前抢先执行并 Abort 掉默认事件
        iren->AddObserver(vtkCommand::RightButtonPressEvent, ccCallback, 1.0);
        iren->AddObserver(vtkCommand::RightButtonReleaseEvent, ccCallback, 1.0);
        iren->AddObserver(vtkCommand::MiddleButtonPressEvent, ccCallback, 1.0);
        iren->AddObserver(vtkCommand::MiddleButtonReleaseEvent, ccCallback, 1.0);
        iren->AddObserver(vtkCommand::LeftButtonDoubleClickEvent, ccCallback, 1.0);
    }
    // =========================================================


    // 解决“双事件循环”死锁 (定时器刷新)
    m_refreshTimer = new QTimer(this);
    connect(m_refreshTimer, &QTimer::timeout, [this](){
        if(m_viewer) m_viewer->spinOnce(1, true);
    });
    m_refreshTimer->start(30);
    // ----------------------------



    // ==========================================
    // 下半部分：控制台日志 (m_console)
    // ==========================================
    m_console = new QTextEdit();
    m_console->setReadOnly(true); // 只读，确保用户不能在里面乱打字破坏日志
    m_console->setPlaceholderText("系统准备就绪...");
    
    // 设置黑客风格/终端样式
    m_console->setStyleSheet(
        "QTextEdit {"
        "   background-color: #1e1e1e;" // 深灰背景
        "   color: #d4d4d4;"            // 浅灰文字
        "   font-family: Consolas, 'Courier New', monospace;"
        "   font-size: 12px;"
        "   border: none;"
        "   border-top: 1px solid #3e3e3e;"
        "   padding: 5px;"
        "}"
    );

    // 将控制台加入分割器 (Index 1)
    splitter->addWidget(m_console);

    // [关键] 设置分割比例：80% : 20%
    // setStretchFactor(index, stretch)
    splitter->setStretchFactor(0, 4); // 上面占 4 份
    splitter->setStretchFactor(1, 1); // 下面占 1 份

    // 将分割器加入主布局
    mainLayout->addWidget(splitter);
}


void SingleModePage::initRightPanel() {
    rightPanel = new QWidget(this);
    rightPanel->setFixedWidth(400);
    rightPanel->setStyleSheet("background: #ffffff; border-left: 1px solid #dcdfe6;");
    
    QVBoxLayout *mainLayout = new QVBoxLayout(rightPanel);
    mainLayout->setContentsMargins(0,0,0,0);

    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    
    QWidget *scrollContent = new QWidget();
    QVBoxLayout *scrollLayout = new QVBoxLayout(scrollContent);
    scrollLayout->setSpacing(0);
    scrollLayout->setContentsMargins(0,0,0,0);

    // =================================================
    // 1. 预处理与裁剪 (Preprocess)
    // =================================================
    auto *box1 = new CollapsibleBox("1. 预处理与裁剪 (Preprocess)");
    auto *lay1 = new QVBoxLayout();
    lay1->setSpacing(8);

    auto *rowTarget = new QHBoxLayout();
    rowTarget->addWidget(new QLabel("操作对象:"));
    QComboBox *comboTarget = new QComboBox();
    comboTarget->addItems({"原始点云 (Raw 5)", "融合点云 (Merged)"});
    comboTarget->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    rowTarget->addWidget(comboTarget);
    lay1->addLayout(rowTarget);

    // 虚线分割
    QFrame *line1 = new QFrame(); line1->setFrameShape(QFrame::HLine); line1->setStyleSheet("color: #eee; border-top: 1px dashed #eee;");
    lay1->addWidget(line1);

    auto addParam = [](QVBoxLayout* l, QString name, double val, QString unit = "") {
        auto *row = new QHBoxLayout();
        QLabel *lbl = new QLabel(name); lbl->setStyleSheet("color: #606266;");
        row->addWidget(lbl);
        QDoubleSpinBox *sb = new QDoubleSpinBox(); sb->setValue(val); sb->setButtonSymbols(QAbstractSpinBox::NoButtons);
        sb->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; border-radius: 3px; padding: 2px;");
        row->addWidget(sb);
        if(!unit.isEmpty()) row->addWidget(new QLabel(unit));
        l->addLayout(row);
    };

    auto *rowLeaf = new QHBoxLayout();
    QLabel *lblLeaf = new QLabel("下采样 (1-100):"); 
    lblLeaf->setStyleSheet("color: #606266;");
    rowLeaf->addWidget(lblLeaf);

    // 初始化成员变量 m_spinLeafSize
    m_spinLeafSize = new QDoubleSpinBox();
    m_spinLeafSize->setRange(1.0, 100.0); // 范围: 1mm ~ 100mm
    m_spinLeafSize->setValue(10.0);       // 默认: 10mm
    m_spinLeafSize->setSingleStep(1.0);   // 步长: 1mm
    m_spinLeafSize->setDecimals(1);       // 小数位: 1
    m_spinLeafSize->setButtonSymbols(QAbstractSpinBox::PlusMinus);
    m_spinLeafSize->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; border-radius: 3px; padding: 2px;");
    
    rowLeaf->addWidget(m_spinLeafSize);
    rowLeaf->addWidget(new QLabel("mm")); // 单位改为 mm
    lay1->addLayout(rowLeaf);

    // === 2. 统计滤波 (SOR) ===
    // 2.1 离群点阈值 (StdDev Mul)
    auto *rowStd = new QHBoxLayout();
    rowStd->addWidget(new QLabel("Std倍数 (0.1-10):")); // UI 修改：名称更准确
    
    m_spinStdDev = new QDoubleSpinBox();
    m_spinStdDev->setRange(0.1, 10.0); // 通常 1.0 - 3.0
    m_spinStdDev->setValue(2.0);       // 默认 1.0
    m_spinStdDev->setSingleStep(0.1);
    m_spinStdDev->setDecimals(1);
    m_spinStdDev->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowStd->addWidget(m_spinStdDev);
    lay1->addLayout(rowStd);

    // 2.2 邻近点数 (MeanK)
    auto *rowMeanK = new QHBoxLayout();
    rowMeanK->addWidget(new QLabel("邻近点数 (1-200):"));
    
    m_spinMeanK = new QSpinBox();
    m_spinMeanK->setRange(1, 200);
    m_spinMeanK->setValue(50);         // 默认 50
    m_spinMeanK->setSingleStep(10);
    m_spinMeanK->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowMeanK->addWidget(m_spinMeanK);
    lay1->addLayout(rowMeanK);

    // === 3. 半径裁剪 (Clip Radius) ===
    auto *rowClip = new QHBoxLayout();
    rowClip->addWidget(new QLabel("背景裁剪 (500-10000):"));
    
    m_spinClipRadius = new QDoubleSpinBox();
    m_spinClipRadius->setRange(500.0, 10000.0); // 0.5m - 10m
    m_spinClipRadius->setValue(2500.0);         // 默认 2.5m
    m_spinClipRadius->setSingleStep(100.0);
    m_spinClipRadius->setSuffix(" mm");
    m_spinClipRadius->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowClip->addWidget(m_spinClipRadius);
    lay1->addLayout(rowClip);

    // === 4. 信号连接 (所有参数改变都触发同一个处理函数) ===
    // 使用 QOverload 处理重载函数的指针
    connect(m_spinLeafSize, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SingleModePage::onRunPreprocess);
    connect(m_spinStdDev, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SingleModePage::onRunPreprocess);
    connect(m_spinMeanK, QOverload<int>::of(&QSpinBox::valueChanged), this, &SingleModePage::onRunPreprocess);
    connect(m_spinClipRadius, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, &SingleModePage::onRunPreprocess);

    auto *btnRow1 = new QHBoxLayout();
    QPushButton *btnPreview = new QPushButton("👁️ 预览效果");
    btnRow1->addWidget(btnPreview);
    // [修复 1] 连接预览按钮 -> 触发处理 (只更新视图)
    connect(btnPreview, &QPushButton::clicked, this, &SingleModePage::onRunPreprocess);

    QPushButton *btnExec1 = new QPushButton("🛠️ 执行处理");
    btnExec1->setObjectName("PrimaryBtn");
    btnRow1->addWidget(btnExec1);
    connect(btnExec1, &QPushButton::clicked, this, &SingleModePage::applyPreprocessToMemory);
    lay1->addLayout(btnRow1);

    box1->setContentLayout(lay1);
    scrollLayout->addWidget(box1);

    // =================================================
    // 2. 配准与融合 (Registration)
    // =================================================
    // 先调用初始化
    initDefaultMatrices();

    auto *box2 = new CollapsibleBox("2. 配准与融合 (Registration)");    // 自定义的 Qt 控件
    auto *lay2 = new QVBoxLayout();
    lay2->setSpacing(2);

    // 2.0 配准目标选择 (Reference Target)
    auto *rowRegTarget = new QHBoxLayout();
    QLabel *lblRef = new QLabel("参考目标 (Reference):");
    lblRef->setToolTip("选择源点云要配准到哪个坐标系\n默认是 Top，也可以选相邻相机");   // 在lblRef上添加提示信息
    rowRegTarget->addWidget(lblRef);

    m_comboRegTarget = new QComboBox();     // 配准目标的下拉选择框
    // 目标可以是任意一个相机
    m_comboRegTarget->addItems({"Top", "LB", "LT", "RB", "RT"}); 
    rowRegTarget->addWidget(m_comboRegTarget);
    lay2->addLayout(rowRegTarget);

    // 2.1 算法选择
    auto *rowAlgo = new QHBoxLayout();
    rowAlgo->addWidget(new QLabel("算法:"));
    m_comboRegMethod = new QComboBox();
    m_comboRegMethod->addItems({"手动矩阵 (Manual)", "ICP (P2Point)", "ICP (P2Plane)", "NDT 配准微调", "G-ICP"});
    rowAlgo->addWidget(m_comboRegMethod);
    lay2->addLayout(rowAlgo);


    // ==========================================
    // [新增] 动态参数面板：ICP 参数
    // ==========================================
    m_icpParamsWidget = new QWidget();
    auto *icpLay = new QGridLayout(m_icpParamsWidget);
    icpLay->setContentsMargins(15, 0, 0, 0); // 左侧缩进一点表示层级
    
    icpLay->addWidget(new QLabel("最大迭代次数:"), 0, 0);
    m_spinIcpIter = new QSpinBox(); 
    m_spinIcpIter->setRange(1, 500); 
    m_spinIcpIter->setValue(60);
    m_spinIcpIter->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    icpLay->addWidget(m_spinIcpIter, 0, 1);

    icpLay->addWidget(new QLabel("最大对应距离(mm):"), 1, 0);
    m_spinIcpDist = new QDoubleSpinBox(); 
    m_spinIcpDist->setRange(1.0, 500.0); 
    m_spinIcpDist->setValue(100.0);
    m_spinIcpDist->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    icpLay->addWidget(m_spinIcpDist, 1, 1);
    
    lay2->addWidget(m_icpParamsWidget);

    // ==========================================
    // [新增] 动态参数面板：NDT 参数
    // ==========================================
    m_ndtParamsWidget = new QWidget();
    auto *ndtLay = new QGridLayout(m_ndtParamsWidget);
    ndtLay->setContentsMargins(15, 0, 0, 0);
    
    ndtLay->addWidget(new QLabel("网格分辨率(mm):"), 0, 0);
    m_spinNdtRes = new QDoubleSpinBox(); 
    m_spinNdtRes->setRange(10.0, 500.0); 
    m_spinNdtRes->setValue(100.0);
    m_spinNdtRes->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    ndtLay->addWidget(m_spinNdtRes, 0, 1);

    ndtLay->addWidget(new QLabel("搜索步长:"), 1, 0);
    m_spinNdtStep = new QDoubleSpinBox(); 
    m_spinNdtStep->setRange(0.01, 5.0); 
    m_spinNdtStep->setValue(0.1); 
    m_spinNdtStep->setSingleStep(0.1);
    m_spinNdtStep->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    ndtLay->addWidget(m_spinNdtStep, 1, 1);

    ndtLay->addWidget(new QLabel("最大迭代次数:"), 2, 0);
    m_spinNdtIter = new QSpinBox(); 
    m_spinNdtIter->setRange(10, 200); 
    m_spinNdtIter->setValue(35);
    m_spinNdtIter->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    ndtLay->addWidget(m_spinNdtIter, 2, 1);

    lay2->addWidget(m_ndtParamsWidget);

    // ==========================================
    // [新增] 动态参数面板：G-ICP 参数
    // ==========================================
    m_gicpParamsWidget = new QWidget();
    auto *gicpLay = new QGridLayout(m_gicpParamsWidget);
    gicpLay->setContentsMargins(15, 0, 0, 0);

    gicpLay->addWidget(new QLabel("最大迭代次数:"), 0, 0);
    m_spinGicpIter = new QSpinBox(); 
    m_spinGicpIter->setRange(1, 500); 
    m_spinGicpIter->setValue(50);
    m_spinGicpIter->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    gicpLay->addWidget(m_spinGicpIter, 0, 1);

    gicpLay->addWidget(new QLabel("最大对应距离(mm):"), 1, 0);
    m_spinGicpDist = new QDoubleSpinBox(); 
    m_spinGicpDist->setRange(1.0, 500.0); 
    m_spinGicpDist->setValue(50.0); // 默认 5cm = 50mm
    m_spinGicpDist->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    gicpLay->addWidget(m_spinGicpDist, 1, 1);

    gicpLay->addWidget(new QLabel("收敛极小值(Epsilon):"), 2, 0);
    m_spinGicpEps = new QDoubleSpinBox(); 
    m_spinGicpEps->setRange(1e-9, 1e-1); 
    m_spinGicpEps->setDecimals(8);   // 允许输入 8 位小数
    m_spinGicpEps->setSingleStep(1e-8);
    m_spinGicpEps->setValue(1e-8);
    m_spinGicpEps->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    gicpLay->addWidget(m_spinGicpEps, 2, 1);

    lay2->addWidget(m_gicpParamsWidget);

    // ==========================================
    // [修改] 面板显示/隐藏逻辑联动
    // ==========================================
    m_icpParamsWidget->setVisible(false);
    m_ndtParamsWidget->setVisible(false);
    m_gicpParamsWidget->setVisible(false); // 默认隐藏

    connect(m_comboRegMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index){
        m_icpParamsWidget->setVisible(index == 1 || index == 2); 
        m_ndtParamsWidget->setVisible(index == 3);               
        m_gicpParamsWidget->setVisible(index == 4); // 选中 G-ICP 时显示
    });

    // 2.2 参与配准的源点云选择
    QLabel* lblSrc = new QLabel("选择待配准源 (Target=Top):");
    lblSrc->setStyleSheet("font-weight: bold; color: #409eff;");    // 蓝色加粗字体
    lay2->addWidget(lblSrc);

    auto *gridSrc = new QGridLayout();
    QStringList sources = {"LB", "LT", "RB", "RT"};
    int col = 0, row = 0;
    for(const auto& src : sources) {    // 遍历检查哪些点云参与配准
        QCheckBox* chk = new QCheckBox(src);
        chk->setChecked(true);      // 默认全选
        m_sourceChecks[src] = chk;  // 存入字典，方便后续读取状态
        gridSrc->addWidget(chk, row, col);  // 加入网格的指定行列
        col++; if(col > 1) { col=0; row++; }    // 核心逻辑：控制换行
    }
    lay2->addLayout(gridSrc);

    // 2.3 矩阵编辑器 (联动显示)
    QFrame *matrixFrame = new QFrame();
    matrixFrame->setStyleSheet("background: #f8f9fa; border: 1px solid #e4e7ed; border-radius: 4px; padding: 5px;");
    QVBoxLayout *matrixLay = new QVBoxLayout(matrixFrame);  // 创建矩阵编辑框
    matrixLay->setContentsMargins(2, 2, 2, 2); 
    matrixLay->setSpacing(5); // 减小 "编辑矩阵" 标签和 文本框 之间的距离
    
    auto *rowMatTarget = new QHBoxLayout();
    rowMatTarget->addWidget(new QLabel("编辑矩阵:"));
    m_comboMatrixView = new QComboBox();
    m_comboMatrixView->addItems(sources);   // LB, LT, RB, RT
    rowMatTarget->addWidget(m_comboMatrixView);

    // [新增] 重置默认矩阵按钮
    QPushButton *btnResetMat = new QPushButton("⟲ 重置");
    btnResetMat->setToolTip("放弃配准结果，恢复为最初始的物理标定矩阵");
    rowMatTarget->addWidget(btnResetMat);
    
    matrixLay->addLayout(rowMatTarget);

    // [新增] 绑定重置按钮逻辑
    connect(btnResetMat, &QPushButton::clicked, this, [this]() {
        initDefaultMatrices(); // 重新加载硬编码的物理标定值覆盖内存
        onMatrixTargetChanged(0); // 刷新界面文本框
        log("已放弃优化，恢复所有视角的矩阵为物理标定默认值。", "INFO");
    });

    m_textMatrix = new QTextEdit();
    m_textMatrix->setObjectName("MatrixEditor");
    m_textMatrix->setFixedHeight(160);
    // 初始显示 LB 的矩阵
    m_textMatrix->setText(matrixToString(m_transforms["LB"]));
    matrixLay->addWidget(m_textMatrix);

    lay2->addWidget(matrixFrame);

    // 连接信号：下拉框切换 -> 更新文本框
    connect(m_comboMatrixView, QOverload<int>::of(&QComboBox::currentIndexChanged), 
            this, &SingleModePage::onMatrixTargetChanged);
    
    // 连接信号：文本框修改 -> 更新内存中的矩阵
    connect(m_textMatrix, &QTextEdit::textChanged, this, &SingleModePage::onMatrixTextChanged);

    // 2.4 执行按钮
    m_btnRunReg = new QPushButton("🚀 执行配准与融合");
    m_btnRunReg->setObjectName("PrimaryBtn");
    lay2->addWidget(m_btnRunReg);

    connect(m_btnRunReg, &QPushButton::clicked, this, &SingleModePage::onExecuteRegistration);

    box2->setContentLayout(lay2);       // 设置 box2 的内容布局
    scrollLayout->addWidget(box2);      // 将 box2 加入右侧滚动区域

    // =================================================
    // 3. 主体精细提取 (Extraction)
    // =================================================
    auto *box3 = new CollapsibleBox("3. 主体精细提取 (Extraction)");
    auto *lay3 = new QVBoxLayout();
    lay3->setSpacing(8);
    
    // 1. 地面滤除阈值 (RANSAC)
    auto *rowRansac = new QHBoxLayout();
    rowRansac->addWidget(new QLabel("地面滤除厚度:"));
    m_spinRansacThresh = new QDoubleSpinBox();
    m_spinRansacThresh->setRange(1.0, 100.0);
    m_spinRansacThresh->setValue(20.0); // 默认将 15mm 厚度的底层视为地面
    m_spinRansacThresh->setSuffix(" mm");
    m_spinRansacThresh->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowRansac->addWidget(m_spinRansacThresh);
    lay3->addLayout(rowRansac);

    // 2. 聚类容差
    auto *rowTol = new QHBoxLayout();
    rowTol->addWidget(new QLabel("聚类容差 (Tol):"));
    m_spinExtractTol = new QDoubleSpinBox();
    m_spinExtractTol->setRange(5.0, 200.0); 
    m_spinExtractTol->setValue(40.0); // 默认 50mm，足够跨越猪身上的小缝隙
    m_spinExtractTol->setSuffix(" mm");
    m_spinExtractTol->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowTol->addWidget(m_spinExtractTol);
    lay3->addLayout(rowTol);
    
    // 3. 最小点数
    auto *rowMinSize = new QHBoxLayout();
    rowMinSize->addWidget(new QLabel("最小簇点数:"));
    m_spinExtractMinSize = new QSpinBox(); 
    m_spinExtractMinSize->setRange(1, 1000000); 
    m_spinExtractMinSize->setValue(5000); // 猪体很大，设为 5000 可以过滤掉设备架子等中型噪点
    m_spinExtractMinSize->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; padding: 2px;");
    rowMinSize->addWidget(m_spinExtractMinSize);
    lay3->addLayout(rowMinSize);

    // 提取按钮
    QPushButton *btnExtract = new QPushButton("🐷 提取平滑最大主体");
    btnExtract->setObjectName("PrimaryBtn");
    lay3->addWidget(btnExtract);
    connect(btnExtract, &QPushButton::clicked, this, &SingleModePage::onExtractBody);


    box3->setContentLayout(lay3);
    scrollLayout->addWidget(box3);

    // =================================================
    // 4. 关键点与测量 (Measure) - [补全 AI 和状态网格]
    // =================================================
    auto *box4 = new CollapsibleBox("4. 关键点与测量 (Measure)");
    auto *lay4 = new QVBoxLayout();
    lay4->setSpacing(10);

    // --- 切换开关 (Manual / AI) ---
    QFrame *toggleFrame = new QFrame();
    toggleFrame->setObjectName("ToggleContainer"); // QSS
    QHBoxLayout *toggleLay = new QHBoxLayout(toggleFrame);
    toggleLay->setContentsMargins(2,2,2,2);
    toggleLay->setSpacing(0);
    
    QButtonGroup *toggleGroup = new QButtonGroup(this);
    toggleGroup->setExclusive(true);

    QPushButton *btnManual = new QPushButton("👆 手动拾取");
    btnManual->setCheckable(true); btnManual->setObjectName("ToggleBtn");
    QPushButton *btnAI = new QPushButton("🤖 AI 自动检测");
    btnAI->setCheckable(true); btnAI->setChecked(true); btnAI->setObjectName("ToggleBtn");
    
    toggleGroup->addButton(btnManual);
    toggleGroup->addButton(btnAI);
    toggleLay->addWidget(btnManual);
    toggleLay->addWidget(btnAI);
    lay4->addWidget(toggleFrame);

    // --- AI 控制区 (双路推理引擎) ---
    QWidget *aiWidget = new QWidget();
    QVBoxLayout *aiLay = new QVBoxLayout(aiWidget);
    aiLay->setContentsMargins(0, 0, 0, 0);
    aiLay->setSpacing(8); // 增加一点间距，让两组按钮层次分明

    // ==========================================
    // 选项 1: 3D 推理 (C/S 架构)
    // ==========================================
    m_btnRunAI3D = new QPushButton("☁️ 运行 3D 点云推理 (服务端)");
    m_btnRunAI3D->setObjectName("WarningBtn"); 
    m_btnRunAI3D->setFixedHeight(32);
    aiLay->addWidget(m_btnRunAI3D);
    connect(m_btnRunAI3D, &QPushButton::clicked, this, &SingleModePage::onRunAIInference);

    // 虚线分割，区分两种 AI 模式
    QFrame *lineAI = new QFrame();
    lineAI->setFrameShape(QFrame::HLine);
    lineAI->setStyleSheet("color: #eee; border-top: 1px dashed #dcdfe6; margin: 2px 0;");
    aiLay->addWidget(lineAI);

    // ==========================================
    // 选项 2: 2D 推理 (本地 ONNX)
    // ==========================================
    auto *fileRow = new QHBoxLayout();
    m_leModelPath = new QLineEdit("F:/Gongzihang/2026/PointCloudProcessUI/build/Release/models/custom_pig_hrnet.onnx"); // 默认路径
    m_leModelPath->setReadOnly(true); 
    m_leModelPath->setStyleSheet("color: #666; background: #f4f4f5; border: none;");
    
    QPushButton *btnBrowseONNX = new QPushButton("📂"); 
    btnBrowseONNX->setFixedWidth(30);
    btnBrowseONNX->setStyleSheet("border: none; background: transparent;");
    
    fileRow->addWidget(m_leModelPath);
    fileRow->addWidget(btnBrowseONNX);
    
    // 带圆角边框的文件选择框
    QFrame *fileFrame = new QFrame();
    fileFrame->setStyleSheet("border: 1px solid #dcdfe6; border-radius: 4px; background: #f4f4f5;");
    fileFrame->setLayout(fileRow);
    fileRow->setContentsMargins(5, 0, 0, 0);
    aiLay->addWidget(fileFrame);

    m_btnRunAI2D = new QPushButton("🖼️ 运行 2D 图像推理 (本地)");
    m_btnRunAI2D->setObjectName("WarningBtn"); 
    m_btnRunAI2D->setFixedHeight(32);
    aiLay->addWidget(m_btnRunAI2D);
    connect(m_btnRunAI2D, &QPushButton::clicked, this, &SingleModePage::onRunAIInference2D);

    // 绑定 ONNX 模型文件浏览操作
    connect(btnBrowseONNX, &QPushButton::clicked, this, [this]() {
        QString path = QFileDialog::getOpenFileName(this, "选择 ONNX 模型", "", "ONNX Files (*.onnx);;All Files (*)");
        if (!path.isEmpty()) {
            m_leModelPath->setText(path);
        }
    });

    lay4->addWidget(aiWidget);

    // 连接切换信号：点击手动时隐藏 AI 控件，点击 AI 时显示
    connect(toggleGroup, &QButtonGroup::idClicked, [aiWidget](int){
        // 简单逻辑：如果 AI 被选中则显示，否则隐藏
        // 这里需要获取 sender 或者直接判断 checked 状态，简化起见：
        // 实际开发中建议单独写 slot
    });
    // 修正：直接连接 lambda
    connect(btnManual, &QPushButton::toggled, this, [this, aiWidget, btnManual](bool checked){
        aiWidget->setVisible(!checked); // 隐藏/显示 AI 面板
        
        if (checked) {
            // 尝试准备点云
            if (!prepareKeypointsCloud()) {
                // 准备失败，强行将手动拾取按钮恢复到未选中状态
                btnManual->blockSignals(true);
                btnManual->setChecked(false);
                btnManual->blockSignals(false);
                aiWidget->setVisible(true);
                return;
            }

            // 准备成功，切换显示图层
            for(auto* chk : m_layerChecks) { chk->setChecked(false); }
            if (m_layerChecks.contains("Keypoints")) {
                m_layerChecks["Keypoints"]->setChecked(true);
                onLayerToggle("Keypoints", true);
            }

            // ==== 进入手动模式 ====
            m_isManualPickingMode = true;
            log("进入手动拾取模式。请按住 [Shift + 鼠标左键] 在点云上拾取 [P1 耳中]", "ALGO");
            
            m_keypoints.clear();      // 清空旧数据
            m_currentPickIndex = 0;   // 重置索引
            
            // 清理 3D 视图上的旧球体
            for (int i = 0; i < 10; ++i) { 
                m_viewer->removeShape("kp_sphere_" + std::to_string(i));
                m_viewer->removeText3D("kp_text_" + std::to_string(i));
            }
            m_viewer->getRenderWindow()->Render();

            // 重置所有 Badge：第一个蓝色，其余灰色
            for (int i = 0; i < 6; ++i) {
                updateBadgeStyle(i, (i == 0) ? 1 : 0);
            }
        } else {
            // ==== 退出手动模式 (切换回 AI 模式) ====
            m_isManualPickingMode = false;
            
            // 1. 清空内部存储的点数据
            m_keypoints.clear();
            
            // 2. 清理 3D 视图上的旧球体和文字
            for (int i = 0; i < 10; ++i) { 
                m_viewer->removeShape("kp_sphere_" + std::to_string(i));
                m_viewer->removeText3D("kp_text_" + std::to_string(i));
            }
            m_viewer->getRenderWindow()->Render();

            // [修改] 3. 将所有 6 个 Badge 状态全部重置为灰色
            // 因为数据已经清空了，UI 应该恢复到未检测的初始状态
            for (int i = 0; i < 6; ++i) {
                updateBadgeStyle(i, 0); 
            }
            
            log("已退出手动拾取模式，切换回 AI 自动检测。旧数据已清空。", "INFO");
        }
    });

    // --- 关键点状态网格 ---
    QLabel *lblKpStatus = new QLabel("关键点状态 (Keypoints):");
    lblKpStatus->setStyleSheet("font-weight: bold; font-size: 11px; color: #606266; margin-top: 5px;");
    lay4->addWidget(lblKpStatus);

    QGridLayout *gridKp = new QGridLayout();
    gridKp->setSpacing(6);
    QStringList kps = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
    m_kpBadges.clear(); // 清空

    // 在 initRightPanel() 函数中，找到创建 QLabel 的循环
    for(int i=0; i<kps.size(); ++i) {
        QLabel *badge = new QLabel(kps[i]);
        badge->setAlignment(Qt::AlignCenter); // 文字居中对齐，更好看
        
        // 默认统一使用灰色“未激活”样式
        badge->setStyleSheet(
            "background-color: #f4f4f5; "
            "color: #909399; "
            "border: 1px solid #d3d4d6; "
            "border-radius: 4px; "
            "padding: 4px;"
        );
        
        m_kpBadges.append(badge); 
        gridKp->addWidget(badge, i/3, i%3);
    }
    lay4->addLayout(gridKp);

    // 计算体尺参数
    // =================================================
    // 在 btnCalc 创建之前，添加体尺算法参数
    // =================================================
    auto *measParamLay = new QGridLayout();
    measParamLay->setSpacing(5);
    
    measParamLay->addWidget(new QLabel("周长切片厚度(mm):"), 0, 0);
    m_spinGirthThick = new QDoubleSpinBox(); m_spinGirthThick->setRange(1.0, 50.0); m_spinGirthThick->setValue(10.0);
    measParamLay->addWidget(m_spinGirthThick, 0, 1);

    measParamLay->addWidget(new QLabel("骨架采样步长(mm):"), 1, 0);
    m_spinSkelStep = new QDoubleSpinBox(); m_spinSkelStep->setRange(5.0, 50.0); m_spinSkelStep->setValue(20.0);
    measParamLay->addWidget(m_spinSkelStep, 1, 1);

    measParamLay->addWidget(new QLabel("骨架搜索半径(mm):"), 2, 0);
    m_spinSkelRadius = new QDoubleSpinBox(); m_spinSkelRadius->setRange(10.0, 100.0); m_spinSkelRadius->setValue(30.0);
    measParamLay->addWidget(m_spinSkelRadius, 2, 1);

    measParamLay->addWidget(new QLabel("地面法向容差(度):"), 3, 0);
    m_spinHeightAngle = new QDoubleSpinBox(); m_spinHeightAngle->setRange(1.0, 45.0); m_spinHeightAngle->setValue(15.0);
    measParamLay->addWidget(m_spinHeightAngle, 3, 1);

    lay4->addLayout(measParamLay);

    QPushButton *btnCalc = new QPushButton("📏 计算体尺参数");
    btnCalc->setObjectName("PrimaryBtn");
    btnCalc->setMinimumHeight(45);
    btnCalc->setStyleSheet("margin-top: 5px; font-weight: bold; font-size: 14px;");
    lay4->addWidget(btnCalc);
    // 绑定槽函数
    connect(btnCalc, &QPushButton::clicked, this, &SingleModePage::onCalculateBodySize);
    box4->setContentLayout(lay4);
    scrollLayout->addWidget(box4);


    // =================================================
    // 5. 导出 (Export)
    // =================================================
    auto *box5 = new CollapsibleBox("5. 导出 (Export)");
    auto *lay5 = new QVBoxLayout();
    lay5->setSpacing(8);

    // [修改] 让 Lambda 返回 QPushButton* 指针
    auto addExportBtn = [](QVBoxLayout* l, QString text) -> QPushButton* {
        QPushButton* b = new QPushButton(text);
        b->setStyleSheet("text-align: left; padding-left: 15px;");
        l->addWidget(b);
        return b; // 返回指针
    };

    // 用三个变量接住创建好的按钮
    QPushButton* btnExportMerged = addExportBtn(lay5, "💾 另存融合点云 (.pcd)");
    QPushButton* btnExportBody   = addExportBtn(lay5, "💾 另存主体点云 (.pcd)");
    QPushButton* btnExportViz    = addExportBtn(lay5, "📸 导出测量可视化 (.pcd)");
    QPushButton* btnExportCSV    = addExportBtn(lay5, "📄 导出测量报告 (.txt)");

    // [新增] 绑定信号与槽函数
    connect(btnExportMerged, &QPushButton::clicked, this, &SingleModePage::onExportMergedCloud);
    connect(btnExportBody,   &QPushButton::clicked, this, &SingleModePage::onExportBodyCloud);
    connect(btnExportViz,    &QPushButton::clicked, this, &SingleModePage::onExportVizCloud); // [新增]
    connect(btnExportCSV,    &QPushButton::clicked, this, &SingleModePage::onExportReport);

    box5->setContentLayout(lay5);
    scrollLayout->addWidget(box5);
    // =================================================
    
    scrollLayout->addStretch();
    scroll->setWidget(scrollContent);
    mainLayout->addWidget(scroll);
}



void SingleModePage::onBrowseFile(const QString& key) {
    QString fileName = QFileDialog::getOpenFileName(
        this, 
        "选择点云或深度图文件 (" + key + ")", 
        "", 
        "Point Cloud Files (*.pcd *.ply *.raw);;All Files (*)"
    );

    if (!fileName.isEmpty()) {
        if (m_fileInputs.contains(key)) {
            QLineEdit* edit = m_fileInputs[key];
            QFileInfo fileInfo(fileName);

            // 1. 界面只显示文件名
            edit->setText(fileInfo.fileName());
            
            // 2. 将完整路径存储在自定义属性 "fullPath" 中
            edit->setProperty("fullPath", fileInfo.absoluteFilePath());
            
            // 3. 设置鼠标悬停提示，方便用户查看完整路径
            edit->setToolTip(fileInfo.absoluteFilePath());
            
            // [UI反馈] 光标移到最前，防止长文件名看不见开头
            edit->setCursorPosition(0); 

            // 立即加载并显示这个点云
            loadCloudToMemory(key, fileName);
        }
    }
}


void SingleModePage::onLoadFolder() {
    QString dirPath = QFileDialog::getExistingDirectory(this, "选择包含点云数据的文件夹", "");
    if (dirPath.isEmpty()) return;

    QDir dir(dirPath);
    QStringList filters; filters << "*.pcd" << "*.raw";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    QMap<QString, QString> keyMap;
    keyMap["005J"] = "Top";
    keyMap["00SE"] = "RB"; 
    keyMap["003W"] = "RT"; 
    keyMap["00YA"] = "LB";
    keyMap["00X6"] = "LT";

    // 1. 先清空
    onClearFiles();
    log("开始批量加载文件夹...", "INFO");

    int matchCount = 0;
    // 你的文件名后缀
    const QString targetSuffix = "_d_pc.pcd";

    for (const QFileInfo& fileInfo : fileList) {
        QString fileName = fileInfo.fileName();

        // 后缀检查
        if (!fileName.endsWith("_d_pc.pcd", Qt::CaseInsensitive) && 
                                    !fileName.endsWith("_depth_raw.raw", Qt::CaseInsensitive)) {
            continue; 
        }
        
        // 遍历 ID 匹配
        for (auto it = keyMap.begin(); it != keyMap.end(); ++it) {
            QString idCode = it.key();   
            QString targetSlot = it.value(); 

            if (fileName.contains(idCode, Qt::CaseInsensitive)) {
                if (m_fileInputs.contains(targetSlot)) {
                    QLineEdit* edit = m_fileInputs[targetSlot];

                    // --- UI 设置 ---
                    edit->setText(fileName);
                    edit->setProperty("fullPath", fileInfo.absoluteFilePath());
                    edit->setToolTip(fileInfo.absoluteFilePath());
                    edit->setCursorPosition(0);

                    // ==========================================
                    // [核心修复] 这里必须手动调用加载函数！
                    // 以前是靠 onLayerToggle 偷懒加载的，现在必须显式加载
                    // ==========================================
                    loadCloudToMemory(targetSlot, fileInfo.absoluteFilePath());

                    matchCount++;
                }
                break; 
            }
        }
    }

    if (matchCount > 0) {
        log(QString("批量加载完成，共加载 %1 个文件").arg(matchCount), "SUCCESS");
        // 自动复位相机，防止看不到
        if (m_viewer) m_viewer->resetCamera();
    } else {
        log("未匹配到有效文件", "WARN");
    }
}

// 清空功能实现
void SingleModePage::onClearFiles() {
    for (auto it = m_fileInputs.begin(); it != m_fileInputs.end(); ++it) {
        QLineEdit* edit = it.value();
        edit->clear();                    
        edit->setProperty("fullPath", ""); 
        edit->setToolTip("");             
    }
    // 清空内存数据和 3D 视图
    m_cloudData.clear();
    m_viewer->removeAllPointClouds();
    
    // =======================================================
    // [新增] 释放 OpenCV 图像矩阵内存
    // =======================================================
    m_topColorImage.release();
    m_topAlignedDepthImage.release();
    
    // 将所有复选框置为 false
    for(auto* chk : m_layerChecks) {
        chk->blockSignals(true);
        chk->setChecked(false);
        chk->blockSignals(false);
    }
}



// 辅助函数：后续算法调用时，不能直接 edit->text()，因为那只是文件名
// 需要调用这个函数来获取真实路径
QString SingleModePage::getFullPath(const QString& camKey) const {
    if (m_fileInputs.contains(camKey)) {
        // 优先读取存储的 "fullPath" 属性
        QString path = m_fileInputs[camKey]->property("fullPath").toString();
        
        // 如果属性为空（比如用户手动粘贴路径进框），则回退使用框内文字
        if (path.isEmpty()) {
            return m_fileInputs[camKey]->text();
        }
        return path;
    }
    return QString();
}

void SingleModePage::loadCloudToMemory(const QString& key, const QString& filePath) {
    if (filePath.isEmpty()) return;

    log(QString("正在加载文件: %1 ...").arg(filePath), "INFO");
    PointCloudT::Ptr cloud(new PointCloudT);

    // [核心修改] 拦截扩展名分支
    if (filePath.endsWith(".raw", Qt::CaseInsensitive)) {
        CameraIntrinsics intr = PointCloudAlgo::getCameraIntrinsics(key, SensorType::DEPTH, 512, 512);
        cloud = PointCloudAlgo::convertRawDepthToPointCloud(filePath, intr);

        // =======================================================
        // [新增] 如果是 Top 视角，顺便加载彩色图和对齐深度图
        // =======================================================
        if (key == "Top") {
            // 推导彩色图路径
            QString colorPath = filePath;
            colorPath.replace("_depth_raw.raw", "_rgb.png"); 
            
            // 加载彩色图
            m_topColorImage = cv::imread(colorPath.toStdString(), cv::IMREAD_COLOR);
            
            // 推导对齐深度图路径
            QString alignedDepthPath = filePath;
            alignedDepthPath.replace("_depth_raw.raw", "_depth_aligned.raw"); 

            // 加载 1280x720 的 16-bit 对齐深度图
            bool depthLoaded = false;
            QFile dFile(alignedDepthPath);
            if (dFile.open(QIODevice::ReadOnly)) {
                QByteArray dData = dFile.readAll();
                if (dData.size() == 1280 * 720 * 2) {
                    m_topAlignedDepthImage = cv::Mat(720, 1280, CV_16UC1, dData.data()).clone(); 
                    depthLoaded = true;
                }
                dFile.close();
            }

            // [核心修复] 极其严谨的状态校验日志
            if (!m_topColorImage.empty() && depthLoaded) {
                log("✅ 成功将 Top 视角 RGB 图像及对齐深度图装载入内存。", "SUCCESS");
            } else {
                if (m_topColorImage.empty()) {
                    log("❌ 警告：无法读取 Top 彩色图像！尝试路径: " + colorPath, "WARN");
                }
                if (!depthLoaded) {
                    log("❌ 警告：无法读取对齐深度图或分辨率不是 1280x720！尝试路径: " + alignedDepthPath, "WARN");
                }
            }
        }

        if (!cloud || cloud->empty()) {
            log("RAW 深度图转换失败！请检查文件完整性或确认内参分辨率(默认1280x720)。", "ERROR");
            QMessageBox::warning(this, "加载失败", "RAW 深度图解析失败，请检查内参分辨率。");
            return;
        }
        log(QString("RAW 深度图已成功转为点云 [%1]: 生成点数 %2").arg(key).arg(cloud->size()), "SUCCESS");
    } 
    else {
        // 原有 PCL 加载逻辑
        if (pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud) == -1) {
            log("无法读取 PCD 文件: " + filePath, "ERROR");
            QMessageBox::warning(this, "加载失败", "无法读取点云文件:\n" + filePath);
            return;
        }
        log(QString("加载 PCD 成功 [%1]: 点数 %2").arg(key).arg(cloud->size()), "SUCCESS");
    }

    // 存入内存
    m_cloudData[key] = cloud;
    
    if (m_layerChecks.contains(key)) {
        m_layerChecks[key]->setChecked(true); 
    }
}


void SingleModePage::onLayerToggle(const QString& layerId, bool checked) {
    if (!m_viewer) return;

    // [新增] 特殊图层独立处理逻辑
    if (layerId == "Measurements") {
        if (checked) {
            if (m_hasResults) drawMeasurements();
            else {
                log("暂无体尺测量数据，请先执行计算。", "WARN");
                m_layerChecks["Measurements"]->blockSignals(true);
                m_layerChecks["Measurements"]->setChecked(false);
                m_layerChecks["Measurements"]->blockSignals(false);
            }
        } else {
            clearMeasurements();
        }
        m_viewer->getRenderWindow()->Render();
        return; // 直接返回，不走下面的普通点云逻辑
    }

    std::string cloudId = layerId.toStdString();

    // 无论显示还是隐藏，先移除旧的，防止 ID 重复导致渲染异常
    m_viewer->removePointCloud(cloudId);

    if (checked) {
        // =========================================================
        // 分支 A: 融合点云 (Merged) - 使用 RGB 颜色
        // =========================================================
        if (layerId == "Merged" && m_mergedCloudRGB && !m_mergedCloudRGB->empty()) {
            
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(m_mergedCloudRGB);
            
            // [关键修复] 显式指定模板类型 <pcl::PointXYZRGB>
            m_viewer->addPointCloud<pcl::PointXYZRGB>(m_mergedCloudRGB, rgbHandler, cloudId);
            
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
        
        // =========================================================
        // 分支 B: 普通/原始点云 (Raw) - 使用固定单色
        // =========================================================
        else if (m_cloudData.contains(layerId) && !m_cloudData[layerId]->empty()) {
            
            // 1. 先计算该相机应该是什么颜色
            int r = 255, g = 255, b = 255;
            getCameraColor(layerId, r, g, b); // 获取颜色 (红/绿/蓝...)

            // 2. 创建自定义颜色处理器 (直接用这个颜色，不再依赖 setPointCloudRenderingProperties)
            pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(m_cloudData[layerId], r, g, b);
            
            // [关键修复] 显式指定模板类型 <PointT>
            // 这一步非常重要，混合了 RGB 和普通点云代码后，必须显式告诉编译器这是普通 PointT
            m_viewer->addPointCloud<PointT>(m_cloudData[layerId], colorHandler, cloudId);
            
            // 3. 只设置点大小
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
    }

    // 刷新渲染窗口
    m_viewer->getRenderWindow()->Render();
}

void SingleModePage::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    
    // 当 Qt 窗口大小改变时，手动通知 VTK 窗口也改变大小
    if (m_viewer && m_vtkContainer) {
        auto renWin = m_viewer->getRenderWindow();
        if (renWin) {
            // 获取容器的新大小
            QSize size = m_vtkContainer->size();
            // 乘以设备像素比 (DPR)，适配高分屏
            qreal dpr = m_vtkContainer->devicePixelRatio();
            renWin->SetSize(size.width() * dpr, size.height() * dpr);
            m_viewer->setWindowBorders(true); // 强制刷新布局
        }
    }
}


void SingleModePage::onRunPreprocess() {
    log("开始执行预处理预览...", "ALGO"); 
    int processedCount = 0;

    // 1. 获取所有参数
    float leaf_mm = m_spinLeafSize->value();
    double std_dev = m_spinStdDev->value();
    int mean_k = m_spinMeanK->value();
    float clip_radius_mm = m_spinClipRadius->value();

    // 2. 遍历所有可见点云
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        QString key = it.key();
        
        // 只处理勾选显示的层
        if (!m_layerChecks.contains(key) || !m_layerChecks[key]->isChecked()) {
            continue; 
        }

        PointCloudT::Ptr currentCloud = it.value(); 

        // --- Step 1: 下采样 ---
        currentCloud = PointCloudAlgo::downsample(currentCloud, leaf_mm);
        if (!currentCloud) continue;

        // --- Step 2: 统计滤波 (SOR) ---
        currentCloud = PointCloudAlgo::statisticalOutlierRemoval(currentCloud, mean_k, std_dev);
        if (!currentCloud) continue;

        // --- Step 3: 半径裁剪 ---
        currentCloud = PointCloudAlgo::distanceClip(currentCloud, clip_radius_mm);
        if (!currentCloud) continue;

        // 3. 更新可视化 (只有 currentCloud 有效才进来)
        if(currentCloud) {
            processedCount++;
            
            // [修复] 在这里声明 cloudId，并在该作用域内完成所有操作
            std::string cloudId = key.toStdString(); 

            // 日志
            log(QString("预处理 [%1]: 剩余点数 %2").arg(key).arg(currentCloud->size()), "INFO");

            // 移除旧的，添加新的
            m_viewer->removePointCloud(cloudId);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(currentCloud, 255, 255, 255);
            m_viewer->addPointCloud(currentCloud, colorHandler, cloudId);

            // [修复] 颜色恢复逻辑移入 if 块内部，这样 cloudId 是可见的
            double r=1.0, g=1.0, b=1.0;
            if (key == "Top") { r=1.0; g=0.0; b=0.0; }
            else if (key == "LB") { r=0.0; g=1.0; b=0.0; }
            else if (key == "LT") { r=0.0; g=0.0; b=1.0; }
            else if (key == "RB") { r=1.0; g=0.84; b=0.0; }
            else if (key == "RT") { r=0.0; g=1.0; b=1.0; }
            
            // 现在 cloudId 在作用域内，不会报错了
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudId);
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
    }

    // [优化] 将总结性日志移到循环外面，避免刷屏
    log(QString("预处理完成，更新了 %1 个视图").arg(processedCount), "SUCCESS");

    // 4. 刷新视图 (只需在最后刷新一次)
    if(m_viewer->getRenderWindow()) m_viewer->getRenderWindow()->Render();
}


void SingleModePage::applyPreprocessToMemory() {
    // 1. 获取当前参数
    float leaf = m_spinLeafSize->value();
    double std_dev = m_spinStdDev->value();
    int mean_k = m_spinMeanK->value();
    float radius = m_spinClipRadius->value();

    int count = 0;
    // 2. 遍历内存数据，永久更新它们
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        // 对所有数据（无论是否显示）都进行处理，保证一致性
        PointCloudT::Ptr raw = it.value();
        
        // 依次执行三个算法
        auto p1 = PointCloudAlgo::downsample(raw, leaf);
        if(!p1) continue;
        auto p2 = PointCloudAlgo::statisticalOutlierRemoval(p1, mean_k, std_dev);
        if(!p2) continue;
        auto p3 = PointCloudAlgo::distanceClip(p2, radius);
        if(!p3) continue;

        // [关键] 覆盖内存中的数据
        m_cloudData[it.key()] = p3;
        count++;
    }

    QMessageBox::information(this, "处理完成", 
        QString("已对 %1 个点云执行预处理并保存到内存。\n后续配准将使用新数据。").arg(count));
        
    // 刷新一下视图
    // 遍历检查左侧面板哪些图层被勾选了，直接将内存中现成的结果刷新到视图
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        QString key = it.key();
        if (m_layerChecks.contains(key) && m_layerChecks[key]->isChecked()) {
            // 利用你已经写好的 onLayerToggle 函数，强制重新加载该图层
            onLayerToggle(key, true); 
        }
    }
}

void SingleModePage::initDefaultMatrices() {
    // 根据你的 pc_register.cpp 中的数据硬编码默认值
    // LB -> Top
    Eigen::Matrix4d lb;
    lb << 
        -0.925691, -0.050326, -0.374918, 727.440735,
        -0.369875, -0.087362, 0.924965, -1535.817993,
        -0.079304, 0.994905, 0.062256, 1811.115601,
        0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["LB"] = lb;

    // LT -> Top
    Eigen::Matrix4d lt;
    lt << -0.893402, 0.129827, -0.430091, 715.208496,
-0.431722, -0.512967, 0.741944, -1244.757813,
-0.124298, 0.848534, 0.514334, 853.710327,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["LT"] = lt;

    // RB -> Top
    Eigen::Matrix4d rb;
    rb << 0.843684, 0.019195, 0.536497, -845.013184,
0.527601, -0.214250, -0.822030, 1084.744995,
0.099166, 0.976590, -0.190886, 2075.820557,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["RB"] = rb;

    // RT -> Top
    Eigen::Matrix4d rt;
    rt << 0.881307, -0.310379, 0.356319, -767.480042,
0.451075, 0.327860, -0.830084, 1185.616089,
0.140818, 0.892285, 0.428950, 1194.222656,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["RT"] = rt;
}


QString SingleModePage::matrixToString(const Eigen::Matrix4d& mat) {
    QString str;
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            // 保留4位小数，右对齐
            str += QString::number(mat(i,j), 'f', 6); 
            if(j < 3) str += "\t"; // 列间用制表符分隔
        }
        if(i < 3) str += "\n";
    }
    return str;
}

Eigen::Matrix4d SingleModePage::stringToMatrix(const QString& text) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    QStringList tokens = text.split(QRegularExpression("[\\s,]+"), Qt::SkipEmptyParts);
    if(tokens.size() == 16) {
        int idx = 0;
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                mat(i,j) = tokens[idx++].toFloat();
            }
        }
    }
    else {
        // 打印警告，如果解析失败，我们在后续逻辑中拦截
        qDebug() << "[警告] 矩阵解析失败！提取到的数字个数不是16个，而是:" << tokens.size();
    }
    return mat;
}

void SingleModePage::onMatrixTargetChanged(int index) {
    QString key = m_comboMatrixView->currentText();
    // 临时断开文本改变信号，防止死循环
    m_textMatrix->blockSignals(true);
    m_textMatrix->setText(matrixToString(m_transforms[key]));
    m_textMatrix->blockSignals(false);
}

void SingleModePage::onMatrixTextChanged() {
    QString key = m_comboMatrixView->currentText();
    m_transforms[key] = stringToMatrix(m_textMatrix->toPlainText());
}

void SingleModePage::onExecuteRegistration() {
    // 1. 同步编辑器矩阵并校验
    QString currentEditingKey = m_comboMatrixView->currentText();
    QString matrixText = m_textMatrix->toPlainText();
    QStringList tokens = matrixText.split(QRegularExpression("[\\s,]+"), Qt::SkipEmptyParts);
    if (tokens.size() == 16) {
        m_transforms[currentEditingKey] = stringToMatrix(matrixText);
    } else {
        QMessageBox::warning(this, "矩阵格式错误", "当前编辑框中的矩阵数字不是16个，请检查！");
        return;
    }

    QString targetKey = m_comboRegTarget->currentText();
    if (!m_cloudData.contains(targetKey)) {
        QMessageBox::warning(this, "错误", "未找到参考目标点云: " + targetKey);
        return;
    }
    PointCloudT::Ptr cloudTarget = m_cloudData[targetKey];
    Eigen::Matrix4d matTargetToTop = Eigen::Matrix4d::Identity();
    if (targetKey != "Top") matTargetToTop = m_transforms[targetKey];

    // ==========================================================
    // 2. 准备并行任务列表
    // ==========================================================
    QList<RegTaskInput> tasks;
    int methodIndex = m_comboRegMethod->currentIndex();

    for (auto it = m_sourceChecks.begin(); it != m_sourceChecks.end(); ++it) {
        QString srcKey = it.key(); 
        if (!it.value()->isChecked() || !m_cloudData.contains(srcKey) || srcKey == targetKey) continue; 

        RegTaskInput task;
        task.srcKey = srcKey;
        task.targetKey = targetKey;
        task.cloudSrc = m_cloudData[srcKey];
        task.cloudTarget = cloudTarget;
        task.initialGuess = matTargetToTop.inverse() * m_transforms[srcKey];
        task.methodIndex = methodIndex;
        task.algoType = (methodIndex == 2) ? PointCloudAlgo::P2Plane : PointCloudAlgo::P2Point;
        
        // UI 参数抓取
        // ICP 参数
        task.icpIter = m_spinIcpIter->value();
        task.icpDist = m_spinIcpDist->value();
        // NDT 参数
        task.ndtRes = m_spinNdtRes->value();
        task.ndtStep = m_spinNdtStep->value();
        task.ndtIter = m_spinNdtIter->value();
        // GICP 参数
        task.gicpIter = m_spinGicpIter->value();
        task.gicpDist = m_spinGicpDist->value();
        task.gicpEps = m_spinGicpEps->value();
        tasks.append(task);
    }

    if (tasks.isEmpty() && targetKey != "Top") {
        log("警告：没有需要配准的源点云。", "WARN");
        return;
    }

    // ==========================================================
    // 3. 挂起 UI 状态，启动并发线程池 (非阻塞)
    // ==========================================================
    log("🚀 启动多线程并行配准流水线，操作已挂起后台运行...", "ALGO");
    m_btnRunReg->setEnabled(false);
    m_btnRunReg->setText("⏳ 正在后台并行配准中...");

    QFutureWatcher<RegTaskOutput> *watcher = new QFutureWatcher<RegTaskOutput>(this);

    // ==========================================================
    // 4. 当所有后台线程完成时，触发回调更新 UI
    // ==========================================================
    connect(watcher, &QFutureWatcher<RegTaskOutput>::finished, this, [this, watcher, targetKey, cloudTarget, matTargetToTop]() {
        
        // 拿回所有线程的计算结果
        QList<RegTaskOutput> results = watcher->future().results();
        
        // 初始化存储容器
        PointCloudT::Ptr geometryMerged(new PointCloudT);
        if (!m_mergedCloudRGB) { m_mergedCloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>); }
        m_mergedCloudRGB->clear();

        // 统一合并颜色的 Lambda
        auto appendColoredCloud = [&](PointCloudT::Ptr inputCloud, const QString& camName) {
            if (!inputCloud) return;
            int r, g, b; getCameraColor(camName, r, g, b);
            for (const auto& pt : inputCloud->points) {
                geometryMerged->points.push_back(pt);
                
                // [修复] 先使用默认构造函数，再分别赋值
                pcl::PointXYZRGB ptRGB;
                ptRGB.x = pt.x; ptRGB.y = pt.y; ptRGB.z = pt.z;
                ptRGB.r = static_cast<uint8_t>(r);
                ptRGB.g = static_cast<uint8_t>(g);
                ptRGB.b = static_cast<uint8_t>(b);
                
                m_mergedCloudRGB->points.push_back(ptRGB);
            }
        };


        // 1. 先加入固定的目标点云
        appendColoredCloud(cloudTarget, targetKey);

        // 2. 将各个子线程算好的点云累加进来
        for(const auto& res : results) {
            if(!res.valid) continue;
            
            // 打印该线程在后台产生的所有日志
            for(const auto& l : res.logs) { log(l.first, l.second); }
            
            log(QString("完成 [%1 -> %2] 的配准与拼装。").arg(res.srcKey).arg(targetKey), "ALGO");

            // 更新内存矩阵
            m_transforms[res.srcKey] = matTargetToTop * res.finalTransform;
            
            // 拼装点云
            if (res.cloudAlignedLocal) appendColoredCloud(res.cloudAlignedLocal, res.srcKey);
        }

        // 3. 收尾工作
        geometryMerged->width = geometryMerged->size(); geometryMerged->height = 1; geometryMerged->is_dense = true;
        m_cloudData["Merged"] = geometryMerged; 
        m_mergedCloudRGB->width = m_mergedCloudRGB->size(); m_mergedCloudRGB->height = 1; m_mergedCloudRGB->is_dense = true;

        if (m_layerChecks.contains("Merged")) {
            m_layerChecks["Merged"]->setChecked(true);
            onLayerToggle("Merged", true); 
        }
        
        if (m_transforms.contains(m_comboMatrixView->currentText())) {
            onMatrixTargetChanged(0); 
        }

        log(QString("✅ 所有线程处理完毕。融合点云总点数: %1").arg(m_mergedCloudRGB->size()), "SUCCESS");

        // 恢复 UI 按钮状态
        m_btnRunReg->setEnabled(true);
        m_btnRunReg->setText("🚀 执行配准与融合");
        
        // 清理内存
        watcher->deleteLater();
    });

    // 🔥 发射任务给 Qt 的全局并发线程池！
    // QtConcurrent::mapped 会根据 CPU 核心数自动切分任务
    QFuture<RegTaskOutput> future = QtConcurrent::mapped(tasks, processRegistrationWorker);
    watcher->setFuture(future);
}


void SingleModePage::log(const QString& msg, const QString& type) {
    if (!m_console) return;

    // 获取当前时间
    QString timeStr = QDateTime::currentDateTime().toString("[HH:mm:ss]");

    // 设置颜色 HTML
    QString color = "#d4d4d4"; // 默认白色
    if (type == "WARN") color = "#e5c07b"; // 黄色
    else if (type == "ERROR") color = "#e06c75"; // 红色
    else if (type == "SUCCESS") color = "#98c379"; // 绿色
    else if (type == "ALGO") color = "#61afef"; // 蓝色

    // 组装 HTML 文本
    QString html = QString("<span style='color:#5c6370;'>%1</span> " // 时间灰色
                           "<span style='color:%2; font-weight:bold;'>[%3]</span> " // 类型带色
                           "<span style='color:#d4d4d4;'>%4</span>") // 内容
                           .arg(timeStr).arg(color).arg(type).arg(msg);

    m_console->append(html);

    // 自动滚动到底部
    QScrollBar *sb = m_console->verticalScrollBar();
    sb->setValue(sb->maximum());
}


void SingleModePage::getCameraColor(const QString& camName, int& r, int& g, int& b) {
    if      (camName == "Top") { r = 255; g = 0;   b = 0;   } // 红
    else if (camName == "LB")  { r = 0;   g = 255; b = 0;   } // 绿
    else if (camName == "LT")  { r = 0;   g = 0;   b = 255; } // 蓝
    else if (camName == "RB")  { r = 255; g = 215; b = 0;   } // 金 (Right-Bottom)
    else if (camName == "RT")  { r = 0;   g = 255; b = 255; } // 青 (Right-Top)
    else                       { r = 255; g = 255; b = 255; } // 默认白
}

// 实现主体精细提取的槽函数
void SingleModePage::onExtractBody() {
    // 1. 检查是否存在融合后的点云数据
    if (!m_cloudData.contains("Merged") || m_cloudData["Merged"]->empty()) {
        QMessageBox::warning(this, "警告", "没有找到融合后的点云！请先执行配准与融合。");
        return;
    }

    // 2. 读取界面参数
    double plane_thresh = m_spinRansacThresh->value();  // 平面检测阈值
    double tol = m_spinExtractTol->value();             // 聚类提取的距离容差
    int min_size = m_spinExtractMinSize->value();       // 聚类提取的最小点数

    // 包装日志回调
    auto logBridge = [this](const QString& msg, const QString& type) {
        this->log(msg, type); 
    };

    // 3. 执行算法，提取主体
    PointCloudT::Ptr bodyCloud = PointCloudAlgo::extractLargestCluster(
        m_cloudData["Merged"], tol, min_size, plane_thresh, logBridge
    );

    // 4. 更新内存与 3D 视图
    if (bodyCloud) {
        // 保存到内存字典
        m_cloudData["Body"] = bodyCloud;

        // 【视觉优化】: 关闭“融合点云”图层，打开“提取主体”图层
        // 这样画面中杂乱的背景会瞬间消失，只剩下粉色的干干净净的猪体
        if (m_layerChecks.contains("Merged")) {
            m_layerChecks["Merged"]->setChecked(false); // 触发 onLayerToggle 隐藏
        }
        
        if (m_layerChecks.contains("Body")) {
            m_layerChecks["Body"]->setChecked(true); // 触发 onLayerToggle 显示粉色
            
            // 手动调用一次确保视图刷新
            onLayerToggle("Body", true);
        }
    }
}


// 在 PCL 视图中绘制关键点的辅助函数
void SingleModePage::drawKeypointsInViewer(const std::vector<Eigen::Vector3f>& kps) {
    if (!m_viewer) return;

    QStringList kpNames = {"P1", "P2", "P3", "P4", "P5", "P6"};

    // 每次绘制前，先清除上一轮画的关键点形状
    for (int i = 0; i < 10; ++i) { 
        m_viewer->removeShape("kp_sphere_" + std::to_string(i));
        m_viewer->removeText3D("kp_text_" + std::to_string(i));
    }

    for (size_t i = 0; i < kps.size() && i < kpNames.size(); ++i) {
        pcl::PointXYZ pt(kps[i].x(), kps[i].y(), kps[i].z());
        
        std::string sphereId = "kp_sphere_" + std::to_string(i);
        std::string textId = "kp_text_" + std::to_string(i);

        // 1. 添加红色的 3D 球体 (半径 15mm，视你的猪体大小而定)
        m_viewer->addSphere(pt, 15.0, 1.0, 0.0, 0.0, sphereId);
        
        // 2. 在球体旁边稍微偏上一点的地方添加文字标签
        pcl::PointXYZ textPt(pt.x, pt.y, pt.z + 20.0); 
        m_viewer->addText3D(kpNames[i].toStdString(), textPt, 15.0, 1.0, 1.0, 1.0, textId);
    }

    m_viewer->getRenderWindow()->Render();
}



/*
 * 作用：执行 AI 模型推理 (客户端侧逻辑)
 * 功能：提取背部点云 -> 计算法线与曲率特征 -> 序列化为 N*7 的二进制流 -> 通过 HTTP POST 发送给 WSL2 中的 Python 服务 -> 异步接收解析关键点坐标 -> 渲染到 3D 视图。
 * 实现机制：使用 PCL 的 NormalEstimationOMP 计算特征；使用 reinterpret_cast 强转指针实现零拷贝级别的二进制打包；使用 QNetworkAccessManager 实现非阻塞异步通信。
 */
void SingleModePage::onRunAIInference() {
    // [修改] 用封装好的函数代替之前长长的提取代码
    if (!prepareKeypointsCloud()) {
        return; // 如果准备失败（比如没有 Top 视角），直接中止
    }
    // 从内存中把刚准备好的背部点云取出来，赋给 backCloud 变量
    PointCloudT::Ptr backCloud = m_cloudData["Keypoints"];

    // 切换图层显示，专注显示关键点云
    for(auto* chk : m_layerChecks) { chk->setChecked(false); }
    if (m_layerChecks.contains("Keypoints")) {
        m_layerChecks["Keypoints"]->setChecked(true);
        onLayerToggle("Keypoints", true);
    }

    // ==========================================================
    // 3. 计算法向量与曲率特征 (N * 4) -> 组合成 N * 7
    // ==========================================================
    log("正在计算点云法线与曲率特征...", "ALGO");
    pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(backCloud);
    ne.setRadiusSearch(50.0); // 根据实际猪体尺寸调整搜索半径

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*cloud_normals);

    // ==========================================================
    // 4. 将数据打包为紧凑的二进制字节流 (QByteArray)
    // 格式: [x1, y1, z1, nx1, ny1, nz1, cur1, x2, y2, z2, nx2...]
    // ==========================================================
    int numPoints = backCloud->size();
    QByteArray postData;
    postData.resize(numPoints * 7 * sizeof(float)); // 预分配内存，7个float = 28字节/点
    
    // 使用指针直接写入内存，速度极快
    float* ptr = reinterpret_cast<float*>(postData.data());
    for (int i = 0; i < numPoints; ++i) {
        *ptr++ = backCloud->points[i].x;
        *ptr++ = backCloud->points[i].y;
        *ptr++ = backCloud->points[i].z;
        *ptr++ = cloud_normals->points[i].normal_x;
        *ptr++ = cloud_normals->points[i].normal_y;
        *ptr++ = cloud_normals->points[i].normal_z;
        *ptr++ = cloud_normals->points[i].curvature;
    }

    // ==========================================================
    // 5. 构建并发送 HTTP POST 请求
    // ==========================================================
    log("正在发送数据至 AI 服务器(WSL2)执行推理...", "INFO");
    
    // 使用 QHttpMultiPart 构建表单数据，匹配 FastAPI 的 UploadFile
    QHttpMultiPart *multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);
    QHttpPart filePart;
    filePart.setHeader(QNetworkRequest::ContentDispositionHeader, 
                       QVariant("form-data; name=\"file\"; filename=\"pointcloud.bin\""));
    filePart.setBody(postData);
    multiPart->append(filePart);

    QNetworkRequest request(QUrl("http://127.0.0.1:8000/predict"));
    
    // 发送请求
    QNetworkReply *reply = m_networkManager->post(request, multiPart);
    multiPart->setParent(reply); // 随 reply 一起自动销毁，防止内存泄漏

    // [可选] 禁用按钮防止重复点击
    m_btnRunAI3D->setEnabled(false); m_btnRunAI3D->setText("3D 推理中...");

    // ==========================================================
    // 6. 异步处理返回结果 (C++11 Lambda 回调)
    // ==========================================================
    connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        if (reply->error() == QNetworkReply::NoError) {
            // 读取返回的二进制数据
            QByteArray responseData = reply->readAll();
            int numKeypoints = responseData.size() / (3 * sizeof(float));
            
            if (numKeypoints > 0) {
                const float* outPtr = reinterpret_cast<const float*>(responseData.constData());
                
                // ---------------------------------------------------------
                // [核心修改 1]：不再使用局部变量，而是直接操作成员变量 m_keypoints
                // 每次 AI 重新推理成功时，必须先清空旧数据（包括之前手动点的或上次AI算的）
                // ---------------------------------------------------------
                this->m_keypoints.clear();
                
                // 遍历解析坐标并存入类成员变量
                for (int i = 0; i < numKeypoints; ++i) {
                    float kx = outPtr[i*3 + 0];
                    float ky = outPtr[i*3 + 1];
                    float kz = outPtr[i*3 + 2];
                    this->m_keypoints.push_back(Eigen::Vector3f(kx, ky, kz));
                }

                log(QString("AI 推理成功！检测到 %1 个关键点。").arg(numKeypoints), "SUCCESS");
                
                // ---------------------------------------------------------
                // 1. 在控制台输出所有关键点的绝对三维坐标
                // ---------------------------------------------------------
                QStringList kpNames = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
                for (int i = 0; i < numKeypoints && i < kpNames.size(); ++i) {
                    // [核心修改 2]：从 this->m_keypoints 中读取坐标进行打印
                    QString coordMsg = QString("  ▶ %1: (X: %2, Y: %3, Z: %4) mm")
                                        .arg(kpNames[i])
                                        .arg(this->m_keypoints[i].x(), 0, 'f', 2)
                                        .arg(this->m_keypoints[i].y(), 0, 'f', 2)
                                        .arg(this->m_keypoints[i].z(), 0, 'f', 2);
                    log(coordMsg, "INFO");
                }

                // 2. 在 3D 视图中渲染
                // [核心修改 3]：将成员变量传给渲染函数
                this->drawKeypointsInViewer(this->m_keypoints);

                // 3. 更新界面 UI 网格 (让 Badge 变绿)
                for (int i = 0; i < m_kpBadges.size(); ++i) {
                    if (i < numKeypoints) {
                        m_kpBadges[i]->setStyleSheet(
                            "background-color: #f0f9eb; color: #67c23a; "
                            "border: 1px solid #c2e7b0; border-radius: 4px; "
                            "padding: 4px; font-weight: bold;"
                        );
                    } else {
                        m_kpBadges[i]->setStyleSheet(
                            "background-color: #f4f4f5; color: #909399; "
                            "border: 1px solid #d3d4d6; border-radius: 4px; padding: 4px;"
                        );
                    }
                }
            } else {
                log("服务端返回的数据为空或不合法！", "ERROR");
            }
        } else {
            // 网络通信错误处理
            log("AI 推理失败: " + reply->errorString(), "ERROR");
        }
        
        reply->deleteLater(); // 释放内存
        
        // 恢复 AI 按钮状态（如果你采用了成员变量 m_btnRunAI）
        if (m_btnRunAI3D) {
            m_btnRunAI3D->setEnabled(true); 
            m_btnRunAI3D->setText("☁️ 运行 3D 点云推理 (服务端)");
        }
    });
}

// 辅助函数：统一管理 Badge 的三种颜色状态
void SingleModePage::updateBadgeStyle(int index, int state) {
    if (index < 0 || index >= m_kpBadges.size()) return;
    
    QLabel* badge = m_kpBadges[index];
    if (state == 0) { // 灰色 (未完成)
        badge->setStyleSheet("background-color: #f4f4f5; color: #909399; border: 1px solid #d3d4d6; border-radius: 4px; padding: 4px;");
    } else if (state == 1) { // 蓝色 (正在等待拾取该点)
        badge->setStyleSheet("background-color: #ecf5ff; color: #409eff; border: 1px solid #b3d8ff; border-radius: 4px; padding: 4px; font-weight: bold;");
    } else if (state == 2) { // 绿色 (已完成拾取/预测)
        badge->setStyleSheet("background-color: #f0f9eb; color: #67c23a; border: 1px solid #c2e7b0; border-radius: 4px; padding: 4px; font-weight: bold;");
    }
}

// 核心逻辑：当用户 Shift+左键 成功点到一个点时触发
void SingleModePage::onManualPointPicked(double x, double y, double z) {
    if (!m_isManualPickingMode || m_currentPickIndex >= 6) return;

    QStringList kpNames = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
    
    // 1. 保存坐标
    m_keypoints.push_back(Eigen::Vector3f(x, y, z));

    // 2. 打印控制台日志
    QString coordMsg = QString("  ▶ 手动拾取 %1: (X: %2, Y: %3, Z: %4) mm")
                        .arg(kpNames[m_currentPickIndex])
                        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2);
    log(coordMsg, "SUCCESS");

    // 3. 将当前 Badge 设为绿色
    updateBadgeStyle(m_currentPickIndex, 2);

    // 4. 在 3D 视图中渲染（复用你之前的函数）
    drawKeypointsInViewer(m_keypoints);

    // 5. 游标前进，高亮下一个 Badge 为蓝色
    m_currentPickIndex++;
    if (m_currentPickIndex < 6) {
        updateBadgeStyle(m_currentPickIndex, 1);
        log(QString("请按住 Shift+左键 拾取下一个点: [%1]").arg(kpNames[m_currentPickIndex]), "INFO");
    } else {
        log("🎉 6 个关键点已全部手动拾取完毕！", "SUCCESS");
        m_isManualPickingMode = false; // 自动退出拾取模式
        // 这里你也可以触发后续的“计算体尺参数”逻辑
    }
}

// 检查并生成关键点检测专用的点云
bool SingleModePage::prepareKeypointsCloud() {
    // 1. 如果已经存在，说明之前生成过，直接返回成功
    if (m_cloudData.contains("Keypoints") && !m_cloudData["Keypoints"]->empty()) {
        return true; 
    }

    // 2. 如果不存在，检查前置依赖 (Top 点云)
    if (!m_cloudData.contains("Top") || m_cloudData["Top"]->empty()) {
        QMessageBox::warning(this, "错误", "缺少 Top 相机点云，无法提取背部特征！");
        return false;
    }

    log("开始准备关键点检测云 (提取背部点云)...", "ALGO");
    
    // 参数设置
    double plane_thresh = 15.0; 
    double tol = 50.0;          
    int min_size = 1000;        

    auto logBridge = [this](const QString& msg, const QString& type) { this->log(msg, type); };
    
    // 调用算法提取
    PointCloudT::Ptr backCloud = PointCloudAlgo::extractLargestCluster(
        m_cloudData["Top"], tol, min_size, plane_thresh, logBridge
    );

    if (!backCloud) {
        log("背部点云提取失败，无法进行标注！", "ERROR");
        return false;
    }

    // 存入内存
    m_cloudData["Keypoints"] = backCloud;
    return true;
}


// ---------------------------------------------------------
// [新增] 清理测量结果图层
// ---------------------------------------------------------
void SingleModePage::clearMeasurements() {
    if (!m_viewer) return;
    m_viewer->removePointCloud("measure_body_cloud");
    m_viewer->removePointCloud("skel_pts");
    m_viewer->removeShape("ground_plane");
    m_viewer->removeShape("meas_height");
    m_viewer->removeShape("meas_width");
    
    // 粗略遍历移除所有动态生成的线段 (PCL的 removeShape 效率很高，不怕多循环)
    for (int i = 0; i < 300; ++i) {
        m_viewer->removeShape("skel_line_" + std::to_string(i));
        m_viewer->removeShape("chest_" + std::to_string(i));
        m_viewer->removeShape("waist_" + std::to_string(i));
        m_viewer->removeShape("hip_" + std::to_string(i));
    }
    m_viewer->removeShape("chest_close");
    m_viewer->removeShape("waist_close");
    m_viewer->removeShape("hip_close");
}

// ---------------------------------------------------------
// [新增] 绘制测量结果图层
// ---------------------------------------------------------
void SingleModePage::drawMeasurements() {
    if (!m_viewer || !m_hasResults) return;
    
    clearMeasurements(); // 绘制前先清理旧的

    BodySizeResults& res = m_latestResults;

    // A. 半透明主体
    pcl::visualization::PointCloudColorHandlerCustom<PointT> body_color(res.aligned_cloud, 200, 200, 200);
    m_viewer->addPointCloud<PointT>(res.aligned_cloud, body_color, "measure_body_cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "measure_body_cloud"); 

    // B. 对齐后的关键点
    std::vector<Eigen::Vector3f> eigen_kps;
    for (const auto& pt : res.aligned_keypoints) eigen_kps.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
    drawKeypointsInViewer(eigen_kps);

    // C. 绿色骨架
    if (res.skeleton_cloud && res.skeleton_cloud->size() > 1) {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> skel_color(res.skeleton_cloud, 0, 255, 0);
        m_viewer->addPointCloud<PointT>(res.skeleton_cloud, skel_color, "skel_pts");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "skel_pts");
        for (size_t i = 0; i < res.skeleton_cloud->size() - 1; ++i) {
            m_viewer->addLine<PointT>(res.skeleton_cloud->points[i], res.skeleton_cloud->points[i+1], 0, 1.0, 0, "skel_line_" + std::to_string(i));
            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "skel_line_" + std::to_string(i));
        }
    }

    // D. 地面与高宽线
    if (res.ground_polygon && res.ground_polygon->size() == 4) {
        m_viewer->addPolygon<PointT>(res.ground_polygon, 0.0, 1.0, 0.0, "ground_plane");
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "ground_plane");
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "ground_plane"); 
    }
    m_viewer->addLine<PointT>(res.height_top, res.height_bottom, 0.0, 0.0, 1.0, "meas_height");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "meas_height");
    m_viewer->addLine<PointT>(res.width_p1, res.width_p2, 1.0, 1.0, 0.0, "meas_width");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "meas_width");

    // E. 轮廓多边形
    auto drawContour = [this](PointCloudT::Ptr contour, std::string id, double r, double g, double b) {
        if (!contour || contour->size() < 3) return;
        for (size_t i = 0; i < contour->size() - 1; ++i) {
            this->m_viewer->addLine(contour->points[i], contour->points[i + 1], r, g, b, id + "_" + std::to_string(i));
            this->m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id + "_" + std::to_string(i));
        }
        this->m_viewer->addLine(contour->points.back(), contour->points.front(), r, g, b, id + "_close");
        this->m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id + "_close");
    };
    drawContour(res.chest_contour, "chest", 1.0, 0.5, 0.0);
    drawContour(res.waist_contour, "waist", 0.0, 1.0, 1.0);
    drawContour(res.hip_contour,   "hip",   1.0, 0.0, 1.0);
}

// ---------------------------------------------------------
// [重构] 计算主函数，剥离强耦合绘图代码
// ---------------------------------------------------------
void SingleModePage::onCalculateBodySize() {
    if (m_keypoints.size() != 6) {
        QMessageBox::warning(this, "警告", "关键点未准备就绪！请确保 6 个关键点已全部检测。");
        return;
    }
    if (!m_cloudData.contains("Merged") || !m_cloudData.contains("Body")) {
        QMessageBox::warning(this, "警告", "缺少主体或融合点云数据！");
        return;
    }

    log("开始执行体尺自动计算流水线...", "ALGO");

    float girth_thick = m_spinGirthThick->value();
    float skel_step = m_spinSkelStep->value();
    float skel_radius = m_spinSkelRadius->value();
    float height_angle = m_spinHeightAngle->value();

    PointCloudT::Ptr cloud_body(new PointCloudT(*m_cloudData["Body"]));
    PointCloudT::Ptr cloud_merged(new PointCloudT(*m_cloudData["Merged"]));
    
    BodySizeResults results = PointCloudAlgo::calculateAllMeasurements(
        cloud_body, cloud_merged, m_keypoints, 
        girth_thick, skel_step, skel_radius, height_angle,
        [this](const QString& msg, const QString& type){ this->log(msg, type); }
    );

    if (!results.aligned_cloud || results.aligned_cloud->empty()) {
        log("体尺计算失败！", "ERROR");
        return;
    }

    // 缓存数据
    m_latestResults = results;
    m_hasResults = true;

    // [核心改变] 自动关闭其它所有图层，并激活 "Measurements" 图层
    for (auto* chk : m_layerChecks) { 
        chk->blockSignals(true); 
        chk->setChecked(false); 
        chk->blockSignals(false); 
    }
    // 关闭所有普通点云
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        m_viewer->removePointCloud(it.key().toStdString());
    }
    
    // 勾选并触发 Measurements 图层，它会自动调用 drawMeasurements()
    if (m_layerChecks.contains("Measurements")) {
        m_layerChecks["Measurements"]->setChecked(true);
        onLayerToggle("Measurements", true);
    }
    
    m_viewer->resetCamera(); // 居中视角

    // 打印结果报告... (保留原来的 log 报告代码)
    log("==============================", "SUCCESS");
    log("       体 尺 测 量 报 告       ", "SUCCESS");
    log(QString("▶ 体长 (Body Length) : %1 mm").arg(results.body_length, 0, 'f', 2), "INFO");
    log(QString("▶ 体高 (Body Height) : %1 mm").arg(results.body_height, 0, 'f', 2), "INFO");
    log(QString("▶ 体宽 (Body Width)  : %1 mm").arg(results.body_width, 0, 'f', 2), "INFO");
    log(QString("▶ 胸围 (Chest Girth) : %1 mm").arg(results.chest_girth, 0, 'f', 2), "INFO");
    log(QString("▶ 腰围 (Waist Girth) : %1 mm").arg(results.waist_girth, 0, 'f', 2), "INFO");
    log(QString("▶ 臀围 (Hip Girth)   : %1 mm").arg(results.hip_girth, 0, 'f', 2), "INFO");
    log("==============================", "SUCCESS");
}


// ==========================================================
// 导出模块：另存融合点云 (PCD)
// ==========================================================
void SingleModePage::onExportMergedCloud() {
    // 检查是否存在融合点云
    if (!m_cloudData.contains("Merged") || m_cloudData["Merged"]->empty()) {
        QMessageBox::warning(this, "警告", "没有找到融合点云，请先执行配准与融合！");
        return;
    }

    // 弹出文件保存对话框
    QString defaultName = QString("Merged_Cloud_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "另存融合点云", defaultName, "PCD Files (*.pcd)");
    
    if (fileName.isEmpty()) return; // 用户取消了保存

    // 提示状态
    log("正在保存融合点云至磁盘...", "INFO");

    // 使用 PCL 保存点云为二进制格式（体积更小，加载更快）
    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *m_cloudData["Merged"]) == 0) {
        log(QString("✅ 融合点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 融合点云保存失败，请检查路径权限！", "ERROR");
    }
}

// ==========================================================
// 导出模块：另存主体点云 (PCD)
// ==========================================================
void SingleModePage::onExportBodyCloud() {
    // 检查是否存在猪主体点云
    if (!m_cloudData.contains("Body") || m_cloudData["Body"]->empty()) {
        QMessageBox::warning(this, "警告", "没有找到提取的主体点云，请先执行主体提取！");
        return;
    }

    QString defaultName = QString("Pig_Body_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "另存主体点云", defaultName, "PCD Files (*.pcd)");
    
    if (fileName.isEmpty()) return;

    log("正在保存主体点云至磁盘...", "INFO");

    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *m_cloudData["Body"]) == 0) {
        log(QString("✅ 主体点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 主体点云保存失败！", "ERROR");
    }
}

// ==========================================================
// 导出模块：导出测量报告 (TXT 精美排版)
// ==========================================================
void SingleModePage::onExportReport() {
    // 检查是否已经计算过体尺
    if (!m_hasResults) {
        QMessageBox::warning(this, "警告", "尚未生成体尺数据！请先点击【计算体尺参数】。");
        return;
    }

    // 1. 弹出保存对话框，后缀改为 .txt
    QString defaultName = QString("Pig_BodySize_Report_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "导出测量报告", defaultName, "Text Files (*.txt)");
    
    if (fileName.isEmpty()) return;

    // 2. 打开文件
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "错误", "无法创建或写入该 TXT 文件！文件可能被其他程序占用。");
        return;
    }

    QTextStream out(&file);
    
    // 兼容性设置：确保以 UTF-8 编码写入 (Qt6 默认已是 UTF-8)
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    out.setCodec("UTF-8");
#endif

    // =======================================================
    // 3. 开始精美排版写入
    // =======================================================
    out << "============================================================\n";
    out << "                     猪 只 体 尺 测 量 报 告                   \n";
    out << "============================================================\n";
    out << QString(" 测量时间 : %1\n").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    out << "------------------------------------------------------------\n";
    out << " 【测量参数】\t\t\t\t【测量值】\n";
    out << "------------------------------------------------------------\n";

    // 辅助 Lambda：使用固定宽度对齐文本 (-25表示左对齐占25字符，8表示右对齐占8字符)
    auto writeLine = [&out](const QString& name, double value) {
        out << " " << QString("%1").arg(name, -25) 
            << " :\t" 
            << QString("%1").arg(value, 8, 'f', 2) 
            << " mm\n";
    };

    writeLine("体长 (Body Length)", m_latestResults.body_length);
    writeLine("体高 (Body Height)", m_latestResults.body_height);
    writeLine("体宽 (Body Width)",  m_latestResults.body_width);
    writeLine("胸围 (Chest Girth)", m_latestResults.chest_girth);
    writeLine("腰围 (Waist Girth)", m_latestResults.waist_girth);
    writeLine("臀围 (Hip Girth)",   m_latestResults.hip_girth);

    out << "============================================================\n";
    out << " * 本报告由多相机 3D 点云处理系统自动生成。\n";

    file.close();
    
    log(QString("✅ 测量报告已成功导出为 TXT: %1").arg(fileName), "SUCCESS");
}

// ==========================================================
// 导出模块：导出测量结果的可视化彩色点云
// 核心技术：将所有 3D 图元(线、多边形、球)降维离散化为密集的彩色点云
// ==========================================================
void SingleModePage::onExportVizCloud() {
    if (!m_hasResults || !m_latestResults.aligned_cloud) {
        QMessageBox::warning(this, "警告", "尚未生成体尺数据！请先点击【计算体尺参数】。");
        return;
    }

    QString defaultName = QString("Pig_Visualization_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "导出测量可视化点云", defaultName, "PCD Files (*.pcd)");
    if (fileName.isEmpty()) return;

    log("正在将线框与图元渲染为彩色点云...", "ALGO");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    BodySizeResults& res = m_latestResults;

    // 1. 注入主体点云 (浅灰色)
    for (const auto& p : res.aligned_cloud->points) {
        pcl::PointXYZRGB pt; pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.r = 200; pt.g = 200; pt.b = 200;
        viz_cloud->push_back(pt);
    }

    // 辅助 Lambda：在两点之间进行插值，并生成物理“加粗”的点云管道
    auto drawLine = [&](const PointT& p1, const PointT& p2, uint8_t r, uint8_t g, uint8_t b, float thickness = 2.0f) {
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


    // 辅助 Lambda：在指定中心画一个实心球体 (关键点)
    auto drawSphere = [&](const PointT& center, float radius, uint8_t r, uint8_t g, uint8_t b) {
        float step = 1.5f; // 空间采样步长
        for (float dx = -radius; dx <= radius; dx += step) {
            for (float dy = -radius; dy <= radius; dy += step) {
                for (float dz = -radius; dz <= radius; dz += step) {
                    if (dx*dx + dy*dy + dz*dz <= radius*radius) {
                        pcl::PointXYZRGB pt; pt.x = center.x + dx; pt.y = center.y + dy; pt.z = center.z + dz;
                        pt.r = r; pt.g = g; pt.b = b;
                        viz_cloud->push_back(pt);
                    }
                }
            }
        }
    };

    // 2. 注入关键点 (红色球体, 半径 15mm)
    for (const auto& kp : res.aligned_keypoints) {
        drawSphere(kp, 15.0f, 255, 0, 0);
    }

    // 3. 注入骨架线 (绿色粗线, 线宽通过并排画多条线模拟，这里为了精简画单根密点线)
    if (res.skeleton_cloud && res.skeleton_cloud->size() > 1) {
        for (size_t i = 0; i < res.skeleton_cloud->size() - 1; ++i) {
            drawLine(res.skeleton_cloud->points[i], res.skeleton_cloud->points[i+1], 0, 255, 0, 3.0f);
        }
    }

    // 4. 注入体高线(蓝色) 和 体宽线(黄色)
    drawLine(res.height_top, res.height_bottom, 0, 0, 255, 2.5f);
    drawLine(res.width_p1, res.width_p2, 255, 255, 0, 2.5f);

    // 5. 注入轮廓线 (胸围橙色, 腰围青色, 臀围品红)
    auto drawContour = [&](PointCloudT::Ptr contour, uint8_t r, uint8_t g, uint8_t b) {
        if (!contour || contour->size() < 3) return;
        for (size_t i = 0; i < contour->size() - 1; ++i) {
            drawLine(contour->points[i], contour->points[i+1], r, g, b, 2.0f);
        }
        drawLine(contour->points.back(), contour->points.front(), r, g, b, 2.0f);
    };
    drawContour(res.chest_contour, 255, 128, 0);
    drawContour(res.waist_contour, 0, 255, 255);
    drawContour(res.hip_contour, 255, 0, 255);

    // 6. 注入地面边界框 (深绿色)
    if (res.ground_polygon && res.ground_polygon->size() == 4) {
        drawContour(res.ground_polygon, 0, 150, 0);
    }

    viz_cloud->width = viz_cloud->size();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    // 保存二进制 PCD
    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *viz_cloud) == 0) {
        log(QString("✅ 可视化点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 可视化点云保存失败！", "ERROR");
    }
}



void SingleModePage::initDefaultIntrinsics() {
    m_intrinsicsMap["Top"] = PointCloudAlgo::getCameraIntrinsics("Top", SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["LB"]  = PointCloudAlgo::getCameraIntrinsics("LB",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["LT"]  = PointCloudAlgo::getCameraIntrinsics("LT",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["RB"]  = PointCloudAlgo::getCameraIntrinsics("RB",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["RT"]  = PointCloudAlgo::getCameraIntrinsics("RT",  SensorType::DEPTH, 512, 512);
}


// [新增] 弹出一个极简的专业内参配置表格
void SingleModePage::onSetIntrinsics() {
    QDialog dlg(this);
    dlg.setWindowTitle("⚙️ 自定义相机内参 (RAW -> PCD)");
    dlg.resize(650, 250);
    QVBoxLayout *lay = new QVBoxLayout(&dlg);

    QTableWidget *table = new QTableWidget(5, 6, &dlg);
    table->setHorizontalHeaderLabels({"fx", "fy", "cx", "cy", "Width", "Height"});
    table->setVerticalHeaderLabels({"Top", "LB", "LT", "RB", "RT"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    
    QStringList keys = {"Top", "LB", "LT", "RB", "RT"};
    for (int r = 0; r < keys.size(); ++r) {
        CameraIntrinsics intr = m_intrinsicsMap[keys[r]];
        table->setItem(r, 0, new QTableWidgetItem(QString::number(intr.fx, 'f', 4)));
        table->setItem(r, 1, new QTableWidgetItem(QString::number(intr.fy, 'f', 4)));
        table->setItem(r, 2, new QTableWidgetItem(QString::number(intr.cx, 'f', 4)));
        table->setItem(r, 3, new QTableWidgetItem(QString::number(intr.cy, 'f', 4)));
        table->setItem(r, 4, new QTableWidgetItem(QString::number(intr.width)));
        table->setItem(r, 5, new QTableWidgetItem(QString::number(intr.height)));
    }
    lay->addWidget(table);

    QPushButton *btnSave = new QPushButton("💾 保存并应用", &dlg);
    lay->addWidget(btnSave);

    connect(btnSave, &QPushButton::clicked, [&]() {
        for (int r = 0; r < keys.size(); ++r) {
            QString key = keys[r];
            m_intrinsicsMap[key].fx = table->item(r, 0)->text().toFloat();
            m_intrinsicsMap[key].fy = table->item(r, 1)->text().toFloat();
            m_intrinsicsMap[key].cx = table->item(r, 2)->text().toFloat();
            m_intrinsicsMap[key].cy = table->item(r, 3)->text().toFloat();
            m_intrinsicsMap[key].width = table->item(r, 4)->text().toInt();
            m_intrinsicsMap[key].height = table->item(r, 5)->text().toInt();
        }
        log("相机内参已更新，下次加载 RAW 文件将使用新参数。", "INFO");
        dlg.accept();
    });

    dlg.exec();
}


void SingleModePage::onRunAIInference2D() {
    prepareKeypointsCloud(); 
    if (m_topColorImage.empty() || m_topAlignedDepthImage.empty()) {
        QMessageBox::warning(this, "错误", "缺少 Top 相机的彩色图或对齐深度图！");
        return;
    }

    QString modelPath = m_leModelPath->text();
    QString initMsg; 
    if (!m_openpose.initModel(modelPath, initMsg)) {
        log(QString("ONNX 加载失败: %1").arg(initMsg), "ERROR"); return;
    }

    log("正在执行本地 2D 图像深度学习推理...", "ALGO");
    
    // 1. 接收带有置信度的 3D 向量 (x, y, 置信度)
    std::vector<cv::Point3f> kps2d_with_conf = m_openpose.predict(m_topColorImage);
    this->m_keypoints.clear();
    
    // 获取相机内外参
    CameraIntrinsics colorIntr = PointCloudAlgo::getCameraIntrinsics("Top", SensorType::COLOR, 1280, 720);
    CameraDeviceParams params = PointCloudAlgo::getCameraParams("Top");

    Eigen::Matrix3f R;
    R << params.extrinsics.R[0], params.extrinsics.R[1], params.extrinsics.R[2],
         params.extrinsics.R[3], params.extrinsics.R[4], params.extrinsics.R[5],
         params.extrinsics.R[6], params.extrinsics.R[7], params.extrinsics.R[8];
    Eigen::Vector3f T(params.extrinsics.T[0], params.extrinsics.T[1], params.extrinsics.T[2]);

    // 准备 OpenCV 矩阵用于去畸变运算
    cv::Mat camMat = (cv::Mat_<double>(3, 3) << colorIntr.fx, 0, colorIntr.cx, 0, colorIntr.fy, colorIntr.cy, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 8) << colorIntr.k1, colorIntr.k2, colorIntr.p1, colorIntr.p2, colorIntr.k3, colorIntr.k4, colorIntr.k5, colorIntr.k6);

    // 克隆一张图用于弹窗可视化
    cv::Mat vizImage = m_topColorImage.clone();

    int validCount = 0;
    for (int i = 0; i < kps2d_with_conf.size(); ++i) {
        float u = kps2d_with_conf[i].x;
        float v = kps2d_with_conf[i].y;
        float conf = kps2d_with_conf[i].z; // 提取热力值
        
        // 【新增 1】将热力值打印到 UI 左下角的自定义日志框中
        log(QString("P%1 置信度: %2").arg(i+1).arg(conf, 0, 'f', 4), "INFO");

        if (u < 0 || v < 0 || conf < 0.1) { 
            log(QString("P%1 关键点未有效检测到！(置信度: %2)").arg(i+1).arg(conf, 0, 'f', 4), "WARN");
            continue;
        }

        // 画 2D 点到可视化图上
        cv::circle(vizImage, cv::Point(u, v), 8, cv::Scalar(0, 0, 255), -1);
        cv::putText(vizImage, "P" + std::to_string(i+1), cv::Point(u + 10, v), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // 深度补洞逻辑 (不变)
        int ui = cvRound(u), vi = cvRound(v);
        uint16_t z_raw = m_topAlignedDepthImage.at<uint16_t>(vi, ui);
        if (z_raw == 0) {
            int r = 3; float sum = 0; int cnt = 0;
            for(int dy = -r; dy <= r; ++dy) {
                for(int dx = -r; dx <= r; ++dx) {
                    int ny = vi + dy, nx = ui + dx;
                    if(nx >= 0 && nx < 1280 && ny >= 0 && ny < 720) {
                        uint16_t val = m_topAlignedDepthImage.at<uint16_t>(ny, nx);
                        if(val > 0) { sum += val; cnt++; }
                    }
                }
            }
            if(cnt > 0) z_raw = sum / cnt; 
        }

        if (z_raw == 0) continue;

        // =======================================================
        // 【新增 2：核心数学修复】对 2D 坐标进行 OpenCV 物理去畸变
        // =======================================================
        std::vector<cv::Point2f> srcPts = {cv::Point2f(u, v)};
        std::vector<cv::Point2f> dstPts;
        // undistortPoints 会直接输出“归一化”的理想坐标 (x/Z, y/Z)
        cv::undistortPoints(srcPts, dstPts, camMat, distCoeffs); 

        float z_c = static_cast<float>(z_raw); 
        // 直接乘以深度，得到 Color 相机坐标系下的完美 3D 坐标
        float x_c = dstPts[0].x * z_c;
        float y_c = dstPts[0].y * z_c;

        Eigen::Vector3f P_color(x_c, y_c, z_c);
        
        // 逆变换回 Depth 点云坐标系
        Eigen::Vector3f P_depth = R.transpose() * (P_color - T);
        
        this->m_keypoints.push_back(P_depth);
        validCount++;

        // 【新增需求】：在控制台中漂亮地输出该点的 2D 像素坐标、3D 空间坐标以及置信度
        log(QString("P%1 [2D] 像素: (%2, %3) | [3D] 毫米: (X:%4, Y:%5, Z:%6) | 置信度: %7")
            .arg(i+1)
            .arg(u, 0, 'f', 1).arg(v, 0, 'f', 1)
            .arg(P_depth.x(), 0, 'f', 1).arg(P_depth.y(), 0, 'f', 1).arg(P_depth.z(), 0, 'f', 1)
            .arg(conf, 0, 'f', 4), "INFO");
    }

    // 【新增 3】弹出带有检测结果的 OpenCV 图片窗口
    cv::namedWindow("2D AI Detection Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("2D AI Detection Result", 1280, 720);
    cv::imshow("2D AI Detection Result", vizImage);

    // 更新 UI 状态
    if (validCount == 6) {
        log("🎉 AI 关键点检测完成，经过物理去畸变与外参映射，已贴合至点云！", "SUCCESS");
        this->drawKeypointsInViewer(this->m_keypoints);
        for (int i = 0; i < m_kpBadges.size(); ++i) updateBadgeStyle(i, 2);
        
        // 【新增 4】强制点亮融合点云/主体点云的图层复选框，让底图显示出来
        if (m_layerChecks.contains("Body")) {
            m_layerChecks["Body"]->setChecked(true); // 激活主体猪图层
        } else if (m_layerChecks.contains("Merged")) {
            m_layerChecks["Merged"]->setChecked(true); // 退而求其次激活融合图层
        }
    } else {
        log("模型未能完整检测 6 个点，请检查图像质量或转入手动模式。", "WARN");
    }
}