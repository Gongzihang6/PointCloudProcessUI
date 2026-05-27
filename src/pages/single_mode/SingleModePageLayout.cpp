/*
 * 文件说明：实现 `SingleModePage` 的页面骨架、交互容器与主界面布局。
 */
#include "pages/single_mode/SingleModePageInternal.h"

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
    // 上半部分：3D 视图容器 (QVTKOpenGLNativeWidget)
    // 使用 Qt 官方推荐的 QOpenGLWidget 方案，避免原生子窗口嵌入 splitter 时的黑块和拖影。
    // ==========================================
    m_vtkWidget = new QVTKOpenGLNativeWidget(centerPanel);
    m_vtkWidget->setMinimumHeight(120);
    splitter->addWidget(m_vtkWidget);

    m_vtkRenderWindow = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
    m_vtkRenderer = vtkSmartPointer<vtkRenderer>::New();
    m_vtkRenderWindow->AddRenderer(m_vtkRenderer);
    m_vtkWidget->setRenderWindow(m_vtkRenderWindow);

    m_viewer.reset(new pcl::visualization::PCLVisualizer(m_vtkRenderer, m_vtkRenderWindow, "viewer", false));
    m_viewer->setupInteractor(m_vtkWidget->interactor(), m_vtkWidget->renderWindow());
    if (m_vtkWidget->interactor()) {
        m_vtkWidget->interactor()->Initialize();
    }
    m_viewer->setBackgroundColor(0.1, 0.1, 0.1);
    m_viewer->addCoordinateSystem(100.0);
    m_viewer->initCameraParameters();

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
    vtkRenderWindowInteractor* iren = m_vtkWidget->interactor();
    vtkInteractorStyleTrackballCamera* style = nullptr;
    if (iren) {
        style = vtkInteractorStyleTrackballCamera::SafeDownCast(iren->GetInteractorStyle());
    }

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
    splitter->setChildrenCollapsible(false);
    splitter->setSizes({640, 160});

    // 将分割器加入主布局
    mainLayout->addWidget(splitter);
}


void SingleModePage::initRightPanel() {
    rightPanel = new QWidget(this);
    rightPanel->setFixedWidth(440);
    rightPanel->setStyleSheet("background: #f7f8fa; border-left: 1px solid #dcdfe6;");
    
    QVBoxLayout *mainLayout = new QVBoxLayout(rightPanel);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    QScrollArea *scroll = new QScrollArea();
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setStyleSheet(
        "QScrollArea { background: #f7f8fa; border: none; }"
        "QScrollBar:vertical { width: 10px; background: #f0f2f5; margin: 4px 2px; border-radius: 5px; }"
        "QScrollBar::handle:vertical { background: #c0c4cc; min-height: 30px; border-radius: 5px; }"
        "QScrollBar::handle:vertical:hover { background: #909399; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0px; }"
    );
    
    QWidget *scrollContent = new QWidget();
    QVBoxLayout *scrollLayout = new QVBoxLayout(scrollContent);
    scrollLayout->setSpacing(12);
    scrollLayout->setContentsMargins(10, 10, 10, 10);

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
    lay3->setSpacing(10);

    QLabel *extractTip = new QLabel("默认推荐使用 WSL 语义分割，直接对融合后的完整点云做主体提取；传统规则提取作为高级备选方案保留。");
    extractTip->setWordWrap(true);
    extractTip->setStyleSheet("color: #606266; line-height: 1.5; background: #f5f7fa; border: 1px solid #ebeef5; border-radius: 6px; padding: 8px;");
    lay3->addWidget(extractTip);

    auto *rowExtractMode = new QHBoxLayout();
    rowExtractMode->addWidget(new QLabel("提取模式:"));
    m_comboBodyExtractMode = new QComboBox();
    m_comboBodyExtractMode->addItems({"WSL 语义分割 (推荐)", "传统规则提取 (高级)"});
    rowExtractMode->addWidget(m_comboBodyExtractMode);
    lay3->addLayout(rowExtractMode);

    m_extractModeStack = new QStackedWidget();

    // -------------------------------------------------
    // 3A. WSL 语义分割模式
    // -------------------------------------------------
    QWidget *segWidget = new QWidget();
    QVBoxLayout *segLay = new QVBoxLayout(segWidget);
    segLay->setContentsMargins(0, 0, 0, 0);
    segLay->setSpacing(10);

    QFrame *segInfoFrame = new QFrame();
    segInfoFrame->setStyleSheet("background: #ecf5ff; border: 1px solid #d9ecff; border-radius: 6px;");
    QVBoxLayout *segInfoLay = new QVBoxLayout(segInfoFrame);
    segInfoLay->setContentsMargins(10, 8, 10, 8);
    segInfoLay->addWidget(new QLabel("输入数据：配准融合后的完整点云 (Merged)"));
    segInfoLay->addWidget(new QLabel("后端职责：文件读取、预处理、语义分割推理、主体点云回传"));
    QLabel *segHint = new QLabel("建议后端返回 JSON 点坐标数组，或直接返回 PCD 二进制文件。");
    segHint->setWordWrap(true);
    segHint->setStyleSheet("color: #606266;");
    segInfoLay->addWidget(segHint);
    segLay->addWidget(segInfoFrame);

    auto *rowSegUrl = new QVBoxLayout();
    QLabel *segUrlLabel = new QLabel("WSL 语义分割服务地址:");
    segUrlLabel->setStyleSheet("font-weight: bold; color: #606266;");
    rowSegUrl->addWidget(segUrlLabel);
    m_leSegServiceUrl = new QLineEdit("http://127.0.0.1:8002/predict");
    m_leSegServiceUrl->setPlaceholderText("例如: http://127.0.0.1:8002/predict");
    rowSegUrl->addWidget(m_leSegServiceUrl);
    segLay->addLayout(rowSegUrl);

    QLabel *segNote = new QLabel("执行后会自动回填为 `Body` 图层，并清空旧的测量结果，方便直接进入关键点与体尺流程。");
    segNote->setWordWrap(true);
    segNote->setStyleSheet("color: #909399; font-size: 12px;");
    segLay->addWidget(segNote);
    segLay->addStretch();

    // -------------------------------------------------
    // 3B. 传统规则提取模式
    // -------------------------------------------------
    QWidget *legacyWidget = new QWidget();
    QVBoxLayout *legacyLay = new QVBoxLayout(legacyWidget);
    legacyLay->setContentsMargins(0, 0, 0, 0);
    legacyLay->setSpacing(10);

    QLabel *legacyHint = new QLabel("当语义分割服务未启动、或需要做规则法回退验证时，可切换到该模式。");
    legacyHint->setWordWrap(true);
    legacyHint->setStyleSheet("color: #909399; font-size: 12px;");
    legacyLay->addWidget(legacyHint);

    QGroupBox *boxGroup = new QGroupBox("空间裁剪范围 (CropBox X/Y/Z)");
    QGridLayout *boxLay = new QGridLayout(boxGroup);

    auto makeBoxSpin = [](double min, double max, double val) {
        QDoubleSpinBox* s = new QDoubleSpinBox();
        s->setRange(min, max);
        s->setValue(val);
        s->setDecimals(0);
        s->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
        return s;
    };
    m_spinBoxMinX = makeBoxSpin(-3000, 3000, -1200);
    m_spinBoxMinY = makeBoxSpin(-3000, 3000, -460);
    m_spinBoxMinZ = makeBoxSpin(-3000, 3000, -500);
    m_spinBoxMaxX = makeBoxSpin(-3000, 3000, 600);
    m_spinBoxMaxY = makeBoxSpin(-3000, 3000, 170);
    m_spinBoxMaxZ = makeBoxSpin(-3000, 3000, 2100);

    boxLay->addWidget(new QLabel("Min:"), 0, 0);
    boxLay->addWidget(m_spinBoxMinX, 0, 1);
    boxLay->addWidget(m_spinBoxMinY, 0, 2);
    boxLay->addWidget(m_spinBoxMinZ, 0, 3);
    boxLay->addWidget(new QLabel("Max:"), 1, 0);
    boxLay->addWidget(m_spinBoxMaxX, 1, 1);
    boxLay->addWidget(m_spinBoxMaxY, 1, 2);
    boxLay->addWidget(m_spinBoxMaxZ, 1, 3);

    m_spinBoxRotZ = new QDoubleSpinBox();
    m_spinBoxRotZ->setRange(-180.0, 180.0);
    m_spinBoxRotZ->setValue(33.0);
    m_spinBoxRotZ->setDecimals(1);
    m_spinBoxRotZ->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    boxLay->addWidget(new QLabel("Z轴旋转(度):"), 2, 0);
    boxLay->addWidget(m_spinBoxRotZ, 2, 1);
    legacyLay->addWidget(boxGroup);

    auto *rowMinPts = new QHBoxLayout();
    rowMinPts->addWidget(new QLabel("最小连续点数:"));
    m_spinExtMinPts = new QSpinBox();
    m_spinExtMinPts->setRange(100, 100000);
    m_spinExtMinPts->setValue(5000);
    m_spinExtMinPts->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    rowMinPts->addWidget(m_spinExtMinPts);
    legacyLay->addLayout(rowMinPts);

    auto *rowExtAlgo = new QHBoxLayout();
    rowExtAlgo->addWidget(new QLabel("主体聚类算法:"));
    m_comboExtMethod = new QComboBox();
    m_comboExtMethod->addItems({"欧式聚类 (距离连通)", "区域生长 (曲面平滑度)"});
    rowExtAlgo->addWidget(m_comboExtMethod);
    legacyLay->addLayout(rowExtAlgo);

    m_euclideanParamsWidget = new QWidget();
    QFormLayout *eLay = new QFormLayout(m_euclideanParamsWidget);
    eLay->setContentsMargins(15, 0, 0, 0);
    m_spinEuclideanTol = new QDoubleSpinBox();
    m_spinEuclideanTol->setRange(1.0, 200.0);
    m_spinEuclideanTol->setValue(40.0);
    m_spinEuclideanTol->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    eLay->addRow("聚类搜索容差(mm):", m_spinEuclideanTol);
    legacyLay->addWidget(m_euclideanParamsWidget);

    m_rgParamsWidget = new QWidget();
    QFormLayout *rgLay = new QFormLayout(m_rgParamsWidget);
    rgLay->setContentsMargins(15, 0, 0, 0);
    m_spinRgNeighbors = new QSpinBox();
    m_spinRgNeighbors->setRange(5, 100);
    m_spinRgNeighbors->setValue(30);
    m_spinRgNeighbors->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    m_spinRgSmoothness = new QDoubleSpinBox();
    m_spinRgSmoothness->setRange(1.0, 45.0);
    m_spinRgSmoothness->setValue(7.0);
    m_spinRgSmoothness->setStyleSheet("background: #fff; border: 1px solid #dcdfe6;");
    rgLay->addRow("法线搜索邻居数:", m_spinRgNeighbors);
    rgLay->addRow("平滑度阈值(度):", m_spinRgSmoothness);
    legacyLay->addWidget(m_rgParamsWidget);

    m_rgParamsWidget->setVisible(false);
    connect(m_comboExtMethod, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index){
        m_euclideanParamsWidget->setVisible(index == 0);
        m_rgParamsWidget->setVisible(index == 1);
    });

    QFrame *ransacFrame = new QFrame();
    ransacFrame->setStyleSheet("background: #fdf6ec; border: 1px solid #faecd8; border-radius: 4px;");
    QHBoxLayout *ransacLay = new QHBoxLayout(ransacFrame);
    ransacLay->setContentsMargins(10, 6, 10, 6);

    m_chkUseRansac = new QCheckBox("开启 RANSAC 平面剔除");
    m_chkUseRansac->setToolTip("在聚类后尝试寻找并移除点云中的最大平面，用于清理残留地面");

    m_spinRansacDist = new QDoubleSpinBox();
    m_spinRansacDist->setRange(1.0, 100.0);
    m_spinRansacDist->setValue(30.0);
    m_spinRansacDist->setSuffix(" mm");
    m_spinRansacDist->setEnabled(false);
    ransacLay->addWidget(m_chkUseRansac);
    ransacLay->addWidget(new QLabel("阈值:"));
    ransacLay->addWidget(m_spinRansacDist);
    legacyLay->addWidget(ransacFrame);
    connect(m_chkUseRansac, &QCheckBox::toggled, m_spinRansacDist, &QDoubleSpinBox::setEnabled);

    m_chkMlsUpsampling = new QCheckBox("开启 MLS 上采样补洞", this);
    m_chkMlsUpsampling->setChecked(true);

    m_spinMlsRadius = new QDoubleSpinBox(this);
    m_spinMlsRadius->setRange(10.0, 100.0);
    m_spinMlsRadius->setSingleStep(5.0);
    m_spinMlsRadius->setValue(80.0);
    m_spinMlsRadius->setSuffix(" mm");

    m_spinMlsUpsampleRadius = new QDoubleSpinBox(this);
    m_spinMlsUpsampleRadius->setRange(5.0, 100.0);
    m_spinMlsUpsampleRadius->setSingleStep(5.0);
    m_spinMlsUpsampleRadius->setValue(25.0);
    m_spinMlsUpsampleRadius->setSuffix(" mm");

    m_spinMlsUpsampleStep = new QDoubleSpinBox(this);
    m_spinMlsUpsampleStep->setRange(1.0, 50.0);
    m_spinMlsUpsampleStep->setSingleStep(1.0);
    m_spinMlsUpsampleStep->setValue(25.0);
    m_spinMlsUpsampleStep->setSuffix(" mm");

    QFormLayout* mlsLay = new QFormLayout();
    mlsLay->setContentsMargins(5, 8, 0, 0);
    mlsLay->addRow(new QLabel("<b>MLS 平滑与补孔</b>"));
    mlsLay->addRow(m_chkMlsUpsampling);
    mlsLay->addRow("MLS 搜索半径:", m_spinMlsRadius);
    mlsLay->addRow("补孔半径:", m_spinMlsUpsampleRadius);
    mlsLay->addRow("生成点步长:", m_spinMlsUpsampleStep);
    legacyLay->addLayout(mlsLay);
    legacyLay->addStretch();

    m_extractModeStack->addWidget(segWidget);
    m_extractModeStack->addWidget(legacyWidget);
    lay3->addWidget(m_extractModeStack);

    m_btnExtractBody = new QPushButton();
    m_btnExtractBody->setObjectName("PrimaryBtn");
    m_btnExtractBody->setMinimumHeight(38);
    lay3->addWidget(m_btnExtractBody);
    connect(m_btnExtractBody, &QPushButton::clicked, this, &SingleModePage::onExtractBody);

    connect(m_comboBodyExtractMode, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this](int index) {
        if (m_extractModeStack) {
            m_extractModeStack->setCurrentIndex(index);
        }
        if (m_btnExtractBody) {
            m_btnExtractBody->setText(index == 0 ? "🐷 调用 WSL 语义分割提取主体"
                                                 : "🐷 执行传统规则主体提取");
        }
    });
    m_comboBodyExtractMode->setCurrentIndex(0);
    m_extractModeStack->setCurrentIndex(0);
    m_btnExtractBody->setText("🐷 调用 WSL 语义分割提取主体");

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
