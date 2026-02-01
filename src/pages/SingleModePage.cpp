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
#include <vtkRenderWindow.h> // [新增] VTK 标准渲染窗口
#include <vtkGenericOpenGLRenderWindow.h> // 必须包含这个
#include <pcl/io/pcd_io.h>                // 用于读取 PCD
#include <QMessageBox>                    // 用于报错提示
#include <QPixmap>
#include <QIcon>
#include <QColor>
#include <QDoubleSpinBox>
#include <QSplitter>
#include <QDateTime>
#include <QScrollBar>

SingleModePage::SingleModePage(QWidget *parent) : QWidget(parent) {
    auto *layout = new QHBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);
    layout->setSpacing(0);

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
    
    // [新增] 底部按钮组容器
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

    // 将按钮加入水平布局
    bottomBtnLayout->addWidget(loadFolderBtn, 3); // 比例 3
    bottomBtnLayout->addWidget(clearBtn, 1);      // 比例 1

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
        {"Body",   "🐷 提取主体", "pink"}
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
    // ==========================================
    m_vtkContainer = new QWidget();
    m_vtkContainer->setAttribute(Qt::WA_OpaquePaintEvent);
    m_vtkContainer->setAttribute(Qt::WA_PaintOnScreen);
    m_vtkContainer->setAttribute(Qt::WA_NativeWindow);

    // 将 VTK 容器加入分割器 (Index 0)
    splitter->addWidget(m_vtkContainer);

    // --- VTK 初始化逻辑保持不变 ---
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    renWin->SetParentId(reinterpret_cast<void*>(m_vtkContainer->winId()));

    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renWin->AddRenderer(renderer);

    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renWin, "viewer", true));
    if (m_viewer->getRenderWindow()->GetInteractor()) {
        m_viewer->getRenderWindow()->GetInteractor()->Initialize();
    }
    m_viewer->setBackgroundColor(0.1, 0.1, 0.1);
    m_viewer->addCoordinateSystem(100.0);
    m_viewer->initCameraParameters();

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
    m_console->setReadOnly(true); // 只读
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
    rightPanel->setFixedWidth(340);
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
    // [修复 2] 连接执行按钮 -> 触发应用 (更新内存数据)
    // 我们需要一个新的槽函数 onApplyPreprocess，或者用 Lambda
    connect(btnExec1, &QPushButton::clicked, this, [this](){
        // 先运行一遍处理逻辑
        onRunPreprocess(); 
        applyPreprocessToMemory(); 
    });
    lay1->addLayout(btnRow1);

    box1->setContentLayout(lay1);
    scrollLayout->addWidget(box1);

    // =================================================
    // 2. 配准与融合 (Registration)
    // =================================================
    // 先调用初始化
    initDefaultMatrices();

    auto *box2 = new CollapsibleBox("2. 配准与融合 (Registration)");
    auto *lay2 = new QVBoxLayout();
    lay2->setSpacing(12);

    // [新增] 2.0 配准目标选择 (Reference Target)
    auto *rowRegTarget = new QHBoxLayout();
    QLabel *lblRef = new QLabel("参考目标 (Reference):");
    lblRef->setToolTip("选择源点云要配准到哪个坐标系\n默认是 Top，也可以选相邻相机");
    rowRegTarget->addWidget(lblRef);

    m_comboRegTarget = new QComboBox();
    // 目标可以是任意一个相机
    m_comboRegTarget->addItems({"Top", "LB", "LT", "RB", "RT"}); 
    rowRegTarget->addWidget(m_comboRegTarget);
    lay2->addLayout(rowRegTarget);

    // 2.1 算法选择
    auto *rowAlgo = new QHBoxLayout();
    rowAlgo->addWidget(new QLabel("算法:"));
    m_comboRegMethod = new QComboBox();
    m_comboRegMethod->addItems({"手动矩阵 (Manual)", "ICP (P2Point)", "ICP (P2Plane)"});
    rowAlgo->addWidget(m_comboRegMethod);
    lay2->addLayout(rowAlgo);

    // 2.2 [关键新增] 参与配准的源点云选择
    QLabel* lblSrc = new QLabel("选择待配准源 (Target=Top):");
    lblSrc->setStyleSheet("font-weight: bold; color: #409eff;");
    lay2->addWidget(lblSrc);

    auto *gridSrc = new QGridLayout();
    QStringList sources = {"LB", "LT", "RB", "RT"};
    int col = 0, row = 0;
    for(const auto& src : sources) {
        QCheckBox* chk = new QCheckBox(src);
        chk->setChecked(true); // 默认全选
        m_sourceChecks[src] = chk;
        gridSrc->addWidget(chk, row, col);
        col++; if(col > 1) { col=0; row++; }
    }
    lay2->addLayout(gridSrc);

    // 2.3 矩阵编辑器 (联动显示)
    QFrame *matrixFrame = new QFrame();
    matrixFrame->setStyleSheet("background: #f8f9fa; border: 1px solid #e4e7ed; border-radius: 4px; padding: 5px;");
    QVBoxLayout *matrixLay = new QVBoxLayout(matrixFrame);
    
    auto *rowMatTarget = new QHBoxLayout();
    rowMatTarget->addWidget(new QLabel("编辑矩阵:"));
    m_comboMatrixView = new QComboBox();
    m_comboMatrixView->addItems(sources); // LB, LT, RB, RT
    rowMatTarget->addWidget(m_comboMatrixView);
    matrixLay->addLayout(rowMatTarget);

    m_textMatrix = new QTextEdit();
    m_textMatrix->setObjectName("MatrixEditor");
    m_textMatrix->setFixedHeight(120);
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
    QPushButton *btnReg = new QPushButton("🚀 执行配准与融合");
    btnReg->setObjectName("PrimaryBtn");
    lay2->addWidget(btnReg);
    
    connect(btnReg, &QPushButton::clicked, this, &SingleModePage::onExecuteRegistration);

    box2->setContentLayout(lay2);
    scrollLayout->addWidget(box2);

    // =================================================
    // 3. 主体精细提取 (Extraction) - [补全]
    // =================================================
    auto *box3 = new CollapsibleBox("3. 主体精细提取 (Extraction)");
    auto *lay3 = new QVBoxLayout();
    lay3->setSpacing(8);
    
    // 补齐之前的缺失参数
    addParam(lay3, "聚类容差 (Tol):", 0.05, "m");
    
    auto *rowMinSize = new QHBoxLayout();
    rowMinSize->addWidget(new QLabel("最小簇点数:"));
    QSpinBox *sbMinSize = new QSpinBox(); sbMinSize->setRange(1, 10000); sbMinSize->setValue(100);
    sbMinSize->setButtonSymbols(QAbstractSpinBox::NoButtons);
    sbMinSize->setStyleSheet("background: #fff; border: 1px solid #dcdfe6; border-radius: 3px; padding: 2px;");
    rowMinSize->addWidget(sbMinSize);
    lay3->addLayout(rowMinSize);

    QPushButton *btnExtract = new QPushButton("🐷 提取最大主体");
    btnExtract->setObjectName("PrimaryBtn"); // 蓝色按钮
    lay3->addWidget(btnExtract);

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

    // --- AI 控制区 (模型加载 + 推理) ---
    QWidget *aiWidget = new QWidget();
    QVBoxLayout *aiLay = new QVBoxLayout(aiWidget);
    aiLay->setContentsMargins(0,0,0,0);
    aiLay->setSpacing(5);

    auto *fileRow = new QHBoxLayout();
    QLineEdit *leModel = new QLineEdit("./models/pig_kp_v3.onnx");
    leModel->setReadOnly(true); 
    leModel->setStyleSheet("color: #666; background: #f4f4f5; border: none;");
    QPushButton *btnBrowse = new QPushButton("📂"); btnBrowse->setFixedWidth(30);
    btnBrowse->setStyleSheet("border: none; background: transparent;");
    fileRow->addWidget(leModel);
    fileRow->addWidget(btnBrowse);
    
    // 给文件行加个外框，像截图里那样
    QFrame *fileFrame = new QFrame();
    fileFrame->setStyleSheet("border: 1px solid #dcdfe6; border-radius: 4px; background: #f4f4f5;");
    fileFrame->setLayout(fileRow);
    // 修正 fileRow 的 margin 以适应外框
    fileRow->setContentsMargins(5, 0, 0, 0);

    aiLay->addWidget(fileFrame);

    QPushButton *btnRunAI = new QPushButton("⚡ 运行模型推理");
    btnRunAI->setObjectName("WarningBtn"); // 橙色按钮 QSS
    btnRunAI->setFixedHeight(32);
    aiLay->addWidget(btnRunAI);

    lay4->addWidget(aiWidget);

    // 连接切换信号：点击手动时隐藏 AI 控件，点击 AI 时显示
    connect(toggleGroup, &QButtonGroup::idClicked, [aiWidget](int){
        // 简单逻辑：如果 AI 被选中则显示，否则隐藏
        // 这里需要获取 sender 或者直接判断 checked 状态，简化起见：
        // 实际开发中建议单独写 slot
    });
    // 修正：直接连接 lambda
    connect(btnManual, &QPushButton::toggled, [aiWidget](bool checked){
        aiWidget->setVisible(!checked);
    });

    // --- 关键点状态网格 ---
    QLabel *lblKpStatus = new QLabel("关键点状态 (Keypoints):");
    lblKpStatus->setStyleSheet("font-weight: bold; font-size: 11px; color: #606266; margin-top: 5px;");
    lay4->addWidget(lblKpStatus);

    QGridLayout *gridKp = new QGridLayout();
    gridKp->setSpacing(6);
    QStringList kps = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
    
    for(int i=0; i<kps.size(); ++i) {
        QLabel *badge = new QLabel(kps[i]);
        badge->setObjectName("KpBadge"); // QSS
        // 模拟：前3个检测到了，后3个没检测到
        if(i < 3) {
            badge->setProperty("detected", true);
        } else {
            badge->setProperty("detected", false);
        }
        gridKp->addWidget(badge, i/3, i%3);
    }
    lay4->addLayout(gridKp);

    // 计算按钮
    QPushButton *btnCalc = new QPushButton("📏 计算体尺参数");
    btnCalc->setObjectName("PrimaryBtn");
    btnCalc->setMinimumHeight(45);
    btnCalc->setStyleSheet("margin-top: 5px; font-weight: bold; font-size: 14px;");
    lay4->addWidget(btnCalc);

    box4->setContentLayout(lay4);
    scrollLayout->addWidget(box4);


    // =================================================
    // 5. 导出 (Export) - [补全]
    // =================================================
    auto *box5 = new CollapsibleBox("5. 导出 (Export)");
    auto *lay5 = new QVBoxLayout();
    lay5->setSpacing(8);

    auto addExportBtn = [](QVBoxLayout* l, QString text) {
        QPushButton* b = new QPushButton(text);
        b->setStyleSheet("text-align: left; padding-left: 15px;");
        l->addWidget(b);
    };

    addExportBtn(lay5, "💾 另存融合点云 (.pcd)");
    addExportBtn(lay5, "💾 另存主体点云 (.pcd)");
    addExportBtn(lay5, "📄 导出测量报告 (.csv)");

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
        "选择点云文件 (" + key + ")", 
        "", 
        "Point Cloud Files (*.pcd *.ply);;All Files (*)"
    );

    if (!fileName.isEmpty()) {
        if (m_fileInputs.contains(key)) {
            QLineEdit* edit = m_fileInputs[key];
            QFileInfo fileInfo(fileName);

            // [修改] 1. 界面只显示文件名
            edit->setText(fileInfo.fileName());
            
            // [修改] 2. 将完整路径存储在自定义属性 "fullPath" 中
            edit->setProperty("fullPath", fileInfo.absoluteFilePath());
            
            // [修改] 3. 设置鼠标悬停提示，方便用户查看完整路径
            edit->setToolTip(fileInfo.absoluteFilePath());
            
            // [UI反馈] 光标移到最前，防止长文件名看不见开头
            edit->setCursorPosition(0); 

            // [新增] 立即加载并显示这个点云
            loadCloudToMemory(key, fileName);
        }
    }
}


void SingleModePage::onLoadFolder() {
    QString dirPath = QFileDialog::getExistingDirectory(this, "选择包含点云数据的文件夹", "");
    if (dirPath.isEmpty()) return;

    QDir dir(dirPath);
    QStringList filters; filters << "*.pcd";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    QMap<QString, QString> keyMap;
    keyMap["005J"] = "Top";
    keyMap["00SE"] = "LT"; 
    keyMap["003W"] = "LB"; 
    keyMap["00YA"] = "RB";
    keyMap["00X6"] = "RT";

    // 1. 先清空
    onClearFiles();
    log("开始批量加载文件夹...", "INFO");

    int matchCount = 0;
    // 你的文件名后缀
    const QString targetSuffix = "_d_pc.pcd";

    for (const QFileInfo& fileInfo : fileList) {
        QString fileName = fileInfo.fileName();

        // 后缀检查
        if (!fileName.endsWith(targetSuffix, Qt::CaseInsensitive)) {
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

// [新增] 清空功能实现
void SingleModePage::onClearFiles() {
    for (auto it = m_fileInputs.begin(); it != m_fileInputs.end(); ++it) {
        QLineEdit* edit = it.value();
        edit->clear();                    // 清空显示的文字
        edit->setProperty("fullPath", ""); // 清空存储的路径
        edit->setToolTip("");             // 清空提示
    }
    // [新增] 清空内存数据和 3D 视图
    m_cloudData.clear();
    m_viewer->removeAllPointClouds();
    
    // 将所有复选框置为 false
    for(auto* chk : m_layerChecks) {
        // 暂时断开信号，防止触发 onLayerToggle 报错
        chk->blockSignals(true);
        chk->setChecked(false);
        chk->blockSignals(false);
    }
}


// [新增] 辅助函数：后续算法调用时，不能直接 edit->text()，因为那只是文件名
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

    // 添加一条正在加载的日志
    log(QString("正在加载文件: %1 ...").arg(filePath), "INFO");

    PointCloudT::Ptr cloud(new PointCloudT);
    if (pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud) == -1) {
        log("无法读取文件: " + filePath, "ERROR"); // 输出错误日志
        QMessageBox::warning(this, "加载失败", "无法读取点云文件:\n" + filePath);
        return;
    }

    m_cloudData[key] = cloud;
    
    // 输出成功日志
    log(QString("加载成功 [%1]: 点数 %2").arg(key).arg(cloud->size()), "SUCCESS");

    if (m_layerChecks.contains(key)) {
        m_layerChecks[key]->setChecked(true); 
    }
}

void SingleModePage::onLayerToggle(const QString& layerId, bool checked) {
    if (!m_viewer) return;
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
    onRunPreprocess(); 
}

void SingleModePage::initDefaultMatrices() {
    // 根据你的 pc_register.cpp 中的数据硬编码默认值
    // LB -> Top
    Eigen::Matrix4f lb;
    lb << 0.998144, 0.040452, 0.045531, 43.625172,
          0.049100, -0.092113, -0.994537, 1393.481567,
          -0.036037, 0.994927, -0.093928, 2062.417725,
          0, 0, 0, 1;
    m_transforms["LB"] = lb;

    // LT -> Top
    Eigen::Matrix4f lt;
    lt << 0.991525, 0.015493, 0.128988, -49.530918,
          0.125268, 0.149176, -0.980844, 1446.867676,
          -0.034438, 0.988689, 0.145971, 1454.759033,
          0, 0, 0, 1;
    m_transforms["LT"] = lt;

    // RB -> Top
    Eigen::Matrix4f rb;
    rb << -0.997376, 0.071718, 0.009833, 9.360316,
          0.006674, -0.044152, 0.999003, -1420.248047,
          0.072081, 0.996447, 0.043558, 1944.664185,
          0, 0, 0, 1;
    m_transforms["RB"] = rb;

    // RT -> Top
    Eigen::Matrix4f rt;
    rt << -0.993564, 0.102217, 0.048803, 11.873594,
          0.016009, -0.299805, 0.953866, -1409.841309,
          0.112133, 0.948509, 0.296239, 1245.285522,
          0, 0, 0, 1;
    m_transforms["RT"] = rt;
}


QString SingleModePage::matrixToString(const Eigen::Matrix4f& mat) {
    QString str;
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            // 保留4位小数，右对齐
            str += QString::number(mat(i,j), 'f', 4); 
            if(j < 3) str += "  ";
        }
        if(i < 3) str += "\n";
    }
    return str;
}

Eigen::Matrix4f SingleModePage::stringToMatrix(const QString& text) {
    Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
    QStringList tokens = text.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    if(tokens.size() == 16) {
        int idx = 0;
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                mat(i,j) = tokens[idx++].toFloat();
            }
        }
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
    log("启动配准与融合流程...", "ALGO");

    // 1. 获取选定的参考目标 (Target)
    // m_comboRegTarget 是 UI 下拉框，获取用户选的是 "Top" 还是 "LT" 等
    QString targetKey = m_comboRegTarget->currentText();

    // 检查目标点云是否存在
    if (!m_cloudData.contains(targetKey)) {
        QMessageBox::warning(this, "错误", "未找到参考目标点云: " + targetKey + "\n请先加载该文件。");
        return;
    }
    PointCloudT::Ptr cloudTarget = m_cloudData[targetKey];

    // 获取目标相对于 Top 的矩阵 (T_target_to_top)
    // 如果目标就是 Top，矩阵就是单位阵；否则从 map 里取
    Eigen::Matrix4f matTargetToTop = Eigen::Matrix4f::Identity();
    if (targetKey != "Top") {
        if (!m_transforms.contains(targetKey)) {
             QMessageBox::warning(this, "错误", "参考目标 " + targetKey + " 尚未初始化矩阵。");
             return;
        }
        matTargetToTop = m_transforms[targetKey];
    }

    bool useICP = m_comboRegMethod->currentText().contains("ICP");
    
    // [修复 1] 变量定义只保留这一次
    int processedCount = 0; 
    
    // =========================================================
    // 2. 初始化输出容器
    // =========================================================
    
    // 容器 A: 用于算法后续处理的纯几何点云 (不带颜色)
    PointCloudT::Ptr geometryMerged(new PointCloudT);
    
    // 容器 B: 用于 3D 窗口显示的彩色点云 (m_mergedCloudRGB 是成员变量)
    if (!m_mergedCloudRGB) {
        m_mergedCloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }
    m_mergedCloudRGB->clear(); // 清空旧数据，准备重新拼接

    // =========================================================
    // 3. 定义 Lambda: 同时向两个容器添加点
    // =========================================================
    auto appendColoredCloud = [&](PointCloudT::Ptr inputCloud, const QString& camName) {
        if (!inputCloud) return;
        
        int r, g, b;
        getCameraColor(camName, r, g, b); // 获取该相机的专属颜色

        // 遍历输入点云的所有点
        for (const auto& pt : inputCloud->points) {
            // 1. 添加到几何云 (用于保存/提取主体)
            geometryMerged->points.push_back(pt);

            // 2. 添加到彩色云 (用于显示，带 RGB)
            pcl::PointXYZRGB ptRGB;
            ptRGB.x = pt.x; ptRGB.y = pt.y; ptRGB.z = pt.z;
            ptRGB.r = r;    ptRGB.g = g;    ptRGB.b = b;
            m_mergedCloudRGB->points.push_back(ptRGB);
        }
    };

    // --- 第一步：先加入 Target (基准) ---
    // Target 作为坐标原点，不需要变换，直接加进去
    appendColoredCloud(cloudTarget, targetKey);
    
    // --- 第二步：遍历 Sources (待配准源) ---
    for (auto it = m_sourceChecks.begin(); it != m_sourceChecks.end(); ++it) {
        QString srcKey = it.key(); 
        QCheckBox* chk = it.value();

        // 跳过没勾选的、没数据的、或者源就是目标自己的
        if (!chk->isChecked() || !m_cloudData.contains(srcKey) || srcKey == targetKey) {
            continue; 
        }

        PointCloudT::Ptr cloudSrc = m_cloudData[srcKey];
        
        // ---------------------------------------------------------
        // 数学计算：计算 T_src_to_target
        // ---------------------------------------------------------
        Eigen::Matrix4f matSrcToTop_Current = m_transforms[srcKey];
        // 猜测矩阵 = Target逆 * Source当前
        Eigen::Matrix4f matSrcToTarget_Guess = matTargetToTop.inverse() * matSrcToTop_Current;

        Eigen::Matrix4f matSrcToTarget_Final = matSrcToTarget_Guess;
        PointCloudT::Ptr cloudAlignedLocal; // 变换到 Target 坐标系的点云

        if (useICP) {
            // 对 Target 进行 ICP
            auto result = PointCloudAlgo::alignICP(cloudSrc, cloudTarget, matSrcToTarget_Guess, 30, 0.05);
            cloudAlignedLocal = result.first;
            matSrcToTarget_Final = result.second;
            log(QString("ICP [%1 -> %2] 收敛").arg(srcKey).arg(targetKey), "ALGO");
        } else {
            // 手动模式，直接应用矩阵
            cloudAlignedLocal = PointCloudAlgo::transformCloud(cloudSrc, matSrcToTarget_Guess);
        }

        // ---------------------------------------------------------
        // 结果回写：更新全局矩阵 T_src_to_top
        // T_src_to_top = T_target_to_top * T_src_to_target
        // ---------------------------------------------------------
        Eigen::Matrix4f matSrcToTop_New = matTargetToTop * matSrcToTarget_Final;
        m_transforms[srcKey] = matSrcToTop_New; // 更新内存中的位置记录
        
        // [关键] 将变换后的点云，涂上 srcKey 的颜色，加入融合云
        if (cloudAlignedLocal) {
            appendColoredCloud(cloudAlignedLocal, srcKey);
            processedCount++;
        }
    }

    // 检查是否只添加了 Target 自己，没有配准其他任何东西
    if (processedCount == 0 && targetKey == "Top") {
        log("警告：只显示了 Target，未配准任何源点云 (请勾选右侧源)", "WARN");
        // 注意：这里不 return，依然显示 Target 也是可以的
    }

    // =========================================================
    // 4. 结果存储与显示
    // =========================================================
    
    // 设置几何点云属性 (使其 Dense，防止后续算法报错)
    geometryMerged->width = geometryMerged->size();
    geometryMerged->height = 1;
    geometryMerged->is_dense = true;
    
    // 将无色的几何数据存入 m_cloudData["Merged"]，用于后续保存文件或提取主体
    m_cloudData["Merged"] = geometryMerged; 

    // 设置彩色点云属性
    m_mergedCloudRGB->width = m_mergedCloudRGB->size();
    m_mergedCloudRGB->height = 1;
    m_mergedCloudRGB->is_dense = true;

    // 5. 强制勾选 "Merged" 图层并触发显示
    if (m_layerChecks.contains("Merged")) {
        // 先设为 true
        m_layerChecks["Merged"]->setChecked(true);
        // 手动调用一次 onLayerToggle 确保视图刷新 (传入 true)
        // 这个函数里会判断如果 layerId 是 "Merged"，就优先用 m_mergedCloudRGB 显示
        onLayerToggle("Merged", true); 
    }

    log(QString("融合完成。总点数: %1 (已保留各视角颜色)").arg(m_mergedCloudRGB->size()), "SUCCESS");
    
    // 如果当前矩阵编辑器正在显示某个被更新的相机，刷新一下文本框
    if (m_transforms.contains(m_comboMatrixView->currentText())) {
        onMatrixTargetChanged(0); 
    }
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