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
    auto *layout = new QVBoxLayout(centerPanel);
    layout->setContentsMargins(0, 0, 0, 0);

    // 1. 创建一个普通的 QWidget 作为容器
    m_vtkContainer = new QWidget(centerPanel);
    
    // [关键] 设置属性，告诉 Qt 这个窗口的内容由外部（VTK）绘制，Qt 别管
    m_vtkContainer->setAttribute(Qt::WA_OpaquePaintEvent);
    m_vtkContainer->setAttribute(Qt::WA_PaintOnScreen);
    m_vtkContainer->setAttribute(Qt::WA_NativeWindow); // 确保它有独立的 HWND
    
    layout->addWidget(m_vtkContainer);

    // 2. 手动创建 VTK 渲染窗口
    vtkSmartPointer<vtkRenderWindow> renWin = vtkSmartPointer<vtkRenderWindow>::New();
    
    // [核心黑科技] 将 VTK 窗口的父句柄设置为我们的 Qt Widget 句柄
    // 这样 VTK 就会画在 Qt Widget 里面了
    renWin->SetParentId(reinterpret_cast<void*>(m_vtkContainer->winId()));

        // 3. 初始化 PCL Visualizer
    // [修复] 手动创建 Renderer 并绑定
    vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
    renWin->AddRenderer(renderer);

    // [修复] 传入 (Renderer, RenderWindow, Name, CreateInteractor)
    m_viewer.reset(new pcl::visualization::PCLVisualizer(renderer, renWin, "viewer", true));
    // [修复] 通过 getRenderWindow() 间接获取 Interactor
    if (m_viewer->getRenderWindow()->GetInteractor()) {
        m_viewer->getRenderWindow()->GetInteractor()->Initialize();
    }

    m_viewer->setBackgroundColor(0.1, 0.1, 0.1);
    m_viewer->addCoordinateSystem(100.0);
    m_viewer->initCameraParameters();

    // 4. [新增] 启动定时器刷新视图 (因为没有 QVTKWidget 帮我们 spin 了)
    m_refreshTimer = new QTimer(this);
    connect(m_refreshTimer, &QTimer::timeout, [this](){
        if(m_viewer) {
            // 相当于 PCL 的 spinOnce，每 30ms 渲染一帧
            m_viewer->spinOnce(1, true); 
        }
    });
    m_refreshTimer->start(30); // 30ms ≈ 33 FPS
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
    m_spinStdDev->setValue(1.0);       // 默认 1.0
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
    auto *box2 = new CollapsibleBox("2. 配准与融合 (Registration)");
    auto *lay2 = new QVBoxLayout();
    lay2->setSpacing(8);

    auto *rowAlgo = new QHBoxLayout();
    rowAlgo->addWidget(new QLabel("算法:"));
    QComboBox *comboAlgo = new QComboBox();
    comboAlgo->addItems({"手动矩阵 (Manual)", "ICP (P2Plane)", "NDT"});
    comboAlgo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    rowAlgo->addWidget(comboAlgo);
    lay2->addLayout(rowAlgo);

    // 矩阵编辑区容器
    QFrame *matrixFrame = new QFrame();
    matrixFrame->setStyleSheet("background: #f8f9fa; border: 1px solid #e4e7ed; border-radius: 4px; padding: 5px;");
    QVBoxLayout *matrixLay = new QVBoxLayout(matrixFrame);
    matrixLay->setContentsMargins(5,5,5,5);
    
    auto *rowMatTarget = new QHBoxLayout();
    QLabel *lblMat = new QLabel("编辑目标变换:"); lblMat->setStyleSheet("font-size: 11px; color: #666;");
    QComboBox *comboMat = new QComboBox();
    comboMat->setStyleSheet("font-size: 11px; height: 20px;");
    comboMat->addItems({"LB -> Top", "LT -> Top", "RB -> Top", "RT -> Top"});
    rowMatTarget->addWidget(lblMat);
    rowMatTarget->addWidget(comboMat);
    matrixLay->addLayout(rowMatTarget);

    QTextEdit *txtMat = new QTextEdit();
    txtMat->setObjectName("MatrixEditor"); // 使用 QSS 样式
    txtMat->setText("0.998  0.040  0.045  43.62\n0.049 -0.092 -0.994 1393.48\n-0.036 0.994 -0.093 2062.41\n0.000  0.000  0.000  1.000");
    txtMat->setFixedHeight(65);
    matrixLay->addWidget(txtMat);

    lay2->addWidget(matrixFrame);

    QPushButton *btnReg = new QPushButton("🚀 执行配准与融合");
    btnReg->setObjectName("PrimaryBtn");
    lay2->addWidget(btnReg);

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
    // [修改] 过滤器只看 pcd 文件
    QStringList filters; filters << "*.pcd";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    // 映射关系
    QMap<QString, QString> keyMap;
    keyMap["005J"] = "Top";
    keyMap["00SE"] = "LT"; 
    keyMap["003W"] = "LB"; 
    keyMap["00YA"] = "RB";
    keyMap["00X6"] = "RT";

    // 先执行一次清空
    onClearFiles();

    int matchCount = 0;
    
    // [新增] 必须包含的后缀
    const QString targetSuffix = "_d_pc.pcd";

    for (const QFileInfo& fileInfo : fileList) {
        QString fileName = fileInfo.fileName();

        // [优化 3] 严格判断后缀
        if (!fileName.endsWith(targetSuffix, Qt::CaseInsensitive)) {
            continue; // 如果不是 _d_pc.pcd 结尾，直接跳过（忽略彩色点云）
        }
        
        // 遍历规则进行匹配
        for (auto it = keyMap.begin(); it != keyMap.end(); ++it) {
            QString idCode = it.key();   
            QString targetSlot = it.value(); 

            // 如果文件名包含ID (且满足上面的后缀条件)
            if (fileName.contains(idCode, Qt::CaseInsensitive)) {
                if (m_fileInputs.contains(targetSlot)) {
                    QLineEdit* edit = m_fileInputs[targetSlot];

                    // [优化 2] 界面只显示文件名
                    edit->setText(fileName);
                    
                    // [优化 2] 存储完整路径
                    edit->setProperty("fullPath", fileInfo.absoluteFilePath());
                    edit->setToolTip(fileInfo.absoluteFilePath());
                    edit->setCursorPosition(0);

                    matchCount++;
                    if (m_cloudData.contains(targetSlot)) m_cloudData.remove(targetSlot);
                }
                break; 
            }
        }
    }

    qDebug() << "自动匹配了" << matchCount << "个深度点云文件";
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

    PointCloudT::Ptr cloud(new PointCloudT);
    
    // 使用 PCL 读取 PCD 文件
    if (pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud) == -1) {
        QMessageBox::warning(this, "加载失败", "无法读取点云文件:\n" + filePath);
        return;
    }

    // 存入内存 Map
    m_cloudData[key] = cloud;
    qDebug() << "已加载点云到内存:" << key << "点数:" << cloud->size();

    // 加载成功后，自动勾选对应的复选框，触发显示
    if (m_layerChecks.contains(key)) {
        // 这会触发 onLayerToggle -> update3DView
        m_layerChecks[key]->setChecked(true); 
    }
}

void SingleModePage::onLayerToggle(const QString& layerId, bool checked) {
    if (!m_viewer) return;

    // 对应的点云 ID (在 PCL Viewer 内部使用的 ID)
    std::string cloudId = layerId.toStdString();

    if (checked) {
        // === 想要显示 ===
        
        // 1. 检查内存里有没有数据
        if (!m_cloudData.contains(layerId) || m_cloudData[layerId]->empty()) {
            // 如果内存没数据，尝试看看输入框里有没有路径，现加载一下
            QString path = getFullPath(layerId);
            if (!path.isEmpty()) {
                // 加载并递归调用一次（因为 loadCloudToMemory 会 setChecked(true)）
                // 为了防止死循环，这里只加载数据，不设 Checkbox
                PointCloudT::Ptr cloud(new PointCloudT);
                if (pcl::io::loadPCDFile<PointT>(path.toStdString(), *cloud) == 0) {
                    m_cloudData[layerId] = cloud;
                }
            }
        }

        // 2. 如果现在有数据了，添加显示
        if (m_cloudData.contains(layerId)) {
            // 先移除旧的防止重名报错
            m_viewer->removePointCloud(cloudId);
            
            // 添加点云
            pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(m_cloudData[layerId], 255, 255, 255);
            m_viewer->addPointCloud(m_cloudData[layerId], colorHandler, cloudId);
            
            // 设置颜色 (根据 ID 预设颜色)
            double r=1.0, g=1.0, b=1.0;
            if (layerId == "Top") { r=1.0; g=0.0; b=0.0; }      // 红
            else if (layerId == "LB") { r=0.0; g=1.0; b=0.0; } // 绿
            else if (layerId == "LT") { r=0.0; g=0.0; b=1.0; } // 蓝
            else if (layerId == "RB") { r=1.0; g=0.84; b=0.0; }// 金
            else if (layerId == "RT") { r=0.0; g=1.0; b=1.0; } // 青
            
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudId);
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
    } else {
        // === 想要隐藏 ===
        m_viewer->removePointCloud(cloudId);
    }

    // 刷新窗口
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
    // 1. 获取所有参数
    float leaf_mm = m_spinLeafSize->value();
    double std_dev = m_spinStdDev->value();
    int mean_k = m_spinMeanK->value();
    float clip_radius_mm = m_spinClipRadius->value();

    // 2. 遍历所有可见点云
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        QString key = it.key();
        
        // 只处理勾选显示的层，节省资源
        if (!m_layerChecks.contains(key) || !m_layerChecks[key]->isChecked()) {
            continue; 
        }

        PointCloudT::Ptr currentCloud = it.value(); // 从原始数据开始

        // --- Step 1: 下采样 ---
        // (先做下采样可以大大显著减少后续 SOR 的计算量)
        currentCloud = PointCloudAlgo::downsample(currentCloud, leaf_mm);
        if (!currentCloud) continue;

        // --- Step 2: 统计滤波 (SOR) ---
        currentCloud = PointCloudAlgo::statisticalOutlierRemoval(currentCloud, mean_k, std_dev);
        if (!currentCloud) continue;

        // --- Step 3: 半径裁剪 ---
        currentCloud = PointCloudAlgo::distanceClip(currentCloud, clip_radius_mm);
        if (!currentCloud) continue;

        // 3. 更新可视化
        std::string cloudId = key.toStdString();
        m_viewer->removePointCloud(cloudId);

        pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(currentCloud, 255, 255, 255);
        m_viewer->addPointCloud(currentCloud, colorHandler, cloudId);
        
        // 恢复颜色
        double r=1.0, g=1.0, b=1.0;
        if (key == "Top") { r=1.0; g=0.0; b=0.0; }
        else if (key == "LB") { r=0.0; g=1.0; b=0.0; }
        else if (key == "LT") { r=0.0; g=0.0; b=1.0; }
        else if (key == "RB") { r=1.0; g=0.84; b=0.0; }
        else if (key == "RT") { r=0.0; g=1.0; b=1.0; }
        
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudId);
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
    }

    // 4. 刷新视图
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
