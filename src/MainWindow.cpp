#include "MainWindow.h"
#include "pages/SingleModePage.h"
#include "pages/BatchModePage.h"
#include <QHBoxLayout>      // 水平布局，让控件从左到右排列
#include <QToolButton>
#include <QVBoxLayout>      // 垂直布局，让控件从上到下排列

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    resize(1280, 800);
    setWindowTitle("Pig3D Metrology System (C++ Native)");      // 设置顶部窗口标题内容

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    auto *mainLayout = new QHBoxLayout(centralWidget);  // 把 centralWidget 里面的子控件按水平方向排列
    mainLayout->setSpacing(0);  // 设置控件之间的间隙为 0
    mainLayout->setContentsMargins(0, 0, 0, 0);     // 设置四周留白为 0。让界面内容撑满整个窗口

    // 初始化侧边栏
    initSidebar();
    mainLayout->addWidget(sidebar);     // 先加入侧边栏（放在左边）

    // 初始化堆叠页面
    mainStack = new QStackedWidget();   // 堆叠窗口，可以在其中放多个页面（Widget），但一次只显示一个
    mainStack->addWidget(new SingleModePage()); // Index 0，单体模式
    mainStack->addWidget(new BatchModePage());  // Index 1，批量模式
    
    mainLayout->addWidget(mainStack);   // 后加入内容栈（放在右边）
}


// 绘制左侧边栏，包含导航按钮（单体模式和批处理模式）
void MainWindow::initSidebar() {
    sidebar = new QWidget(this);
    sidebar->setFixedWidth(68);     // 强制宽度为 68px，高度随窗口变化
    sidebar->setStyleSheet("background-color: #2c3e50;");   // 设置左侧边栏背景颜色

    auto *layout = new QVBoxLayout(sidebar);    // 设置侧边栏内部，按钮从上到下排列
    layout->setContentsMargins(10, 20, 10, 20);     // 设置内部边距，让按钮不要贴着边框
    layout->setSpacing(20);

    navGroup = new QButtonGroup(this);
    navGroup->setExclusive(true);   // 开启互斥模式，保证同一时间只能选中一个按钮

    // 单体模式按钮
    QToolButton *btnSingle = new QToolButton();
    btnSingle->setText("🖥️"); // 使用 Emoji 代替图标文件
    btnSingle->setObjectName("NavBtn");     // 给对象起个“身份证号”，用于在 QSS 样式表中通过 ID 选择器（如 #NavBtn）来单独设置这些按钮的样式
    btnSingle->setCheckable(true);      // 让按钮具有“按下/弹起”两种状态
    btnSingle->setChecked(true);        // 默认选中第一个按钮
    navGroup->addButton(btnSingle, 0);  // 将按钮加入逻辑组，并分配 ID 0
    layout->addWidget(btnSingle);

    // 批处理模式按钮
    QToolButton *btnBatch = new QToolButton();
    btnBatch->setText("📚");
    btnBatch->setObjectName("NavBtn");
    btnBatch->setCheckable(true);
    navGroup->addButton(btnBatch, 1);
    layout->addWidget(btnBatch);

    layout->addStretch(); // 底部顶起

    /**
     * 连接信号和槽：当 navGroup 中的按钮被点击时，发出 idClicked(int) 信号，参数是被点击按钮的 ID（0 或 1）。
     * 这个信号连接到 MainWindow 的 switchPage(int) 槽函数，槽函数会根据 ID 切换 mainStack 中显示的页面。
     */
    connect(navGroup, &QButtonGroup::idClicked, this, &MainWindow::switchPage);
}

// 切换页面的槽函数，根据按钮 ID 切换 mainStack 中显示的页面
void MainWindow::switchPage(int id) {
    mainStack->setCurrentIndex(id);
}
