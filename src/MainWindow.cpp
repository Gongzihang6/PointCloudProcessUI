#include "MainWindow.h"
#include "pages/SingleModePage.h"
#include "pages/BatchModePage.h"
#include <QHBoxLayout>
#include <QToolButton>
#include <QVBoxLayout>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    resize(1280, 800);
    setWindowTitle("Pig3D Metrology System (C++ Native)");

    QWidget *centralWidget = new QWidget(this);
    setCentralWidget(centralWidget);

    auto *mainLayout = new QHBoxLayout(centralWidget);
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // 初始化侧边栏
    initSidebar();
    mainLayout->addWidget(sidebar);

    // 初始化堆叠页面
    mainStack = new QStackedWidget();
    mainStack->addWidget(new SingleModePage()); // Index 0
    mainStack->addWidget(new BatchModePage());  // Index 1
    
    mainLayout->addWidget(mainStack);
}

void MainWindow::initSidebar() {
    sidebar = new QWidget(this);
    sidebar->setFixedWidth(68);
    sidebar->setStyleSheet("background-color: #2c3e50;");

    auto *layout = new QVBoxLayout(sidebar);
    layout->setContentsMargins(10, 20, 10, 20);
    layout->setSpacing(20);

    navGroup = new QButtonGroup(this);
    navGroup->setExclusive(true);

    // 单体模式按钮
    QToolButton *btnSingle = new QToolButton();
    btnSingle->setText("🖥️"); // 使用 Emoji 代替图标文件
    btnSingle->setObjectName("NavBtn");
    btnSingle->setCheckable(true);
    btnSingle->setChecked(true);
    navGroup->addButton(btnSingle, 0);
    layout->addWidget(btnSingle);

    // 批处理模式按钮
    QToolButton *btnBatch = new QToolButton();
    btnBatch->setText("📚");
    btnBatch->setObjectName("NavBtn");
    btnBatch->setCheckable(true);
    navGroup->addButton(btnBatch, 1);
    layout->addWidget(btnBatch);

    layout->addStretch(); // 底部顶起

    connect(navGroup, &QButtonGroup::idClicked, this, &MainWindow::switchPage);
}

void MainWindow::switchPage(int id) {
    mainStack->setCurrentIndex(id);
}
