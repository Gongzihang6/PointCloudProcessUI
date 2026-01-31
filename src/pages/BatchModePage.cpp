#include "pages/BatchModePage.h"
#include <QVBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QProgressBar>
#include <QTextEdit>

BatchModePage::BatchModePage(QWidget *parent) : QWidget(parent) {
    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(40, 40, 40, 40); // 增加内边距，类似 Dashboard

    // 卡片1：配置
    QWidget *configCard = new QWidget();
    configCard->setObjectName("Panel"); // 应用边框样式
    configCard->setStyleSheet("#Panel { background: #fff; border: 1px solid #dcdfe6; border-radius: 8px; }");
    
    auto *cardLayout = new QVBoxLayout(configCard);
    QLabel *title = new QLabel("📁 批处理配置");
    title->setStyleSheet("font-size: 16px; font-weight: bold; margin-bottom: 10px;");
    cardLayout->addWidget(title);

    auto *row1 = new QHBoxLayout();
    row1->addWidget(new QLabel("输入目录:"));
    row1->addWidget(new QLineEdit("F:/Data/2026/Input"));
    row1->addWidget(new QPushButton("选择..."));
    cardLayout->addLayout(row1);

    auto *row2 = new QHBoxLayout();
    row2->addWidget(new QLabel("输出目录:"));
    row2->addWidget(new QLineEdit("F:/Data/2026/Output"));
    row2->addWidget(new QPushButton("选择..."));
    cardLayout->addLayout(row2);

    QPushButton *startBtn = new QPushButton("🚀 开始批处理");
    startBtn->setObjectName("PrimaryBtn");
    startBtn->setFixedHeight(40);
    cardLayout->addWidget(startBtn);

    mainLayout->addWidget(configCard);

    // 卡片2：日志与进度
    QWidget *logCard = new QWidget();
    logCard->setObjectName("Panel");
    logCard->setStyleSheet("#Panel { background: #fff; border: 1px solid #dcdfe6; border-radius: 8px; }");
    
    auto *logLayout = new QVBoxLayout(logCard);
    logLayout->addWidget(new QLabel("📊 运行状态"));
    
    QProgressBar *progress = new QProgressBar();
    progress->setValue(45);
    logLayout->addWidget(progress);

    QTextEdit *logTerminal = new QTextEdit();
    logTerminal->setReadOnly(true);
    logTerminal->setStyleSheet("background: #23272e; color: #abb2bf; font-family: Consolas;");
    logTerminal->setText("[INFO] System Ready.\n[INFO] Loading files...");
    logLayout->addWidget(logTerminal);

    mainLayout->addWidget(logCard);
}
