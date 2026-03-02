#include "widgets/CollapsibleBox.h"
#include <QPropertyAnimation>
#include <QLayout> // 必须包含
#include <QDebug>

CollapsibleBox::CollapsibleBox(const QString &title, QWidget *parent) : QWidget(parent) {
    toggleButton = new QToolButton(this);
    toggleButton->setText(title);
    toggleButton->setCheckable(true);
    toggleButton->setChecked(true);
    // 样式保持不变
    toggleButton->setStyleSheet("QToolButton { border: none; background: #fff; text-align: left; font-weight: bold; padding: 10px; border-bottom: 1px solid #eee; } QToolButton:hover { background: #f9f9f9; }");
    toggleButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toggleButton->setArrowType(Qt::DownArrow);
    toggleButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    contentArea = new QScrollArea(this);
    contentArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    contentArea->setMaximumHeight(0); 
    contentArea->setFrameShape(QFrame::NoFrame);
    // [关键] 关闭水平滚动条，防止意外出现
    contentArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff); 
    // [关键] 垂直滚动条按需显示（如果高度计算正确，它就不会出现）
    contentArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);

    toggleAnimation = new QParallelAnimationGroup(this);
    auto *animation = new QPropertyAnimation(contentArea, "maximumHeight");
    animation->setDuration(300);
    toggleAnimation->addAnimation(animation);

    auto *mainLayout = new QVBoxLayout(this);
    mainLayout->setSpacing(0);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->addWidget(toggleButton);
    mainLayout->addWidget(contentArea);

    connect(toggleButton, &QToolButton::toggled, this, &CollapsibleBox::onToggle);
}

void CollapsibleBox::setContentLayout(QLayout *layout) {
    delete contentArea->layout();
    QWidget *contentWidget = new QWidget();
    contentWidget->setLayout(layout);
    
    // [修改 A]：只设置背景色，去掉 QSS 中的 padding
    // 因为 QSS 的 padding 经常导致 sizeHint 计算偏小
    contentWidget->setStyleSheet("background-color: #fff;"); 
    
    // [修改 B]：使用布局系统的 margin 来代替 padding
    // 这样 Qt 能完美计算出高度
    layout->setContentsMargins(10, 10, 10, 10); 

    contentArea->setWidget(contentWidget);
    contentArea->setWidgetResizable(true);
    
    // 初始化时触发一次打开
    onToggle(true);
}

void CollapsibleBox::onToggle(bool checked) {
    toggleButton->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
    toggleAnimation->setDirection(checked ? QAbstractAnimation::Forward : QAbstractAnimation::Backward);
    
    auto *animation = static_cast<QPropertyAnimation *>(toggleAnimation->animationAt(0));
    
    if (checked) {
        // [修改 C]：每次展开前，重新计算高度！
        // 这一步至关重要，它确保了如果内容变了，盒子也能正确展开
        if(contentArea->widget()) {
            contentArea->widget()->adjustSize(); // 强制重新计算布局
            
            // 获取最新高度
            int h = contentArea->widget()->sizeHint().height();
            
            // [修改 D]：增加 10px 的安全缓冲
            // 防止因为 1 像素的误差导致出现滚动条
            contentHeight = h + 40; 
        }
        animation->setEndValue(contentHeight);
    } else {
        animation->setEndValue(0);
    }
    
    animation->setStartValue(contentArea->maximumHeight()); // 从当前高度开始，保证流畅
    toggleAnimation->start();
}