#include "widgets/CollapsibleBox.h"
#include <QPropertyAnimation>

CollapsibleBox::CollapsibleBox(const QString &title, QWidget *parent) : QWidget(parent) {
    toggleButton = new QToolButton(this);
    toggleButton->setText(title);
    toggleButton->setCheckable(true);
    toggleButton->setChecked(true); // 默认展开
    toggleButton->setStyleSheet("QToolButton { border: none; background: #fff; text-align: left; font-weight: bold; padding: 10px; border-bottom: 1px solid #eee; } QToolButton:hover { background: #f9f9f9; }");
    toggleButton->setToolButtonStyle(Qt::ToolButtonTextBesideIcon);
    toggleButton->setArrowType(Qt::DownArrow);
    toggleButton->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);

    contentArea = new QScrollArea(this);
    contentArea->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
    contentArea->setMaximumHeight(0); // 初始高度需动态计算
    contentArea->setFrameShape(QFrame::NoFrame);

    // 动画效果
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
    contentWidget->setStyleSheet("background-color: #fff; padding: 10px;");
    contentArea->setWidget(contentWidget);
    contentArea->setWidgetResizable(true);
    
    // 计算内容高度并展开
    contentWidget->adjustSize();
    contentHeight = contentWidget->sizeHint().height() + 20; 
    onToggle(true);
}

void CollapsibleBox::onToggle(bool checked) {
    toggleButton->setArrowType(checked ? Qt::DownArrow : Qt::RightArrow);
    toggleAnimation->setDirection(checked ? QAbstractAnimation::Forward : QAbstractAnimation::Backward);
    auto *animation = static_cast<QPropertyAnimation *>(toggleAnimation->animationAt(0));
    animation->setStartValue(0);
    animation->setEndValue(contentHeight);
    toggleAnimation->start();
}
