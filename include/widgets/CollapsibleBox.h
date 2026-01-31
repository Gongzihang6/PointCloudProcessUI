#pragma once
#include <QWidget>
#include <QToolButton>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QParallelAnimationGroup>

class CollapsibleBox : public QWidget {
    Q_OBJECT
public:
    explicit CollapsibleBox(const QString &title = "", QWidget *parent = nullptr);
    void setContentLayout(QLayout *layout);

private slots:
    void onToggle(bool checked);

private:
    QToolButton *toggleButton;
    QScrollArea *contentArea;
    QParallelAnimationGroup *toggleAnimation;
    int contentHeight = 0;
};
