#pragma once
#include <QMainWindow>
#include <QStackedWidget>
#include <QButtonGroup>

class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);

private slots:
    void switchPage(int id);

private:
    void initSidebar();
    
    QWidget *sidebar;
    QStackedWidget *mainStack;
    QButtonGroup *navGroup;
};
