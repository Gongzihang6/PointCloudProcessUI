#pragma once
#include <QMainWindow>      // 提供了一个标准的应用程序主窗口框架（包含菜单栏、工具栏、状态栏和中心区域）
#include <QStackedWidget>   // QStackedWidget 是一个容器类，可以在同一位置堆叠多个子窗口，并通过 setCurrentIndex() 方法切换显示哪个子窗口，是实现“侧边栏切换页面”功能的关键
#include <QButtonGroup>     // QButtonGroup 用于将多个按钮（如侧边栏的导航按钮）分组管理，方便实现互斥选择（即一次只能选择一个按钮），并且可以通过信号槽机制轻松连接到切换页面的功能

/**
 * MainWindow 类继承自 QMainWindow，代表应用程序的主窗口。
 * 在 Qt 中，绝大多数自定义窗口都是继承自某个 Qt 基类。
 * 这意味着你的窗口不仅仅是一个 C++ 类，它直接继承了 Qt 庞大的窗口系统能力（如绘制、事件处理、窗口句柄管理）。
 */
class MainWindow : public QMainWindow {
    Q_OBJECT
public:
    MainWindow(QWidget *parent = nullptr);

private slots:      // 下面的函数不仅是普通的 C++ 成员函数，还是一个可以在运行时被动态调用的‘槽’
    void switchPage(int id);

private:
    // 一个辅助函数，用于将繁杂的 UI 初始化代码（如创建按钮、设置布局、美化样式）从构造函数中分离出来，保持代码整洁。
    void initSidebar();
    
    QWidget *sidebar;
    QStackedWidget *mainStack;
    QButtonGroup *navGroup;
};
