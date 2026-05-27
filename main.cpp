#include <QApplication>     // 所有 Qt GUI 应用程序的核心类，负责管理 GUI 的控制流和主设置
#include <QFile>            // 用于处理文件的输入输出（I/O）
#include <QCoreApplication>
#include <QSurfaceFormat>
#include "MainWindow.h"
#include <QVTKOpenGLNativeWidget.h>
#include <vtkOutputWindow.h>    // VTK 是一个强大的开源 3D 计算机图形库，常与 Qt 结合使用。vtkOutputWindow 类用于管理 VTK 的调试和错误信息输出
int main(int argc, char *argv[]) {
    // 禁用 VTK 的弹出式错误窗口
    // 0 表示不显示，这样程序退出时的警告就会被静默忽略
    vtkOutputWindow::SetGlobalWarningDisplay(0);
    QCoreApplication::setAttribute(Qt::AA_ShareOpenGLContexts);
    QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());

    /**
     * QApplication 类是所有 Qt GUI 应用程序的核心类，是一个单例对象
     * 作用：
     *  初始化底层的 GUI 系统（如显示驱动、字体）
     *  准备事件循环 (Event Loop)，这是 Qt 程序能够响应鼠标点击、键盘输入的核心机制
     *  利用 argc 和 argv 解析命令行参数
     */
    QApplication app(argc, argv);

    // 加载样式表
    QFile styleFile("resources/app_style.qss");
    if(styleFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        app.setStyleSheet(styleFile.readAll());
        styleFile.close();
    }

    MainWindow w;
    w.show();

    return app.exec();      // 启动 Qt 的事件循环 (Event Loop)
}
