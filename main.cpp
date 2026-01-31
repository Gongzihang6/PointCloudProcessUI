#include <QApplication>
#include <QFile>
#include "MainWindow.h"
#include <vtkOutputWindow.h>
int main(int argc, char *argv[]) {
    // [新增] 禁用 VTK 的弹出式错误窗口
    // 0 表示不显示，这样程序退出时的警告就会被静默忽略
    vtkOutputWindow::SetGlobalWarningDisplay(0);

    
    QApplication app(argc, argv);

    // 加载样式表
    QFile styleFile("resources/app_style.qss");
    if(styleFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        app.setStyleSheet(styleFile.readAll());
        styleFile.close();
    }

    MainWindow w;
    w.show();

    return app.exec();
}
