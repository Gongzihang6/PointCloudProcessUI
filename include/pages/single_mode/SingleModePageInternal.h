#pragma once

#include <windows.h>

#include "pages/SingleModePage.h"
#include "widgets/CollapsibleBox.h"

#include <QButtonGroup>
#include <QCheckBox>
#include <QColor>
#include <QComboBox>
#include <QDebug>
#include <QDialog>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QFuture>
#include <QFutureWatcher>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QHBoxLayout>
#include <QHttpMultiPart>
#include <QHttpPart>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QNetworkReply>
#include <QPixmap>
#include <QPushButton>
#include <QRegularExpression>
#include <QResizeEvent>
#include <QScrollArea>
#include <QScrollBar>
#include <QSignalBlocker>
#include <QSpinBox>
#include <QStackedWidget>
#include <QSplitter>
#include <QStyle>
#include <QTableWidget>
#include <QTableWidgetItem>
#include <QTemporaryFile>
#include <QTextEdit>
#include <QUrl>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>

#include <QtConcurrent>

#include <vtkCamera.h>
#include <vtkCellPicker.h>
#include <vtkCommand.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/io/pcd_io.h>

#include <functional>
#include <string>
#include <utility>
#include <vector>

struct RegTaskInput {
    QString srcKey;
    QString targetKey;
    PointCloudT::Ptr cloudSrc;
    PointCloudT::Ptr cloudTarget;
    Eigen::Matrix4d initialGuess;
    int methodIndex = 0;
    int algoType = 0;
    int icpIter = 0;
    double icpDist = 0.0;
    float ndtRes = 0.0f;
    float ndtStep = 0.0f;
    int ndtIter = 0;
    int gicpIter = 0;
    double gicpDist = 0.0;
    double gicpEps = 0.0;
};

struct RegTaskOutput {
    QString srcKey;
    PointCloudT::Ptr cloudAlignedLocal;
    Eigen::Matrix4d finalTransform = Eigen::Matrix4d::Identity();
    std::vector<std::pair<QString, QString>> logs;
    bool valid = false;
};

RegTaskOutput processRegistrationWorker(const RegTaskInput& input);
