/*
 * 文件说明：实现 `SingleModePage` 的数据装载、图层切换、日志与基础视图辅助逻辑。
 */
#include "pages/single_mode/SingleModePageInternal.h"

void SingleModePage::onBrowseFile(const QString& key) {
    QString fileName = QFileDialog::getOpenFileName(
        this, 
        "选择点云或深度图文件 (" + key + ")", 
        "", 
        "Point Cloud Files (*.pcd *.ply *.raw);;All Files (*)"
    );

    if (!fileName.isEmpty()) {
        if (m_fileInputs.contains(key)) {
            QLineEdit* edit = m_fileInputs[key];
            QFileInfo fileInfo(fileName);

            // 1. 界面只显示文件名
            edit->setText(fileInfo.fileName());
            
            // 2. 将完整路径存储在自定义属性 "fullPath" 中
            edit->setProperty("fullPath", fileInfo.absoluteFilePath());
            
            // 3. 设置鼠标悬停提示，方便用户查看完整路径
            edit->setToolTip(fileInfo.absoluteFilePath());
            
            // [UI反馈] 光标移到最前，防止长文件名看不见开头
            edit->setCursorPosition(0); 

            // 立即加载并显示这个点云
            loadCloudToMemory(key, fileName);
        }
    }
}


void SingleModePage::onLoadFolder() {
    QString dirPath = QFileDialog::getExistingDirectory(this, "选择包含点云数据的文件夹", "");
    if (dirPath.isEmpty()) return;

    QDir dir(dirPath);
    QStringList filters; filters << "*.pcd" << "*.raw";
    QFileInfoList fileList = dir.entryInfoList(filters, QDir::Files);

    QMap<QString, QString> keyMap;
    keyMap["005J"] = "Top";
    keyMap["00SE"] = "RB"; 
    keyMap["003W"] = "RT"; 
    keyMap["00YA"] = "LB";
    keyMap["00X6"] = "LT";

    // 1. 先清空
    onClearFiles();
    log("开始批量加载文件夹...", "INFO");

    int matchCount = 0;
    // 你的文件名后缀
    const QString targetSuffix = "_d_pc.pcd";

    for (const QFileInfo& fileInfo : fileList) {
        QString fileName = fileInfo.fileName();

        // 后缀检查
        if (!fileName.endsWith("_d_pc.pcd", Qt::CaseInsensitive) && 
                                    !fileName.endsWith("_depth_raw.raw", Qt::CaseInsensitive)) {
            continue; 
        }
        
        // 遍历 ID 匹配
        for (auto it = keyMap.begin(); it != keyMap.end(); ++it) {
            QString idCode = it.key();   
            QString targetSlot = it.value(); 

            if (fileName.contains(idCode, Qt::CaseInsensitive)) {
                if (m_fileInputs.contains(targetSlot)) {
                    QLineEdit* edit = m_fileInputs[targetSlot];

                    // --- UI 设置 ---
                    edit->setText(fileName);
                    edit->setProperty("fullPath", fileInfo.absoluteFilePath());
                    edit->setToolTip(fileInfo.absoluteFilePath());
                    edit->setCursorPosition(0);

                    // ==========================================
                    // [核心修复] 这里必须手动调用加载函数！
                    // 以前是靠 onLayerToggle 偷懒加载的，现在必须显式加载
                    // ==========================================
                    loadCloudToMemory(targetSlot, fileInfo.absoluteFilePath());

                    matchCount++;
                }
                break; 
            }
        }
    }

    if (matchCount > 0) {
        log(QString("批量加载完成，共加载 %1 个文件").arg(matchCount), "SUCCESS");
        // 自动复位相机，防止看不到
        if (m_viewer) m_viewer->resetCamera();
    } else {
        log("未匹配到有效文件", "WARN");
    }
}

// 清空功能实现
void SingleModePage::onClearFiles() {
    for (auto it = m_fileInputs.begin(); it != m_fileInputs.end(); ++it) {
        QLineEdit* edit = it.value();
        edit->clear();                    
        edit->setProperty("fullPath", ""); 
        edit->setToolTip("");             
    }
    // 清空内存数据和 3D 视图
    m_cloudData.clear();
    m_viewer->removeAllPointClouds();
    
    // =======================================================
    // [新增] 释放 OpenCV 图像矩阵内存
    // =======================================================
    m_topColorImage.release();
    m_topAlignedDepthImage.release();
    
    // 将所有复选框置为 false
    for(auto* chk : m_layerChecks) {
        chk->blockSignals(true);
        chk->setChecked(false);
        chk->blockSignals(false);
    }
}



// 辅助函数：后续算法调用时，不能直接 edit->text()，因为那只是文件名
// 需要调用这个函数来获取真实路径
QString SingleModePage::getFullPath(const QString& camKey) const {
    if (m_fileInputs.contains(camKey)) {
        // 优先读取存储的 "fullPath" 属性
        QString path = m_fileInputs[camKey]->property("fullPath").toString();
        
        // 如果属性为空（比如用户手动粘贴路径进框），则回退使用框内文字
        if (path.isEmpty()) {
            return m_fileInputs[camKey]->text();
        }
        return path;
    }
    return QString();
}

void SingleModePage::loadCloudToMemory(const QString& key, const QString& filePath) {
    if (filePath.isEmpty()) return;

    log(QString("正在加载文件: %1 ...").arg(filePath), "INFO");
    PointCloudT::Ptr cloud(new PointCloudT);

    // [核心修改] 拦截扩展名分支
    if (filePath.endsWith(".raw", Qt::CaseInsensitive)) {
        CameraIntrinsics intr = PointCloudAlgo::getCameraIntrinsics(key, SensorType::DEPTH, 512, 512);
        cloud = PointCloudAlgo::convertRawDepthToPointCloud(filePath, intr);

        // =======================================================
        // [新增] 如果是 Top 视角，顺便加载彩色图和对齐深度图
        // =======================================================
        if (key == "Top") {
            // 推导彩色图路径
            QString colorPath = filePath;
            colorPath.replace("_depth_raw.raw", "_rgb.png"); 
            
            // 加载彩色图
            m_topColorImage = cv::imread(colorPath.toStdString(), cv::IMREAD_COLOR);
            
            // 推导对齐深度图路径
            QString alignedDepthPath = filePath;
            alignedDepthPath.replace("_depth_raw.raw", "_depth_aligned.raw"); 

            // 加载 1280x720 的 16-bit 对齐深度图
            bool depthLoaded = false;
            QFile dFile(alignedDepthPath);
            if (dFile.open(QIODevice::ReadOnly)) {
                QByteArray dData = dFile.readAll();
                if (dData.size() == 1280 * 720 * 2) {
                    m_topAlignedDepthImage = cv::Mat(720, 1280, CV_16UC1, dData.data()).clone(); 
                    depthLoaded = true;
                }
                dFile.close();
            }

            // [核心修复] 极其严谨的状态校验日志
            if (!m_topColorImage.empty() && depthLoaded) {
                log("✅ 成功将 Top 视角 RGB 图像及对齐深度图装载入内存。", "SUCCESS");
            } else {
                if (m_topColorImage.empty()) {
                    log("❌ 警告：无法读取 Top 彩色图像！尝试路径: " + colorPath, "WARN");
                }
                if (!depthLoaded) {
                    log("❌ 警告：无法读取对齐深度图或分辨率不是 1280x720！尝试路径: " + alignedDepthPath, "WARN");
                }
            }
        }

        if (!cloud || cloud->empty()) {
            log("RAW 深度图转换失败！请检查文件完整性或确认内参分辨率(默认1280x720)。", "ERROR");
            QMessageBox::warning(this, "加载失败", "RAW 深度图解析失败，请检查内参分辨率。");
            return;
        }
        log(QString("RAW 深度图已成功转为点云 [%1]: 生成点数 %2").arg(key).arg(cloud->size()), "SUCCESS");
    } 
    else {
        // 原有 PCL 加载逻辑
        if (pcl::io::loadPCDFile<PointT>(filePath.toStdString(), *cloud) == -1) {
            log("无法读取 PCD 文件: " + filePath, "ERROR");
            QMessageBox::warning(this, "加载失败", "无法读取点云文件:\n" + filePath);
            return;
        }
        log(QString("加载 PCD 成功 [%1]: 点数 %2").arg(key).arg(cloud->size()), "SUCCESS");
    }

    // 存入内存
    m_cloudData[key] = cloud;
    Eigen::Vector4f minPt, maxPt;
    pcl::getMinMax3D(*cloud, minPt, maxPt);
    log(QString("Bounds [%1]: min=(%2,%3,%4), max=(%5,%6,%7)")
            .arg(key)
            .arg(minPt[0]).arg(minPt[1]).arg(minPt[2])
            .arg(maxPt[0]).arg(maxPt[1]).arg(maxPt[2]), "INFO");
    
    if (m_layerChecks.contains(key)) {
        m_layerChecks[key]->setChecked(true); 
    }

    if (m_vtkRenderer) {
        m_vtkRenderer->ResetCamera();
        m_vtkRenderer->ResetCameraClippingRange();
    }
    if (m_viewer && m_viewer->getRenderWindow()) {
        m_viewer->getRenderWindow()->Render();
    }
}


void SingleModePage::onLayerToggle(const QString& layerId, bool checked) {
    if (!m_viewer) return;

    // [新增] 特殊图层独立处理逻辑
    if (layerId == "Measurements") {
        if (checked) {
            if (m_hasResults) drawMeasurements();
            else {
                log("暂无体尺测量数据，请先执行计算。", "WARN");
                m_layerChecks["Measurements"]->blockSignals(true);
                m_layerChecks["Measurements"]->setChecked(false);
                m_layerChecks["Measurements"]->blockSignals(false);
            }
        } else {
            clearMeasurements();
        }
        m_viewer->getRenderWindow()->Render();
        return; // 直接返回，不走下面的普通点云逻辑
    }

    std::string cloudId = layerId.toStdString();

    // 无论显示还是隐藏，先移除旧的，防止 ID 重复导致渲染异常
    m_viewer->removePointCloud(cloudId);

    if (checked) {
        // =========================================================
        // 分支 A: 融合点云 (Merged) - 使用 RGB 颜色
        // =========================================================
        if (layerId == "Merged" && m_mergedCloudRGB && !m_mergedCloudRGB->empty()) {
            
            pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgbHandler(m_mergedCloudRGB);
            
            // [关键修复] 显式指定模板类型 <pcl::PointXYZRGB>
            bool ok = m_viewer->addPointCloud<pcl::PointXYZRGB>(m_mergedCloudRGB, rgbHandler, cloudId);
            log(QString("addPointCloud [%1] => %2").arg(layerId).arg(ok ? "true" : "false"), ok ? "INFO" : "ERROR");
            
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
        
        // =========================================================
        // 分支 B: 普通/原始点云 (Raw) - 使用固定单色
        // =========================================================
        else if (m_cloudData.contains(layerId) && !m_cloudData[layerId]->empty()) {
            
            // 1. 先计算该相机应该是什么颜色
            int r = 255, g = 255, b = 255;
            getCameraColor(layerId, r, g, b); // 获取颜色 (红/绿/蓝...)

            // 2. 创建自定义颜色处理器 (直接用这个颜色，不再依赖 setPointCloudRenderingProperties)
            pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(m_cloudData[layerId], r, g, b);
            
            // [关键修复] 显式指定模板类型 <PointT>
            // 这一步非常重要，混合了 RGB 和普通点云代码后，必须显式告诉编译器这是普通 PointT
            bool ok = m_viewer->addPointCloud<PointT>(m_cloudData[layerId], colorHandler, cloudId);
            log(QString("addPointCloud [%1] => %2").arg(layerId).arg(ok ? "true" : "false"), ok ? "INFO" : "ERROR");
            
            // 3. 只设置点大小
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
    }

    if (checked) {
        if (m_vtkRenderer) {
            m_vtkRenderer->ResetCamera();
            m_vtkRenderer->ResetCameraClippingRange();
        }
    }

    // 刷新渲染窗口
    m_viewer->getRenderWindow()->Render();
}

void SingleModePage::resizeEvent(QResizeEvent *event) {
    QWidget::resizeEvent(event);
    if (m_vtkWidget && m_viewer && m_viewer->getRenderWindow()) {
        m_vtkWidget->update();
        m_viewer->getRenderWindow()->Render();
    }
}

void SingleModePage::log(const QString& msg, const QString& type) {
    if (!m_console) return;

    // 获取当前时间
    QString timeStr = QDateTime::currentDateTime().toString("[HH:mm:ss]");

    // 设置颜色 HTML
    QString color = "#d4d4d4"; // 默认白色
    if (type == "WARN") color = "#e5c07b"; // 黄色
    else if (type == "ERROR") color = "#e06c75"; // 红色
    else if (type == "SUCCESS") color = "#98c379"; // 绿色
    else if (type == "ALGO") color = "#61afef"; // 蓝色

    // 组装 HTML 文本
    QString html = QString("<span style='color:#5c6370;'>%1</span> " // 时间灰色
                           "<span style='color:%2; font-weight:bold;'>[%3]</span> " // 类型带色
                           "<span style='color:#d4d4d4;'>%4</span>") // 内容
                           .arg(timeStr).arg(color).arg(type).arg(msg);

    m_console->append(html);

    // 自动滚动到底部
    QScrollBar *sb = m_console->verticalScrollBar();
    sb->setValue(sb->maximum());
}


void SingleModePage::getCameraColor(const QString& camName, int& r, int& g, int& b) {
    if      (camName == "Top") { r = 255; g = 0;   b = 0;   } // 红
    else if (camName == "LB")  { r = 0;   g = 255; b = 0;   } // 绿
    else if (camName == "LT")  { r = 0;   g = 0;   b = 255; } // 蓝
    else if (camName == "RB")  { r = 255; g = 215; b = 0;   } // 金 (Right-Bottom)
    else if (camName == "RT")  { r = 0;   g = 255; b = 255; } // 青 (Right-Top)
    else                       { r = 255; g = 255; b = 255; } // 默认白
}
void SingleModePage::initDefaultIntrinsics() {
    m_intrinsicsMap["Top"] = PointCloudAlgo::getCameraIntrinsics("Top", SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["LB"]  = PointCloudAlgo::getCameraIntrinsics("LB",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["LT"]  = PointCloudAlgo::getCameraIntrinsics("LT",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["RB"]  = PointCloudAlgo::getCameraIntrinsics("RB",  SensorType::DEPTH, 512, 512);
    m_intrinsicsMap["RT"]  = PointCloudAlgo::getCameraIntrinsics("RT",  SensorType::DEPTH, 512, 512);
}


// [新增] 弹出一个极简的专业内参配置表格
void SingleModePage::onSetIntrinsics() {
    QDialog dlg(this);
    dlg.setWindowTitle("⚙️ 自定义相机内参 (RAW -> PCD)");
    dlg.resize(650, 250);
    QVBoxLayout *lay = new QVBoxLayout(&dlg);

    QTableWidget *table = new QTableWidget(5, 6, &dlg);
    table->setHorizontalHeaderLabels({"fx", "fy", "cx", "cy", "Width", "Height"});
    table->setVerticalHeaderLabels({"Top", "LB", "LT", "RB", "RT"});
    table->horizontalHeader()->setSectionResizeMode(QHeaderView::Stretch);
    
    QStringList keys = {"Top", "LB", "LT", "RB", "RT"};
    for (int r = 0; r < keys.size(); ++r) {
        CameraIntrinsics intr = m_intrinsicsMap[keys[r]];
        table->setItem(r, 0, new QTableWidgetItem(QString::number(intr.fx, 'f', 4)));
        table->setItem(r, 1, new QTableWidgetItem(QString::number(intr.fy, 'f', 4)));
        table->setItem(r, 2, new QTableWidgetItem(QString::number(intr.cx, 'f', 4)));
        table->setItem(r, 3, new QTableWidgetItem(QString::number(intr.cy, 'f', 4)));
        table->setItem(r, 4, new QTableWidgetItem(QString::number(intr.width)));
        table->setItem(r, 5, new QTableWidgetItem(QString::number(intr.height)));
    }
    lay->addWidget(table);

    QPushButton *btnSave = new QPushButton("💾 保存并应用", &dlg);
    lay->addWidget(btnSave);

    connect(btnSave, &QPushButton::clicked, [&]() {
        for (int r = 0; r < keys.size(); ++r) {
            QString key = keys[r];
            m_intrinsicsMap[key].fx = table->item(r, 0)->text().toFloat();
            m_intrinsicsMap[key].fy = table->item(r, 1)->text().toFloat();
            m_intrinsicsMap[key].cx = table->item(r, 2)->text().toFloat();
            m_intrinsicsMap[key].cy = table->item(r, 3)->text().toFloat();
            m_intrinsicsMap[key].width = table->item(r, 4)->text().toInt();
            m_intrinsicsMap[key].height = table->item(r, 5)->text().toInt();
        }
        log("相机内参已更新，下次加载 RAW 文件将使用新参数。", "INFO");
        dlg.accept();
    });

    dlg.exec();
}
// 辅助函数：刷新 3D 视图
void SingleModePage::update3DView() {
    if (m_vtkWidget && m_viewer) {
        m_vtkWidget->update();
        m_viewer->getRenderWindow()->Render();
    }
}
