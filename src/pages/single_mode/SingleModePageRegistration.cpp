/*
 * 文件说明：实现 `SingleModePage` 的预处理、矩阵编辑、并行配准与融合逻辑。
 */
#include "pages/single_mode/SingleModePageInternal.h"

// ==========================================================
// [新增] 纯后台工作函数 (剥离 UI，完全在后台子线程中并行执行)
// ==========================================================
RegTaskOutput processRegistrationWorker(const RegTaskInput& input) {
    RegTaskOutput output;
    output.srcKey = input.srcKey;
    output.valid = true;
    output.finalTransform = input.initialGuess; // 默认使用初始矩阵
    
    // 伪装一个 logger，将日志存入缓存而不是直接操作 UI
    auto logBridge = [&output](const QString& msg, const QString& type) {
        output.logs.push_back({msg, type});
    };

    if (input.methodIndex == 0) { // 手动矩阵
        output.cloudAlignedLocal = PointCloudAlgo::transformCloud(input.cloudSrc, input.initialGuess);
    } 
    else if (input.methodIndex == 1 || input.methodIndex == 2) { // ICP
        auto result = PointCloudAlgo::alignICP(input.cloudSrc, input.cloudTarget, input.initialGuess, input.icpIter, input.icpDist, input.algoType, logBridge);
        output.cloudAlignedLocal = result.first;
        output.finalTransform = result.second;
    } 
    else if (input.methodIndex == 3) { // NDT
        Eigen::Matrix4f guess_f = input.initialGuess.cast<float>();
        Eigen::Matrix4f final_f = PointCloudAlgo::refineRegistrationNDT(input.cloudSrc, input.cloudTarget, guess_f, input.ndtRes, input.ndtStep, input.ndtIter, logBridge);
        output.finalTransform = final_f.cast<double>();
        output.cloudAlignedLocal = PointCloudAlgo::transformCloud(input.cloudSrc, output.finalTransform);
    }
    else if (input.methodIndex == 4) {  // G-ICP
        auto result = PointCloudAlgo::alignGICP(
            input.cloudSrc, input.cloudTarget, input.initialGuess, 
            input.gicpIter, input.gicpDist, input.gicpEps, logBridge
        );
        output.cloudAlignedLocal = result.first;
        output.finalTransform = result.second;
    }
    return output;
}
void SingleModePage::onRunPreprocess() {
    log("开始执行预处理预览...", "ALGO"); 
    int processedCount = 0;

    // 1. 获取所有参数
    float leaf_mm = m_spinLeafSize->value();
    double std_dev = m_spinStdDev->value();
    int mean_k = m_spinMeanK->value();
    float clip_radius_mm = m_spinClipRadius->value();

    // 2. 遍历所有可见点云
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        QString key = it.key();
        
        // 只处理勾选显示的层
        if (!m_layerChecks.contains(key) || !m_layerChecks[key]->isChecked()) {
            continue; 
        }

        PointCloudT::Ptr currentCloud = it.value(); 

        // --- Step 1: 下采样 ---
        currentCloud = PointCloudAlgo::downsample(currentCloud, leaf_mm);
        if (!currentCloud) continue;

        // --- Step 2: 统计滤波 (SOR) ---
        currentCloud = PointCloudAlgo::statisticalOutlierRemoval(currentCloud, mean_k, std_dev);
        if (!currentCloud) continue;

        // --- Step 3: 半径裁剪 ---
        currentCloud = PointCloudAlgo::distanceClip(currentCloud, clip_radius_mm);
        if (!currentCloud) continue;

        // 3. 更新可视化 (只有 currentCloud 有效才进来)
        if(currentCloud) {
            processedCount++;
            
            // [修复] 在这里声明 cloudId，并在该作用域内完成所有操作
            std::string cloudId = key.toStdString(); 

            // 日志
            log(QString("预处理 [%1]: 剩余点数 %2").arg(key).arg(currentCloud->size()), "INFO");

            // 移除旧的，添加新的
            m_viewer->removePointCloud(cloudId);
            pcl::visualization::PointCloudColorHandlerCustom<PointT> colorHandler(currentCloud, 255, 255, 255);
            m_viewer->addPointCloud(currentCloud, colorHandler, cloudId);

            // [修复] 颜色恢复逻辑移入 if 块内部，这样 cloudId 是可见的
            double r=1.0, g=1.0, b=1.0;
            if (key == "Top") { r=1.0; g=0.0; b=0.0; }
            else if (key == "LB") { r=0.0; g=1.0; b=0.0; }
            else if (key == "LT") { r=0.0; g=0.0; b=1.0; }
            else if (key == "RB") { r=1.0; g=0.84; b=0.0; }
            else if (key == "RT") { r=0.0; g=1.0; b=1.0; }
            
            // 现在 cloudId 在作用域内，不会报错了
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, r, g, b, cloudId);
            m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, cloudId);
        }
    }

    // [优化] 将总结性日志移到循环外面，避免刷屏
    log(QString("预处理完成，更新了 %1 个视图").arg(processedCount), "SUCCESS");

    // 4. 刷新视图 (只需在最后刷新一次)
    if(m_viewer->getRenderWindow()) m_viewer->getRenderWindow()->Render();
}


void SingleModePage::applyPreprocessToMemory() {
    // 1. 获取当前参数
    float leaf = m_spinLeafSize->value();
    double std_dev = m_spinStdDev->value();
    int mean_k = m_spinMeanK->value();
    float radius = m_spinClipRadius->value();

    int count = 0;
    // 2. 遍历内存数据，永久更新它们
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        // 对所有数据（无论是否显示）都进行处理，保证一致性
        PointCloudT::Ptr raw = it.value();
        
        // 依次执行三个算法
        auto p1 = PointCloudAlgo::downsample(raw, leaf);
        if(!p1) continue;
        auto p2 = PointCloudAlgo::statisticalOutlierRemoval(p1, mean_k, std_dev);
        if(!p2) continue;
        auto p3 = PointCloudAlgo::distanceClip(p2, radius);
        if(!p3) continue;

        // [关键] 覆盖内存中的数据
        m_cloudData[it.key()] = p3;
        count++;
    }

    QMessageBox::information(this, "处理完成", 
        QString("已对 %1 个点云执行预处理并保存到内存。\n后续配准将使用新数据。").arg(count));
        
    // 刷新一下视图
    // 遍历检查左侧面板哪些图层被勾选了，直接将内存中现成的结果刷新到视图
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        QString key = it.key();
        if (m_layerChecks.contains(key) && m_layerChecks[key]->isChecked()) {
            // 利用你已经写好的 onLayerToggle 函数，强制重新加载该图层
            onLayerToggle(key, true); 
        }
    }
}

void SingleModePage::initDefaultMatrices() {
    // 根据你的 pc_register.cpp 中的数据硬编码默认值
    // LB -> Top
    Eigen::Matrix4d lb;
    lb << 
        -0.926086, -0.052355, -0.373662, 719.549438,
        -0.368365, -0.088922, 0.925419, -1532.871094,
        -0.081677, 0.994662, 0.063064, 1826.347290,
        0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["LB"] = lb;

    // LT -> Top
    Eigen::Matrix4d lt;
    lt << -0.893402, 0.129827, -0.430091, 715.208496,
-0.431722, -0.512967, 0.741944, -1244.757813,
-0.124298, 0.848534, 0.514334, 853.710327,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["LT"] = lt;

    // RB -> Top
    Eigen::Matrix4d rb;
    rb << 0.843684, 0.019195, 0.536497, -845.013184,
0.527601, -0.214250, -0.822030, 1084.744995,
0.099166, 0.976590, -0.190886, 2075.820557,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["RB"] = rb;

    // RT -> Top
    Eigen::Matrix4d rt;
    rt << 0.881307, -0.310379, 0.356319, -767.480042,
0.451075, 0.327860, -0.830084, 1185.616089,
0.140818, 0.892285, 0.428950, 1194.222656,
0.000000, 0.000000, 0.000000, 1.000000;
    m_transforms["RT"] = rt;
}


QString SingleModePage::matrixToString(const Eigen::Matrix4d& mat) {
    QString str;
    for(int i=0; i<4; ++i) {
        for(int j=0; j<4; ++j) {
            // 保留4位小数，右对齐
            str += QString::number(mat(i,j), 'f', 6); 
            if(j < 3) str += "\t"; // 列间用制表符分隔
        }
        if(i < 3) str += "\n";
    }
    return str;
}

Eigen::Matrix4d SingleModePage::stringToMatrix(const QString& text) {
    Eigen::Matrix4d mat = Eigen::Matrix4d::Identity();
    QStringList tokens = text.split(QRegularExpression("[\\s,]+"), Qt::SkipEmptyParts);
    if(tokens.size() == 16) {
        int idx = 0;
        for(int i=0; i<4; ++i) {
            for(int j=0; j<4; ++j) {
                mat(i,j) = tokens[idx++].toFloat();
            }
        }
    }
    else {
        // 打印警告，如果解析失败，我们在后续逻辑中拦截
        qDebug() << "[警告] 矩阵解析失败！提取到的数字个数不是16个，而是:" << tokens.size();
    }
    return mat;
}

void SingleModePage::onMatrixTargetChanged(int index) {
    QString key = m_comboMatrixView->currentText();
    // 临时断开文本改变信号，防止死循环
    m_textMatrix->blockSignals(true);
    m_textMatrix->setText(matrixToString(m_transforms[key]));
    m_textMatrix->blockSignals(false);
}

void SingleModePage::onMatrixTextChanged() {
    QString key = m_comboMatrixView->currentText();
    m_transforms[key] = stringToMatrix(m_textMatrix->toPlainText());
}

void SingleModePage::onExecuteRegistration() {
    // 1. 同步编辑器矩阵并校验
    QString currentEditingKey = m_comboMatrixView->currentText();
    QString matrixText = m_textMatrix->toPlainText();
    QStringList tokens = matrixText.split(QRegularExpression("[\\s,]+"), Qt::SkipEmptyParts);
    if (tokens.size() == 16) {
        m_transforms[currentEditingKey] = stringToMatrix(matrixText);
    } else {
        QMessageBox::warning(this, "矩阵格式错误", "当前编辑框中的矩阵数字不是16个，请检查！");
        return;
    }

    QString targetKey = m_comboRegTarget->currentText();
    if (!m_cloudData.contains(targetKey)) {
        QMessageBox::warning(this, "错误", "未找到参考目标点云: " + targetKey);
        return;
    }
    PointCloudT::Ptr cloudTarget = m_cloudData[targetKey];
    Eigen::Matrix4d matTargetToTop = Eigen::Matrix4d::Identity();
    if (targetKey != "Top") matTargetToTop = m_transforms[targetKey];

    // ==========================================================
    // 2. 准备并行任务列表
    // ==========================================================
    QList<RegTaskInput> tasks;
    int methodIndex = m_comboRegMethod->currentIndex();

    for (auto it = m_sourceChecks.begin(); it != m_sourceChecks.end(); ++it) {
        QString srcKey = it.key(); 
        if (!it.value()->isChecked() || !m_cloudData.contains(srcKey) || srcKey == targetKey) continue; 

        RegTaskInput task;
        task.srcKey = srcKey;
        task.targetKey = targetKey;
        task.cloudSrc = m_cloudData[srcKey];
        task.cloudTarget = cloudTarget;
        task.initialGuess = matTargetToTop.inverse() * m_transforms[srcKey];
        task.methodIndex = methodIndex;
        task.algoType = (methodIndex == 2) ? PointCloudAlgo::P2Plane : PointCloudAlgo::P2Point;
        
        // UI 参数抓取
        // ICP 参数
        task.icpIter = m_spinIcpIter->value();
        task.icpDist = m_spinIcpDist->value();
        // NDT 参数
        task.ndtRes = m_spinNdtRes->value();
        task.ndtStep = m_spinNdtStep->value();
        task.ndtIter = m_spinNdtIter->value();
        // GICP 参数
        task.gicpIter = m_spinGicpIter->value();
        task.gicpDist = m_spinGicpDist->value();
        task.gicpEps = m_spinGicpEps->value();
        tasks.append(task);
    }

    if (tasks.isEmpty() && targetKey != "Top") {
        log("警告：没有需要配准的源点云。", "WARN");
        return;
    }

    // ==========================================================
    // 3. 挂起 UI 状态，启动并发线程池 (非阻塞)
    // ==========================================================
    log("🚀 启动多线程并行配准流水线，操作已挂起后台运行...", "ALGO");
    m_btnRunReg->setEnabled(false);
    m_btnRunReg->setText("⏳ 正在后台并行配准中...");

    QFutureWatcher<RegTaskOutput> *watcher = new QFutureWatcher<RegTaskOutput>(this);

    // ==========================================================
    // 4. 当所有后台线程完成时，触发回调更新 UI
    // ==========================================================
    connect(watcher, &QFutureWatcher<RegTaskOutput>::finished, this, [this, watcher, targetKey, cloudTarget, matTargetToTop]() {
        
        // 拿回所有线程的计算结果
        QList<RegTaskOutput> results = watcher->future().results();
        
        // 初始化存储容器
        PointCloudT::Ptr geometryMerged(new PointCloudT);
        if (!m_mergedCloudRGB) { m_mergedCloudRGB.reset(new pcl::PointCloud<pcl::PointXYZRGB>); }
        m_mergedCloudRGB->clear();

        // 统一合并颜色的 Lambda
        auto appendColoredCloud = [&](PointCloudT::Ptr inputCloud, const QString& camName) {
            if (!inputCloud) return;
            int r, g, b; getCameraColor(camName, r, g, b);
            for (const auto& pt : inputCloud->points) {
                geometryMerged->points.push_back(pt);
                
                // [修复] 先使用默认构造函数，再分别赋值
                pcl::PointXYZRGB ptRGB;
                ptRGB.x = pt.x; ptRGB.y = pt.y; ptRGB.z = pt.z;
                ptRGB.r = static_cast<uint8_t>(r);
                ptRGB.g = static_cast<uint8_t>(g);
                ptRGB.b = static_cast<uint8_t>(b);
                
                m_mergedCloudRGB->points.push_back(ptRGB);
            }
        };


        // 1. 先加入固定的目标点云
        appendColoredCloud(cloudTarget, targetKey);

        // 2. 将各个子线程算好的点云累加进来
        for(const auto& res : results) {
            if(!res.valid) continue;
            
            // 打印该线程在后台产生的所有日志
            for(const auto& l : res.logs) { log(l.first, l.second); }
            
            log(QString("完成 [%1 -> %2] 的配准与拼装。").arg(res.srcKey).arg(targetKey), "ALGO");

            // 更新内存矩阵
            m_transforms[res.srcKey] = matTargetToTop * res.finalTransform;
            
            // 拼装点云
            if (res.cloudAlignedLocal) appendColoredCloud(res.cloudAlignedLocal, res.srcKey);
        }

        // 3. 收尾工作
        geometryMerged->width = geometryMerged->size(); geometryMerged->height = 1; geometryMerged->is_dense = true;
        m_cloudData["Merged"] = geometryMerged; 
        m_mergedCloudRGB->width = m_mergedCloudRGB->size(); m_mergedCloudRGB->height = 1; m_mergedCloudRGB->is_dense = true;

        // 融合结果一旦更新，旧的主体提取与测量结果就不再可靠，必须同步失效。
        resetMeasurementState();
        m_cloudData.remove("Body");
        if (m_viewer) {
            m_viewer->removePointCloud("Body");
        }
        if (m_layerChecks.contains("Body")) {
            QSignalBlocker blocker(m_layerChecks["Body"]);
            m_layerChecks["Body"]->setChecked(false);
        }

        if (m_layerChecks.contains("Merged")) {
            m_layerChecks["Merged"]->setChecked(true);
            onLayerToggle("Merged", true); 
        }
        
        if (m_transforms.contains(m_comboMatrixView->currentText())) {
            onMatrixTargetChanged(0); 
        }

        log(QString("✅ 所有线程处理完毕。融合点云总点数: %1").arg(m_mergedCloudRGB->size()), "SUCCESS");

        // 恢复 UI 按钮状态
        m_btnRunReg->setEnabled(true);
        m_btnRunReg->setText("🚀 执行配准与融合");
        
        // 清理内存
        watcher->deleteLater();
    });

    // 🔥 发射任务给 Qt 的全局并发线程池！
    // QtConcurrent::mapped 会根据 CPU 核心数自动切分任务
    QFuture<RegTaskOutput> future = QtConcurrent::mapped(tasks, processRegistrationWorker);
    watcher->setFuture(future);
}
