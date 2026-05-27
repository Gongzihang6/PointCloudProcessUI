/*
 * 文件说明：实现 `SingleModePage` 的 AI 推理、关键点交互与体尺测量逻辑。
 */
#include "pages/single_mode/SingleModePageInternal.h"

// 在 PCL 视图中绘制关键点的辅助函数
void SingleModePage::drawKeypointsInViewer(const std::vector<Eigen::Vector3f>& kps) {
    if (!m_viewer) return;

    QStringList kpNames = {"P1", "P2", "P3", "P4", "P5", "P6"};

    // 每次绘制前，先清除上一轮画的关键点形状
    for (int i = 0; i < 10; ++i) { 
        m_viewer->removeShape("kp_sphere_" + std::to_string(i));
        m_viewer->removeText3D("kp_text_" + std::to_string(i));
    }

    for (size_t i = 0; i < kps.size() && i < kpNames.size(); ++i) {
        pcl::PointXYZ pt(kps[i].x(), kps[i].y(), kps[i].z());
        
        std::string sphereId = "kp_sphere_" + std::to_string(i);
        std::string textId = "kp_text_" + std::to_string(i);

        // 1. 添加红色的 3D 球体 (半径 15mm，视你的猪体大小而定)
        m_viewer->addSphere(pt, 15.0, 1.0, 0.0, 0.0, sphereId);
        
        // 2. 在球体旁边稍微偏上一点的地方添加文字标签
        pcl::PointXYZ textPt(pt.x, pt.y, pt.z + 20.0); 
        m_viewer->addText3D(kpNames[i].toStdString(), textPt, 15.0, 1.0, 1.0, 1.0, textId);
    }

    m_viewer->getRenderWindow()->Render();
}



/*
 * 作用：执行 AI 模型推理 (客户端侧逻辑)
 * 功能：提取背部点云 -> 计算法线与曲率特征 -> 序列化为 N*7 的二进制流 -> 通过 HTTP POST 发送给 WSL2 中的 Python 服务 -> 异步接收解析关键点坐标 -> 渲染到 3D 视图。
 * 实现机制：使用 PCL 的 NormalEstimationOMP 计算特征；使用 reinterpret_cast 强转指针实现零拷贝级别的二进制打包；使用 QNetworkAccessManager 实现非阻塞异步通信。
 */
void SingleModePage::onRunAIInference() {
    // [修改] 用封装好的函数代替之前长长的提取代码
    if (!prepareKeypointsCloud()) {
        return; // 如果准备失败（比如没有 Top 视角），直接中止
    }
    // 从内存中把刚准备好的背部点云取出来，赋给 backCloud 变量
    PointCloudT::Ptr backCloud = m_cloudData["Keypoints"];

    // 切换图层显示，专注显示关键点云
    for(auto* chk : m_layerChecks) { chk->setChecked(false); }
    if (m_layerChecks.contains("Keypoints")) {
        m_layerChecks["Keypoints"]->setChecked(true);
        onLayerToggle("Keypoints", true);
    }

    // ==========================================================
    // 3. 计算法向量与曲率特征 (N * 4) -> 组合成 N * 7
    // ==========================================================
    log("正在计算点云法线与曲率特征...", "ALGO");
    pcl::NormalEstimationOMP<PointT, pcl::PointNormal> ne;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);
    ne.setInputCloud(backCloud);
    ne.setRadiusSearch(50.0); // 根据实际猪体尺寸调整搜索半径

    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_normals(new pcl::PointCloud<pcl::PointNormal>);
    ne.compute(*cloud_normals);

    // ==========================================================
    // 4. 将数据打包为紧凑的二进制字节流 (QByteArray)
    // 格式: [x1, y1, z1, nx1, ny1, nz1, cur1, x2, y2, z2, nx2...]
    // ==========================================================
    int numPoints = backCloud->size();
    QByteArray postData;
    postData.resize(numPoints * 7 * sizeof(float)); // 预分配内存，7个float = 28字节/点
    
    // 使用指针直接写入内存，速度极快
    float* ptr = reinterpret_cast<float*>(postData.data());
    for (int i = 0; i < numPoints; ++i) {
        *ptr++ = backCloud->points[i].x;
        *ptr++ = backCloud->points[i].y;
        *ptr++ = backCloud->points[i].z;
        *ptr++ = cloud_normals->points[i].normal_x;
        *ptr++ = cloud_normals->points[i].normal_y;
        *ptr++ = cloud_normals->points[i].normal_z;
        *ptr++ = cloud_normals->points[i].curvature;
    }

    // ==========================================================
    // 5. 构建并发送 HTTP POST 请求
    // ==========================================================
    log("正在发送数据至 AI 服务器(WSL2)执行推理...", "INFO");
    
    // 使用 QHttpMultiPart 构建表单数据，匹配 FastAPI 的 UploadFile
    QHttpMultiPart *multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);
    QHttpPart filePart;
    filePart.setHeader(QNetworkRequest::ContentDispositionHeader, 
                       QVariant("form-data; name=\"file\"; filename=\"pointcloud.bin\""));
    filePart.setBody(postData);
    multiPart->append(filePart);

    QNetworkRequest request(QUrl("http://127.0.0.1:8000/predict"));
    
    // 发送请求
    QNetworkReply *reply = m_networkManager->post(request, multiPart);
    multiPart->setParent(reply); // 随 reply 一起自动销毁，防止内存泄漏

    // [可选] 禁用按钮防止重复点击
    m_btnRunAI3D->setEnabled(false); m_btnRunAI3D->setText("3D 推理中...");

    // ==========================================================
    // 6. 异步处理返回结果 (C++11 Lambda 回调)
    // ==========================================================
    connect(reply, &QNetworkReply::finished, this, [this, reply]() {
        if (reply->error() == QNetworkReply::NoError) {
            // 读取返回的二进制数据
            QByteArray responseData = reply->readAll();
            int numKeypoints = responseData.size() / (3 * sizeof(float));
            
            if (numKeypoints > 0) {
                const float* outPtr = reinterpret_cast<const float*>(responseData.constData());
                
                // ---------------------------------------------------------
                // [核心修改 1]：不再使用局部变量，而是直接操作成员变量 m_keypoints
                // 每次 AI 重新推理成功时，必须先清空旧数据（包括之前手动点的或上次AI算的）
                // ---------------------------------------------------------
                this->m_keypoints.clear();
                
                // 遍历解析坐标并存入类成员变量
                for (int i = 0; i < numKeypoints; ++i) {
                    float kx = outPtr[i*3 + 0];
                    float ky = outPtr[i*3 + 1];
                    float kz = outPtr[i*3 + 2];
                    this->m_keypoints.push_back(Eigen::Vector3f(kx, ky, kz));
                }

                log(QString("AI 推理成功！检测到 %1 个关键点。").arg(numKeypoints), "SUCCESS");
                
                // ---------------------------------------------------------
                // 1. 在控制台输出所有关键点的绝对三维坐标
                // ---------------------------------------------------------
                QStringList kpNames = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
                for (int i = 0; i < numKeypoints && i < kpNames.size(); ++i) {
                    // [核心修改 2]：从 this->m_keypoints 中读取坐标进行打印
                    QString coordMsg = QString("  ▶ %1: (X: %2, Y: %3, Z: %4) mm")
                                        .arg(kpNames[i])
                                        .arg(this->m_keypoints[i].x(), 0, 'f', 2)
                                        .arg(this->m_keypoints[i].y(), 0, 'f', 2)
                                        .arg(this->m_keypoints[i].z(), 0, 'f', 2);
                    log(coordMsg, "INFO");
                }

                // 2. 在 3D 视图中渲染
                // [核心修改 3]：将成员变量传给渲染函数
                this->drawKeypointsInViewer(this->m_keypoints);

                // 3. 更新界面 UI 网格 (让 Badge 变绿)
                for (int i = 0; i < m_kpBadges.size(); ++i) {
                    if (i < numKeypoints) {
                        m_kpBadges[i]->setStyleSheet(
                            "background-color: #f0f9eb; color: #67c23a; "
                            "border: 1px solid #c2e7b0; border-radius: 4px; "
                            "padding: 4px; font-weight: bold;"
                        );
                    } else {
                        m_kpBadges[i]->setStyleSheet(
                            "background-color: #f4f4f5; color: #909399; "
                            "border: 1px solid #d3d4d6; border-radius: 4px; padding: 4px;"
                        );
                    }
                }
            } else {
                log("服务端返回的数据为空或不合法！", "ERROR");
            }
        } else {
            // 网络通信错误处理
            log("AI 推理失败: " + reply->errorString(), "ERROR");
        }
        
        reply->deleteLater(); // 释放内存
        
        // 恢复 AI 按钮状态（如果你采用了成员变量 m_btnRunAI）
        if (m_btnRunAI3D) {
            m_btnRunAI3D->setEnabled(true); 
            m_btnRunAI3D->setText("☁️ 运行 3D 点云推理 (服务端)");
        }
    });
}

// 辅助函数：统一管理 Badge 的三种颜色状态
void SingleModePage::updateBadgeStyle(int index, int state) {
    if (index < 0 || index >= m_kpBadges.size()) return;
    
    QLabel* badge = m_kpBadges[index];
    if (state == 0) { // 灰色 (未完成)
        badge->setStyleSheet("background-color: #f4f4f5; color: #909399; border: 1px solid #d3d4d6; border-radius: 4px; padding: 4px;");
    } else if (state == 1) { // 蓝色 (正在等待拾取该点)
        badge->setStyleSheet("background-color: #ecf5ff; color: #409eff; border: 1px solid #b3d8ff; border-radius: 4px; padding: 4px; font-weight: bold;");
    } else if (state == 2) { // 绿色 (已完成拾取/预测)
        badge->setStyleSheet("background-color: #f0f9eb; color: #67c23a; border: 1px solid #c2e7b0; border-radius: 4px; padding: 4px; font-weight: bold;");
    }
}

// 核心逻辑：当用户 Shift+左键 成功点到一个点时触发
void SingleModePage::onManualPointPicked(double x, double y, double z) {
    if (!m_isManualPickingMode || m_currentPickIndex >= 6) return;

    QStringList kpNames = {"P1 耳中", "P2 肩胛", "P3 背中", "P4 腰", "P5 臀", "P6 尾根"};
    
    // 1. 保存坐标
    m_keypoints.push_back(Eigen::Vector3f(x, y, z));

    // 2. 打印控制台日志
    QString coordMsg = QString("  ▶ 手动拾取 %1: (X: %2, Y: %3, Z: %4) mm")
                        .arg(kpNames[m_currentPickIndex])
                        .arg(x, 0, 'f', 2).arg(y, 0, 'f', 2).arg(z, 0, 'f', 2);
    log(coordMsg, "SUCCESS");

    // 3. 将当前 Badge 设为绿色
    updateBadgeStyle(m_currentPickIndex, 2);

    // 4. 在 3D 视图中渲染（复用你之前的函数）
    drawKeypointsInViewer(m_keypoints);

    // 5. 游标前进，高亮下一个 Badge 为蓝色
    m_currentPickIndex++;
    if (m_currentPickIndex < 6) {
        updateBadgeStyle(m_currentPickIndex, 1);
        log(QString("请按住 Shift+左键 拾取下一个点: [%1]").arg(kpNames[m_currentPickIndex]), "INFO");
    } else {
        log("🎉 6 个关键点已全部手动拾取完毕！", "SUCCESS");
        m_isManualPickingMode = false; // 自动退出拾取模式
        // 这里你也可以触发后续的“计算体尺参数”逻辑
    }
}

// 检查并生成关键点检测专用的点云
bool SingleModePage::prepareKeypointsCloud() {
    // 1. 如果已经存在，说明之前生成过，直接返回成功
    if (m_cloudData.contains("Keypoints") && !m_cloudData["Keypoints"]->empty()) {
        return true; 
    }

    // 2. 如果不存在，检查前置依赖 (Top 点云)
    if (!m_cloudData.contains("Top") || m_cloudData["Top"]->empty()) {
        QMessageBox::warning(this, "错误", "缺少 Top 相机点云，无法提取背部特征！");
        return false;
    }

    log("开始准备关键点检测云 (提取背部点云)...", "ALGO");
    
    // =======================================================
    // [核心修改] 组装提取参数 (利用 UI 包围盒 + 强制欧式聚类)
    // =======================================================
    ExtractionParams params;
    // 直接抓取用户在右侧 UI 面板设置的裁剪包围盒
    params.boxMinX = m_spinBoxMinX->value();
    params.boxMinY = m_spinBoxMinY->value();
    params.boxMinZ = m_spinBoxMinZ->value();
    params.boxMaxX = m_spinBoxMaxX->value();
    params.boxMaxY = m_spinBoxMaxY->value();
    params.boxMaxZ = m_spinBoxMaxZ->value();
    
    // 针对背部顶层提取，设置较小的簇限制和较大的容差
    params.minClusterSize = 1000;
    params.methodIndex = 0; // 强制使用欧式聚类 (速度最快，适合粗提)
    params.euclideanTolerance = 50.0;

    auto logBridge = [this](const QString& msg, const QString& type) { this->log(msg, type); };
    
    // 调用最新的算法 API
    PointCloudT::Ptr backCloud = PointCloudAlgo::extractLargestCluster(m_cloudData["Top"], params, logBridge);

    if (!backCloud) {
        log("背部点云提取失败，无法进行标注！", "ERROR");
        return false;
    }

    // 存入内存
    m_cloudData["Keypoints"] = backCloud;
    return true;
}



// ---------------------------------------------------------
// [新增] 清理测量结果图层
// ---------------------------------------------------------
void SingleModePage::clearMeasurements() {
    if (!m_viewer) return;
    m_viewer->removePointCloud("measure_body_cloud");
    m_viewer->removePointCloud("skel_pts");
    m_viewer->removeShape("ground_plane");
    m_viewer->removeShape("meas_height");
    m_viewer->removeShape("meas_width");
    
    // 粗略遍历移除所有动态生成的线段 (PCL的 removeShape 效率很高，不怕多循环)
    for (int i = 0; i < 300; ++i) {
        m_viewer->removeShape("skel_line_" + std::to_string(i));
        m_viewer->removeShape("chest_" + std::to_string(i));
        m_viewer->removeShape("waist_" + std::to_string(i));
        m_viewer->removeShape("hip_" + std::to_string(i));
    }
    m_viewer->removeShape("chest_close");
    m_viewer->removeShape("waist_close");
    m_viewer->removeShape("hip_close");
}

// ---------------------------------------------------------
// [新增] 绘制测量结果图层
// ---------------------------------------------------------
void SingleModePage::drawMeasurements() {
    if (!m_viewer || !m_hasResults) return;
    
    clearMeasurements(); // 绘制前先清理旧的

    BodySizeResults& res = m_latestResults;

    // A. 半透明主体
    pcl::visualization::PointCloudColorHandlerCustom<PointT> body_color(res.aligned_cloud, 200, 200, 200);
    m_viewer->addPointCloud<PointT>(res.aligned_cloud, body_color, "measure_body_cloud");
    m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.4, "measure_body_cloud"); 

    // B. 对齐后的关键点
    std::vector<Eigen::Vector3f> eigen_kps;
    for (const auto& pt : res.aligned_keypoints) eigen_kps.push_back(Eigen::Vector3f(pt.x, pt.y, pt.z));
    drawKeypointsInViewer(eigen_kps);

    // C. 绿色骨架
    if (res.skeleton_cloud && res.skeleton_cloud->size() > 1) {
        pcl::visualization::PointCloudColorHandlerCustom<PointT> skel_color(res.skeleton_cloud, 0, 255, 0);
        m_viewer->addPointCloud<PointT>(res.skeleton_cloud, skel_color, "skel_pts");
        m_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, "skel_pts");
        for (size_t i = 0; i < res.skeleton_cloud->size() - 1; ++i) {
            m_viewer->addLine<PointT>(res.skeleton_cloud->points[i], res.skeleton_cloud->points[i+1], 0, 1.0, 0, "skel_line_" + std::to_string(i));
            m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, "skel_line_" + std::to_string(i));
        }
    }

    // D. 地面与高宽线
    if (res.ground_polygon && res.ground_polygon->size() == 4) {
        m_viewer->addPolygon<PointT>(res.ground_polygon, 0.0, 1.0, 0.0, "ground_plane");
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, "ground_plane");
        m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.3, "ground_plane"); 
    }
    m_viewer->addLine<PointT>(res.height_top, res.height_bottom, 0.0, 0.0, 1.0, "meas_height");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "meas_height");
    m_viewer->addLine<PointT>(res.width_p1, res.width_p2, 1.0, 1.0, 0.0, "meas_width");
    m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "meas_width");

    // E. 轮廓多边形
    auto drawContour = [this](PointCloudT::Ptr contour, std::string id, double r, double g, double b) {
        if (!contour || contour->size() < 3) return;
        for (size_t i = 0; i < contour->size() - 1; ++i) {
            this->m_viewer->addLine(contour->points[i], contour->points[i + 1], r, g, b, id + "_" + std::to_string(i));
            this->m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id + "_" + std::to_string(i));
        }
        this->m_viewer->addLine(contour->points.back(), contour->points.front(), r, g, b, id + "_close");
        this->m_viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 3, id + "_close");
    };
    drawContour(res.chest_contour, "chest", 1.0, 0.5, 0.0);
    drawContour(res.waist_contour, "waist", 0.0, 1.0, 1.0);
    drawContour(res.hip_contour,   "hip",   1.0, 0.0, 1.0);
}

// ---------------------------------------------------------
// [重构] 计算主函数，剥离强耦合绘图代码
// ---------------------------------------------------------
void SingleModePage::onCalculateBodySize() {
    if (m_keypoints.size() != 6) {
        QMessageBox::warning(this, "警告", "关键点未准备就绪！请确保 6 个关键点已全部检测。");
        return;
    }
    if (!m_cloudData.contains("Merged") || !m_cloudData.contains("Body")) {
        QMessageBox::warning(this, "警告", "缺少主体或融合点云数据！");
        return;
    }

    log("开始执行体尺自动计算流水线...", "ALGO");

    float girth_thick = m_spinGirthThick->value();
    float skel_step = m_spinSkelStep->value();
    float skel_radius = m_spinSkelRadius->value();
    float height_angle = m_spinHeightAngle->value();

    PointCloudT::Ptr cloud_body(new PointCloudT(*m_cloudData["Body"]));
    PointCloudT::Ptr cloud_merged(new PointCloudT(*m_cloudData["Merged"]));
    
    BodySizeResults results = PointCloudAlgo::calculateAllMeasurements(
        cloud_body, cloud_merged, m_keypoints, 
        girth_thick, skel_step, skel_radius, height_angle,
        [this](const QString& msg, const QString& type){ this->log(msg, type); }
    );

    if (!results.aligned_cloud || results.aligned_cloud->empty()) {
        log("体尺计算失败！", "ERROR");
        return;
    }

    // 缓存数据
    m_latestResults = results;
    m_hasResults = true;

    // [核心改变] 自动关闭其它所有图层，并激活 "Measurements" 图层
    for (auto* chk : m_layerChecks) { 
        chk->blockSignals(true); 
        chk->setChecked(false); 
        chk->blockSignals(false); 
    }
    // 关闭所有普通点云
    for (auto it = m_cloudData.begin(); it != m_cloudData.end(); ++it) {
        m_viewer->removePointCloud(it.key().toStdString());
    }
    
    // 勾选并触发 Measurements 图层，它会自动调用 drawMeasurements()
    if (m_layerChecks.contains("Measurements")) {
        m_layerChecks["Measurements"]->setChecked(true);
        onLayerToggle("Measurements", true);
    }
    
    m_viewer->resetCamera(); // 居中视角

    // 打印结果报告... (保留原来的 log 报告代码)
    log("==============================", "SUCCESS");
    log("       体 尺 测 量 报 告       ", "SUCCESS");
    log(QString("▶ 体长 (Body Length) : %1 mm").arg(results.body_length, 0, 'f', 2), "INFO");
    log(QString("▶ 体高 (Body Height) : %1 mm").arg(results.body_height, 0, 'f', 2), "INFO");
    log(QString("▶ 体宽 (Body Width)  : %1 mm").arg(results.body_width, 0, 'f', 2), "INFO");
    log(QString("▶ 胸围 (Chest Girth) : %1 mm").arg(results.chest_girth, 0, 'f', 2), "INFO");
    log(QString("▶ 腰围 (Waist Girth) : %1 mm").arg(results.waist_girth, 0, 'f', 2), "INFO");
    log(QString("▶ 臀围 (Hip Girth)   : %1 mm").arg(results.hip_girth, 0, 'f', 2), "INFO");
    log("==============================", "SUCCESS");
}
void SingleModePage::onRunAIInference2D() {
    prepareKeypointsCloud(); 
    if (m_topColorImage.empty() || m_topAlignedDepthImage.empty()) {
        QMessageBox::warning(this, "错误", "缺少 Top 相机的彩色图或对齐深度图！");
        return;
    }

    QString modelPath = m_leModelPath->text();
    QString initMsg; 
    if (!m_openpose.initModel(modelPath, initMsg)) {
        log(QString("ONNX 加载失败: %1").arg(initMsg), "ERROR"); return;
    }

    log("正在执行本地 2D 图像深度学习推理...", "ALGO");
    
    // 1. 接收带有置信度的 3D 向量 (x, y, 置信度)
    std::vector<cv::Point3f> kps2d_with_conf = m_openpose.predict(m_topColorImage);
    this->m_keypoints.clear();
    
    // 获取相机内外参
    CameraIntrinsics colorIntr = PointCloudAlgo::getCameraIntrinsics("Top", SensorType::COLOR, 1280, 720);
    CameraDeviceParams params = PointCloudAlgo::getCameraParams("Top");

    Eigen::Matrix3f R;
    R << params.extrinsics.R[0], params.extrinsics.R[1], params.extrinsics.R[2],
         params.extrinsics.R[3], params.extrinsics.R[4], params.extrinsics.R[5],
         params.extrinsics.R[6], params.extrinsics.R[7], params.extrinsics.R[8];
    Eigen::Vector3f T(params.extrinsics.T[0], params.extrinsics.T[1], params.extrinsics.T[2]);

    // 准备 OpenCV 矩阵用于去畸变运算
    cv::Mat camMat = (cv::Mat_<double>(3, 3) << colorIntr.fx, 0, colorIntr.cx, 0, colorIntr.fy, colorIntr.cy, 0, 0, 1);
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 8) << colorIntr.k1, colorIntr.k2, colorIntr.p1, colorIntr.p2, colorIntr.k3, colorIntr.k4, colorIntr.k5, colorIntr.k6);

    // 克隆一张图用于弹窗可视化
    cv::Mat vizImage = m_topColorImage.clone();

    int validCount = 0;
    for (int i = 0; i < kps2d_with_conf.size(); ++i) {
        float u = kps2d_with_conf[i].x;
        float v = kps2d_with_conf[i].y;
        float conf = kps2d_with_conf[i].z; // 提取热力值
        
        // 【新增 1】将热力值打印到 UI 左下角的自定义日志框中
        log(QString("P%1 置信度: %2").arg(i+1).arg(conf, 0, 'f', 4), "INFO");

        if (u < 0 || v < 0 || conf < 0.1) { 
            log(QString("P%1 关键点未有效检测到！(置信度: %2)").arg(i+1).arg(conf, 0, 'f', 4), "WARN");
            continue;
        }

        // 画 2D 点到可视化图上
        cv::circle(vizImage, cv::Point(u, v), 8, cv::Scalar(0, 0, 255), -1);
        cv::putText(vizImage, "P" + std::to_string(i+1), cv::Point(u + 10, v), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);

        // 深度补洞逻辑 (不变)
        int ui = cvRound(u), vi = cvRound(v);
        uint16_t z_raw = m_topAlignedDepthImage.at<uint16_t>(vi, ui);
        if (z_raw == 0) {
            int r = 3; float sum = 0; int cnt = 0;
            for(int dy = -r; dy <= r; ++dy) {
                for(int dx = -r; dx <= r; ++dx) {
                    int ny = vi + dy, nx = ui + dx;
                    if(nx >= 0 && nx < 1280 && ny >= 0 && ny < 720) {
                        uint16_t val = m_topAlignedDepthImage.at<uint16_t>(ny, nx);
                        if(val > 0) { sum += val; cnt++; }
                    }
                }
            }
            if(cnt > 0) z_raw = sum / cnt; 
        }

        if (z_raw == 0) continue;

        // =======================================================
        // 【新增 2：核心数学修复】对 2D 坐标进行 OpenCV 物理去畸变
        // =======================================================
        std::vector<cv::Point2f> srcPts = {cv::Point2f(u, v)};
        std::vector<cv::Point2f> dstPts;
        // undistortPoints 会直接输出“归一化”的理想坐标 (x/Z, y/Z)
        cv::undistortPoints(srcPts, dstPts, camMat, distCoeffs); 

        float z_c = static_cast<float>(z_raw); 
        // 直接乘以深度，得到 Color 相机坐标系下的完美 3D 坐标
        float x_c = dstPts[0].x * z_c;
        float y_c = dstPts[0].y * z_c;

        Eigen::Vector3f P_color(x_c, y_c, z_c);
        
        // 逆变换回 Depth 点云坐标系
        Eigen::Vector3f P_depth = R.transpose() * (P_color - T);
        
        this->m_keypoints.push_back(P_depth);
        validCount++;

        // 【新增需求】：在控制台中漂亮地输出该点的 2D 像素坐标、3D 空间坐标以及置信度
        log(QString("P%1 [2D] 像素: (%2, %3) | [3D] 毫米: (X:%4, Y:%5, Z:%6) | 置信度: %7")
            .arg(i+1)
            .arg(u, 0, 'f', 1).arg(v, 0, 'f', 1)
            .arg(P_depth.x(), 0, 'f', 1).arg(P_depth.y(), 0, 'f', 1).arg(P_depth.z(), 0, 'f', 1)
            .arg(conf, 0, 'f', 4), "INFO");
    }

    // 【新增 3】弹出带有检测结果的 OpenCV 图片窗口
    cv::namedWindow("2D AI Detection Result", cv::WINDOW_NORMAL);
    cv::resizeWindow("2D AI Detection Result", 1280, 720);
    cv::imshow("2D AI Detection Result", vizImage);

    // 更新 UI 状态
    if (validCount == 6) {
        log("🎉 AI 关键点检测完成，经过物理去畸变与外参映射，已贴合至点云！", "SUCCESS");
        this->drawKeypointsInViewer(this->m_keypoints);
        for (int i = 0; i < m_kpBadges.size(); ++i) updateBadgeStyle(i, 2);
        
        // 【新增 4】强制点亮融合点云/主体点云的图层复选框，让底图显示出来
        if (m_layerChecks.contains("Body")) {
            m_layerChecks["Body"]->setChecked(true); // 激活主体猪图层
        } else if (m_layerChecks.contains("Merged")) {
            m_layerChecks["Merged"]->setChecked(true); // 退而求其次激活融合图层
        }
    } else {
        log("模型未能完整检测 6 个点，请检查图像质量或转入手动模式。", "WARN");
    }
}
