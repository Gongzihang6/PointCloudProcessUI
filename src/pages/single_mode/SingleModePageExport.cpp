/*
 * 文件说明：实现 `SingleModePage` 的点云、报告与可视化结果导出逻辑。
 */
#include "pages/single_mode/SingleModePageInternal.h"

// ==========================================================
// 导出模块：另存融合点云 (PCD)
// ==========================================================
void SingleModePage::onExportMergedCloud() {
    // 检查是否存在融合点云
    if (!m_cloudData.contains("Merged") || m_cloudData["Merged"]->empty()) {
        QMessageBox::warning(this, "警告", "没有找到融合点云，请先执行配准与融合！");
        return;
    }

    // 弹出文件保存对话框
    QString defaultName = QString("Merged_Cloud_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "另存融合点云", defaultName, "PCD Files (*.pcd)");
    
    if (fileName.isEmpty()) return; // 用户取消了保存

    // 提示状态
    log("正在保存融合点云至磁盘...", "INFO");

    // 使用 PCL 保存点云为二进制格式（体积更小，加载更快）
    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *m_cloudData["Merged"]) == 0) {
        log(QString("✅ 融合点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 融合点云保存失败，请检查路径权限！", "ERROR");
    }
}

// ==========================================================
// 导出模块：另存主体点云 (PCD)
// ==========================================================
void SingleModePage::onExportBodyCloud() {
    // 检查是否存在猪主体点云
    if (!m_cloudData.contains("Body") || m_cloudData["Body"]->empty()) {
        QMessageBox::warning(this, "警告", "没有找到提取的主体点云，请先执行主体提取！");
        return;
    }

    QString defaultName = QString("Pig_Body_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "另存主体点云", defaultName, "PCD Files (*.pcd)");
    
    if (fileName.isEmpty()) return;

    log("正在保存主体点云至磁盘...", "INFO");

    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *m_cloudData["Body"]) == 0) {
        log(QString("✅ 主体点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 主体点云保存失败！", "ERROR");
    }
}

// ==========================================================
// 导出模块：导出测量报告 (TXT 精美排版)
// ==========================================================
void SingleModePage::onExportReport() {
    // 检查是否已经计算过体尺
    if (!m_hasResults) {
        QMessageBox::warning(this, "警告", "尚未生成体尺数据！请先点击【计算体尺参数】。");
        return;
    }

    // 1. 弹出保存对话框，后缀改为 .txt
    QString defaultName = QString("Pig_BodySize_Report_%1.txt").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "导出测量报告", defaultName, "Text Files (*.txt)");
    
    if (fileName.isEmpty()) return;

    // 2. 打开文件
    QFile file(fileName);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "错误", "无法创建或写入该 TXT 文件！文件可能被其他程序占用。");
        return;
    }

    QTextStream out(&file);
    
    // 兼容性设置：确保以 UTF-8 编码写入 (Qt6 默认已是 UTF-8)
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
    out.setCodec("UTF-8");
#endif

    // =======================================================
    // 3. 开始精美排版写入
    // =======================================================
    out << "============================================================\n";
    out << "                     猪 只 体 尺 测 量 报 告                   \n";
    out << "============================================================\n";
    out << QString(" 测量时间 : %1\n").arg(QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss"));
    out << "------------------------------------------------------------\n";
    out << " 【测量参数】\t\t\t\t【测量值】\n";
    out << "------------------------------------------------------------\n";

    // 辅助 Lambda：使用固定宽度对齐文本 (-25表示左对齐占25字符，8表示右对齐占8字符)
    auto writeLine = [&out](const QString& name, double value) {
        out << " " << QString("%1").arg(name, -25) 
            << " :\t" 
            << QString("%1").arg(value, 8, 'f', 2) 
            << " mm\n";
    };

    writeLine("体长 (Body Length)", m_latestResults.body_length);
    writeLine("体高 (Body Height)", m_latestResults.body_height);
    writeLine("体宽 (Body Width)",  m_latestResults.body_width);
    writeLine("胸围 (Chest Girth)", m_latestResults.chest_girth);
    writeLine("腰围 (Waist Girth)", m_latestResults.waist_girth);
    writeLine("臀围 (Hip Girth)",   m_latestResults.hip_girth);

    out << "============================================================\n";
    out << " * 本报告由多相机 3D 点云处理系统自动生成。\n";

    file.close();
    
    log(QString("✅ 测量报告已成功导出为 TXT: %1").arg(fileName), "SUCCESS");
}

// ==========================================================
// 导出模块：导出测量结果的可视化彩色点云
// 核心技术：将所有 3D 图元(线、多边形、球)降维离散化为密集的彩色点云
// ==========================================================
void SingleModePage::onExportVizCloud() {
    if (!m_hasResults || !m_latestResults.aligned_cloud) {
        QMessageBox::warning(this, "警告", "尚未生成体尺数据！请先点击【计算体尺参数】。");
        return;
    }

    QString defaultName = QString("Pig_Visualization_%1.pcd").arg(QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss"));
    QString fileName = QFileDialog::getSaveFileName(this, "导出测量可视化点云", defaultName, "PCD Files (*.pcd)");
    if (fileName.isEmpty()) return;

    log("正在将线框与图元渲染为彩色点云...", "ALGO");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr viz_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    BodySizeResults& res = m_latestResults;

    // 1. 注入主体点云 (浅灰色)
    for (const auto& p : res.aligned_cloud->points) {
        pcl::PointXYZRGB pt; pt.x = p.x; pt.y = p.y; pt.z = p.z;
        pt.r = 200; pt.g = 200; pt.b = 200;
        viz_cloud->push_back(pt);
    }

    // 辅助 Lambda：在两点之间进行插值，并生成物理“加粗”的点云管道
    auto drawLine = [&](const PointT& p1, const PointT& p2, uint8_t r, uint8_t g, uint8_t b, float thickness = 2.0f) {
        Eigen::Vector3f v1 = p1.getVector3fMap();
        Eigen::Vector3f v2 = p2.getVector3fMap();
        float dist = (v2 - v1).norm();
        float forward_step = 1.0f; // 沿线段前进的步长 1mm
        int steps = std::max(1, static_cast<int>(dist / forward_step));

        for (int i = 0; i <= steps; ++i) {
            Eigen::Vector3f p = v1 + (v2 - v1) * (static_cast<float>(i) / steps);
            
            // 为了加粗，在中心点周围的立体空间内撒点，形成有厚度的管道
            // thickness 为管道的物理半径 (mm)
            for (float dx = -thickness; dx <= thickness; dx += 1.5f) {
                for (float dy = -thickness; dy <= thickness; dy += 1.5f) {
                    for (float dz = -thickness; dz <= thickness; dz += 1.5f) {
                        // 利用勾股定理，只保留圆柱/球截面内的点，使线条圆润
                        if (dx*dx + dy*dy + dz*dz <= thickness*thickness) {
                            pcl::PointXYZRGB pt;
                            pt.x = p.x() + dx; 
                            pt.y = p.y() + dy; 
                            pt.z = p.z() + dz;
                            pt.r = r; pt.g = g; pt.b = b;
                            viz_cloud->push_back(pt);
                        }
                    }
                }
            }
        }
    };


    // 辅助 Lambda：在指定中心画一个实心球体 (关键点)
    auto drawSphere = [&](const PointT& center, float radius, uint8_t r, uint8_t g, uint8_t b) {
        float step = 1.5f; // 空间采样步长
        for (float dx = -radius; dx <= radius; dx += step) {
            for (float dy = -radius; dy <= radius; dy += step) {
                for (float dz = -radius; dz <= radius; dz += step) {
                    if (dx*dx + dy*dy + dz*dz <= radius*radius) {
                        pcl::PointXYZRGB pt; pt.x = center.x + dx; pt.y = center.y + dy; pt.z = center.z + dz;
                        pt.r = r; pt.g = g; pt.b = b;
                        viz_cloud->push_back(pt);
                    }
                }
            }
        }
    };

    // 2. 注入关键点 (红色球体, 半径 15mm)
    for (const auto& kp : res.aligned_keypoints) {
        drawSphere(kp, 15.0f, 255, 0, 0);
    }

    // 3. 注入骨架线 (绿色粗线, 线宽通过并排画多条线模拟，这里为了精简画单根密点线)
    if (res.skeleton_cloud && res.skeleton_cloud->size() > 1) {
        for (size_t i = 0; i < res.skeleton_cloud->size() - 1; ++i) {
            drawLine(res.skeleton_cloud->points[i], res.skeleton_cloud->points[i+1], 0, 255, 0, 3.0f);
        }
    }

    // 4. 注入体高线(蓝色) 和 体宽线(黄色)
    drawLine(res.height_top, res.height_bottom, 0, 0, 255, 2.5f);
    drawLine(res.width_p1, res.width_p2, 255, 255, 0, 2.5f);

    // 5. 注入轮廓线 (胸围橙色, 腰围青色, 臀围品红)
    auto drawContour = [&](PointCloudT::Ptr contour, uint8_t r, uint8_t g, uint8_t b) {
        if (!contour || contour->size() < 3) return;
        for (size_t i = 0; i < contour->size() - 1; ++i) {
            drawLine(contour->points[i], contour->points[i+1], r, g, b, 2.0f);
        }
        drawLine(contour->points.back(), contour->points.front(), r, g, b, 2.0f);
    };
    drawContour(res.chest_contour, 255, 128, 0);
    drawContour(res.waist_contour, 0, 255, 255);
    drawContour(res.hip_contour, 255, 0, 255);

    // 6. 注入地面边界框 (深绿色)
    if (res.ground_polygon && res.ground_polygon->size() == 4) {
        drawContour(res.ground_polygon, 0, 150, 0);
    }

    viz_cloud->width = viz_cloud->size();
    viz_cloud->height = 1;
    viz_cloud->is_dense = true;

    // 保存二进制 PCD
    if (pcl::io::savePCDFileBinary(fileName.toStdString(), *viz_cloud) == 0) {
        log(QString("✅ 可视化点云保存成功: %1").arg(fileName), "SUCCESS");
    } else {
        log("❌ 可视化点云保存失败！", "ERROR");
    }
}
