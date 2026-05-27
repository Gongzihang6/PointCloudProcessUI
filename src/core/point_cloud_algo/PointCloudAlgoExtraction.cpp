/*
 * 文件说明：
 * 该文件实现 `PointCloudAlgo` 的主体提取与表面平滑流程。
 *
 * 包含内容：
 * 1. CropBox 空间裁剪；
 * 2. 欧式聚类 / 区域生长主体提取；
 * 3. RANSAC 平面剔除；
 * 4. SOR、陶宾平滑与 MLS 平滑。
 */
#include "core/point_cloud_algo/PointCloudAlgoInternal.h"

/*
 * -------------------------------------------------------------------------
 * 代码作用：优化版的猪主体点云提取与平滑算法。
 * 功能：从包含复杂背景（护栏、过道、地面）的原始点云中，精确、干净地提取出连续完整的猪三维表面点云。
 * 实现了什么：
 * 1. 空间裁剪 (CropBox)：基于预设包围盒快速剔除视野外的大面积背景干扰。
 * 2. 地面剔除 (RANSAC)：利用随机采样一致性提取并移除平面特征（过道地面）。
 * 3. 主体提取 (Euclidean Clustering)：利用点之间的空间距离连通性，提取出点数最大的独立簇。
 * 4. 去噪与平滑 (SOR + MLS)：滤除空间离群飞点，并通过移动最小二乘法重建出极其平滑的表面。
 * 怎么实现的：
 * - 在流水线最前端插入 pcl::CropBox 滤波器，使用预计算的极值加上适当 Padding 定义检测空间。
 * - 将粗筛后的紧凑点云送入 RANSAC 拟合地面模型并提取非地面点（Negative = true）。
 * - 构建 KdTree 加速，运用欧式聚类抓取最大的连通域（假设猪占据了检测空间内最大的非地面体积）。
 * - 依次串联 StatisticalOutlierRemoval 和 MovingLeastSquares 提升最终点云质量，为后续体型测量提供高质量数据。
 * -------------------------------------------------------------------------
 */

PointCloudT::Ptr PointCloudAlgo::extractLargestCluster(
    PointCloudT::Ptr input_cloud, 
    const ExtractionParams& params,
    std::function<void(const QString&, const QString&)> logger)
{
    if (!input_cloud || input_cloud->empty()) return nullptr;

    // =======================================================
    // 步骤 0: 共用步骤 - CropBox 空间裁剪
    // =======================================================
    if(logger) logger("0. 执行 CropBox 空间裁剪...", "ALGO");
    PointCloudT::Ptr cloud_cropped(new PointCloudT);
    pcl::CropBox<PointT> box_filter;
    box_filter.setMin(Eigen::Vector4f(params.boxMinX, params.boxMinY, params.boxMinZ, 1.0f)); 
    box_filter.setMax(Eigen::Vector4f(params.boxMaxX, params.boxMaxY, params.boxMaxZ, 1.0f)); 
    // [新增] 设置包围盒的旋转
    // setRotation 接收的是 (Roll, Pitch, Yaw) 的弧度值。我们只绕 Z 轴旋转。
    float yaw_rad = params.boxRotZ * M_PI / 180.0f;
    box_filter.setRotation(Eigen::Vector3f(0.0f, 0.0f, yaw_rad));

    box_filter.setInputCloud(input_cloud);
    box_filter.filter(*cloud_cropped);

    if (cloud_cropped->empty()) {
        if(logger) logger("CropBox 裁剪后点云为空，请检查包围盒坐标！", "ERROR");
        return nullptr;
    }

    std::vector<pcl::PointIndices> clusters;
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_cropped);

    // =======================================================
    // 步骤 1: 算法分支选择
    // =======================================================
    if (params.methodIndex == 0) {
        // --- 分支 A: 欧式聚类 (Euclidean Clustering) ---
        if(logger) logger(QString("1. 执行欧式聚类 (容差: %1 mm)...").arg(params.euclideanTolerance), "ALGO");
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(params.euclideanTolerance); 
        ec.setMinClusterSize(params.minClusterSize);
        ec.setMaxClusterSize(cloud_cropped->size());
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_cropped);
        ec.extract(clusters);
    } 
    else if (params.methodIndex == 1) {
        // --- 分支 B: 区域生长 (Region Growing) ---
        if(logger) logger("1. 多线程计算法线并执行区域生长聚类...", "ALGO");
        pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;
        ne.setNumberOfThreads(8);
        ne.setInputCloud(cloud_cropped);
        ne.setSearchMethod(tree);
        ne.setKSearch(params.rgNeighbors);
        ne.compute(*normals);

        pcl::RegionGrowing<PointT, pcl::Normal> reg;
        reg.setMinClusterSize(params.minClusterSize);   
        reg.setMaxClusterSize(cloud_cropped->size());
        reg.setSearchMethod(tree);
        reg.setNumberOfNeighbours(params.rgNeighbors); 
        reg.setInputCloud(cloud_cropped);
        reg.setInputNormals(normals);
        reg.setSmoothnessThreshold(params.rgSmoothness / 180.0 * M_PI); // 转弧度
        reg.setCurvatureThreshold(params.rgCurvature);
        reg.extract(clusters);
    }

    if (clusters.empty()) {
        if(logger) logger("聚类分割失败：未找到符合条件的点云簇！", "ERROR");
        return nullptr;
    }

    // =======================================================
    // 步骤 2: 提取最大的点云簇 (猪体主体)
    // =======================================================
    PointCloudT::Ptr body_cloud(new PointCloudT);
    for (const auto& idx : clusters[0].indices) {
        body_cloud->points.push_back(cloud_cropped->points[idx]);
    }


    // =======================================================
    // [新增] 步骤 2.5: RANSAC 二次平面剔除与半空间“地下”噪点隔离
    // =======================================================
    if (params.useRansac && !body_cloud->empty()) {
        if(logger) logger(QString("2.5 执行 RANSAC 平面与半空间剔除 (距离阈值: %1mm, 角度限制: %2°)...")
                          .arg(params.ransacDistThresh).arg(params.ransacAngleThresh), "ALGO");
        
        pcl::SACSegmentation<PointT> seg;
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE); 
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f)); 
        seg.setEpsAngle(params.ransacAngleThresh * M_PI / 180.0); 
        seg.setMaxIterations(1000); 
        seg.setDistanceThreshold(params.ransacDistThresh);

        seg.setInputCloud(body_cloud);
        seg.segment(*inliers, *coefficients);

        if (!inliers->indices.empty()) {
            // 【阶段A】常规剔除平面本身 (地面上的点)
            pcl::ExtractIndices<PointT> extract;
            extract.setInputCloud(body_cloud);
            extract.setIndices(inliers);
            extract.setNegative(true); 
            
            PointCloudT::Ptr cloud_no_ground(new PointCloudT);
            extract.filter(*cloud_no_ground);
            
            if (!cloud_no_ground->empty()) {
                // 提取平面方程参数: Ax + By + Cz + D = 0
                float a = coefficients->values[0];
                float b = coefficients->values[1];
                float c = coefficients->values[2];
                float d = coefficients->values[3];

                // 【阶段B】半空间剔除 (Half-Space Culling) - 分离平面两侧的点
                PointCloudT::Ptr cloud_side_positive(new PointCloudT);
                PointCloudT::Ptr cloud_side_negative(new PointCloudT);

                for (const auto& pt : cloud_no_ground->points) {
                    // 将点代入平面方程，判断其在平面的哪一侧
                    float distance_sign = a * pt.x + b * pt.y + c * pt.z + d;
                    
                    if (distance_sign > 0) {
                        cloud_side_positive->points.push_back(pt);
                    } else {
                        cloud_side_negative->points.push_back(pt);
                    }
                }

                // 【阶段C】保留点数多的一侧（猪主体），抛弃点数少的一侧（地下噪点）
                if (cloud_side_positive->points.size() > cloud_side_negative->points.size()) {
                    body_cloud = cloud_side_positive;
                    if(logger) logger(QString("-> 半空间剔除完成: 保留正侧(猪体) %1 点，剔除负侧(地下噪点) %2 点。")
                                      .arg(cloud_side_positive->size()).arg(cloud_side_negative->size()), "INFO");
                } else {
                    body_cloud = cloud_side_negative;
                    if(logger) logger(QString("-> 半空间剔除完成: 保留负侧(猪体) %1 点，剔除正侧(地下噪点) %2 点。")
                                      .arg(cloud_side_negative->size()).arg(cloud_side_positive->size()), "INFO");
                }

                // 规范化最终留存的点云属性
                body_cloud->width = body_cloud->points.size();
                body_cloud->height = 1;
                body_cloud->is_dense = true;
            }
        } else {
            if(logger) logger("-> 未找到符合角度限制的连续平面，跳过半空间剔除。", "WARN");
        }
    }

    if(logger) logger(QString("主体提取成功。点数: %1").arg(body_cloud->size()), "INFO");

    // =======================================================
    // 步骤 3: 共用步骤 - SOR去噪 -> 陶宾平滑 -> MLS平滑
    // =======================================================
    if(logger) logger("2. 执行 SOR 去噪、陶宾平滑与 MLS 表面平滑...", "ALGO");
    
    // --- 阶段 3.1: 统计滤波 (SOR) 剔除离群点 ---
    PointCloudT::Ptr cloud_sor(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(body_cloud);
    sor.setMeanK(50);
    sor.setStddevMulThresh(3.0);
    sor.filter(*cloud_sor);
    // 可选：如果需要在UI日志中显示每步点数，可以取消下面注释
    // if(logger) logger(QString("SOR滤波完成。剩余点数: %1").arg(cloud_sor->size()), "INFO");

    // --- 阶段 3.2: 陶宾平滑 (Taubin Smoothing) 无收缩去噪 ---
    // 调用你现有的 applyTaubinSmoothing 函数
    int taubin_iterations = 5;
    double taubin_lambda = 0.6;
    double taubin_mu = -0.65;
    
    PointCloudT::Ptr cloud_taubin = applyTaubinSmoothing<PointT>(
        cloud_sor, 
        taubin_iterations, 
        taubin_lambda, 
        taubin_mu);
    // if(logger) logger(QString("陶宾平滑完成。剩余点数: %1").arg(cloud_taubin->size()), "INFO");

    // --- 阶段 3.3: 移动最小二乘平滑 (MLS) 与曲面重建 ---
    if(logger) logger(QString("3.3 执行 MLS 表面平滑 (搜索半径: %1 mm)...").arg(params.mlsSearchRadius), "ALGO");
    
    PointCloudT::Ptr cloud_smoothed(new PointCloudT);
    pcl::search::KdTree<PointT>::Ptr mls_tree(new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls;
    
    mls.setComputeNormals(true); 
    mls.setInputCloud(cloud_taubin); 
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(mls_tree);
    
    // 1. 设置从 UI 传过来的搜索半径
    mls.setSearchRadius(params.mlsSearchRadius); 

    // 2. 动态判断是否开启上采样补洞
    if (params.useMlsUpsampling) {
        if(logger) logger(QString(" -> 开启局部平面上采样 (补孔半径: %1 mm, 步长: %2 mm)")
                          .arg(params.mlsUpsamplingRadius).arg(params.mlsUpsamplingStep), "INFO");
        
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::SAMPLE_LOCAL_PLANE);
        mls.setUpsamplingRadius(params.mlsUpsamplingRadius); 
        mls.setUpsamplingStepSize(params.mlsUpsamplingStep); 
    } else {
        mls.setUpsamplingMethod(pcl::MovingLeastSquares<PointT, PointT>::NONE);
    }

    mls.process(*cloud_smoothed);

    cloud_smoothed->width = cloud_smoothed->points.size();
    cloud_smoothed->height = 1;
    cloud_smoothed->is_dense = true;


    if(logger) logger("精细提取与三阶段平滑处理全部完成！", "SUCCESS");
    
    return cloud_smoothed;
}
