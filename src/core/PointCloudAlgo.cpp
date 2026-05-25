#include "core/PointCloudAlgo.h"
#include <iostream>

// 点云下采样
PointCloudT::Ptr PointCloudAlgo::downsample(PointCloudT::Ptr cloud_in, float leaf_size_mm) {
    if (!cloud_in || cloud_in->empty()) {
        return nullptr;
    }

    PointCloudT::Ptr cloud_out(new PointCloudT);

    // PCL 的 setLeafSize 默认单位是米 (如果你的点云数据单位是米)
    // 通常点云数据单位如果是米，这里传入 mm 需要除以 1000
    // 如果你的点云数据单位本身就是毫米(比如 Realsense 某些设置)，则不需要除
    // 根据你的之前的代码逻辑，这里假设点云单位是 "米"，所以 mm -> m 需要 / 1000.0f
    // float leaf_size_m = leaf_size_mm / 1000.0f;

    pcl::VoxelGrid<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setLeafSize(leaf_size_mm, leaf_size_mm, leaf_size_mm);
    sor.filter(*cloud_out);

    std::cout << "[Algo] 下采样: " << cloud_in->size() << " -> " << cloud_out->size() 
              << " (Leaf: " << leaf_size_mm << "mm)" << std::endl;

    return cloud_out;
}

// 点云统计滤波移除离群点
PointCloudT::Ptr PointCloudAlgo::statisticalOutlierRemoval(PointCloudT::Ptr cloud_in, int mean_k, double std_dev_mul) {
    if (!cloud_in || cloud_in->empty()) return nullptr;

    PointCloudT::Ptr cloud_out(new PointCloudT);
    
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud_in);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(std_dev_mul);
    sor.filter(*cloud_out);

    std::cout << "[Algo] SOR滤波: " << cloud_in->size() << " -> " << cloud_out->size() 
              << " (MeanK: " << mean_k << ", StdDev: " << std_dev_mul << ")" << std::endl;
    
    return cloud_out;
}

// 基于距离阈值的裁剪
PointCloudT::Ptr PointCloudAlgo::distanceClip(PointCloudT::Ptr cloud_in, float radius_mm) {
    if (!cloud_in || cloud_in->empty()) return nullptr;

    PointCloudT::Ptr cloud_out(new PointCloudT);
    cloud_out->reserve(cloud_in->size());

    // 手动遍历比使用 PassThrough 更灵活（这里是球形裁剪，不仅仅是Z轴）
    // 注意：输入半径单位是 mm，点云单位假设是 m (根据你之前的代码逻辑)
    // float radius_m = radius_mm / 1000.0f;
    float sq_radius = radius_mm * radius_mm;

    for (const auto& pt : cloud_in->points) {
        float sq_dist = pt.x * pt.x + pt.y * pt.y + pt.z * pt.z;
        if (sq_dist <= sq_radius) {
            cloud_out->points.push_back(pt);
        }
    }
    
    // 重新设置宽高（无序点云）
    cloud_out->width = cloud_out->size();
    cloud_out->height = 1;
    cloud_out->is_dense = true; // 可能包含无效点，视具体情况而定，这里手动push通常是dense的

    std::cout << "[Algo] 半径裁剪: " << cloud_in->size() << " -> " << cloud_out->size() 
              << " (Radius: " << radius_mm << "mm)" << std::endl;

    return cloud_out;
}

// 使用变换矩阵对点云进行变换
PointCloudT::Ptr PointCloudAlgo::transformCloud(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d& matrix) {
    if (!cloud_in || cloud_in->empty()) return nullptr;
    PointCloudT::Ptr cloud_out(new PointCloudT);
    pcl::transformPointCloud(*cloud_in, *cloud_out, matrix.cast<float>());
    return cloud_out;
}

// 计算发现的辅助函数（点到面ICP需要法线）
PointCloudNormalT::Ptr PointCloudAlgo::computeNormals(PointCloudT::Ptr cloud, double radius) {
    // 创建法线估计对象 (使用 OpenMP 多核加速)
    pcl::NormalEstimationOMP<PointT, PointNormalT> ne;
    ne.setInputCloud(cloud);

    // 创建 KD-Tree 用于近邻搜索
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    ne.setSearchMethod(tree);

    // 输出数据集
    PointCloudNormalT::Ptr cloud_with_normals(new PointCloudNormalT);

    // 设置搜索半径 (例如 30mm)
    // 这个参数很关键：太小受噪声影响大，太大处理慢且抹平细节
    ne.setRadiusSearch(radius); 

    // 计算法线
    // 注意：PointNormalT 包含 XYZ 和 NormalXYZ，我们需要先把 XYZ 拷过去，再填入 Normal
    // PCL 的 compute 会自动处理 output 的 XYZ 部分吗？不一定，最稳妥是先 copy
    // 但 NormalEstimation 的 compute 函数只填充法线部分。
    // 所以通常做法是先计算法线云(pcl::Normal)，再拼接(concatenateFields)。
    // 这里为了代码简洁，使用直接计算到 PointNormalT 的方式：
    
    // 1. 初始化输出点云的大小和 XYZ 坐标
    cloud_with_normals->resize(cloud->size());
    for(size_t i=0; i<cloud->size(); ++i) {
        cloud_with_normals->points[i].x = cloud->points[i].x;
        cloud_with_normals->points[i].y = cloud->points[i].y;
        cloud_with_normals->points[i].z = cloud->points[i].z;
    }

    // 2. 计算法线
    ne.compute(*cloud_with_normals);
    
    return cloud_with_normals;
}

// ICP 配准
std::pair<PointCloudT::Ptr, Eigen::Matrix4d> PointCloudAlgo::alignICP(
    PointCloudT::Ptr cloud_source,          // 源点云
    PointCloudT::Ptr cloud_target,          // 目标点云
    const Eigen::Matrix4d& init_guess,      // 初始变换猜测
    int max_iter,                           // 最大迭代次数
    double dist_thresh,                     // 距离阈值 (对应点对之间的最大距离)
    int method, // 0: P2Point, 1: P2Plane   // 点到点或点到面 ICP 模式选择
    std::function<void(const QString&, const QString&)> logger)     // 日志回调函数
{
    if (!cloud_source || !cloud_target) return {nullptr, Eigen::Matrix4d::Identity()};

    PointCloudT::Ptr cloud_aligned(new PointCloudT);
    Eigen::Matrix4d final_transform = init_guess;
    double score = 0.0;
    bool hasConverged = false;

    // ==========================================
    // 分支 1: 点到点 (Point-to-Point)
    // ==========================================
    if (method == P2Point) {
        if(logger) logger("正在执行: ICP (点到点)...", "INFO");
        
        pcl::IterativeClosestPoint<PointT, PointT> icp;
        icp.setInputSource(cloud_source);
        icp.setInputTarget(cloud_target);
        icp.setMaxCorrespondenceDistance(dist_thresh);
        icp.setMaximumIterations(max_iter);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-5);
        icp.setUseReciprocalCorrespondences(true); 

        icp.align(*cloud_aligned, init_guess.cast<float>());

        hasConverged = icp.hasConverged();
        score = icp.getFitnessScore();
        final_transform = icp.getFinalTransformation().cast<double>();
    }
    // ==========================================
    // 分支 2: 点到面 (Point-to-Plane)
    // ==========================================
    else if (method == P2Plane) {
        if(logger) logger("正在执行: ICP (点到面) - 正在计算法线...", "INFO");

        // 1. 必须先计算法线 (这是点到面前提)
        // 搜索半径设为 30mm (根据你的猪体点云单位调整，如果是米则设为 0.03)
        // 建议：这个半径最好比点云间距(LeafSize)大 2-3 倍
        double normal_radius = 30.0; 
        
        auto source_normals = computeNormals(cloud_source, normal_radius);
        auto target_normals = computeNormals(cloud_target, normal_radius);

        if(logger) logger("法线计算完成，开始配准...", "INFO");

        // 2. 使用带法线的 ICP 类
        pcl::IterativeClosestPointWithNormals<PointNormalT, PointNormalT> icp;
        icp.setInputSource(source_normals);
        icp.setInputTarget(target_normals);
        
        // 点到面特有参数设置
        icp.setMaxCorrespondenceDistance(dist_thresh);
        icp.setMaximumIterations(max_iter);
        icp.setTransformationEpsilon(1e-8);
        icp.setEuclideanFitnessEpsilon(1e-5);

        icp.setUseReciprocalCorrespondences(true);      // 相互最近邻约束，双向对应可以提高稳定性

        // 3. 执行配准
        // 注意：这里输出的是 PointNormalT 类型的点云
        PointCloudNormalT::Ptr aligned_normals(new PointCloudNormalT);
        icp.align(*aligned_normals, init_guess.cast<float>());

        hasConverged = icp.hasConverged();
        score = icp.getFitnessScore();
        final_transform = icp.getFinalTransformation().cast<double>();

        // 4. 将结果转换回普通 PointT (为了统一输出格式)
        pcl::copyPointCloud(*aligned_normals, *cloud_aligned);
    }

    // ==========================================
    // 结果处理
    // ==========================================
    if (hasConverged) {
        if (logger) {
            QString modeStr = (method == P2Point) ? "P2Point" : "P2Plane";
            QString msg = QString("ICP[%1] 收敛! 分数: %2").arg(modeStr).arg(score, 0, 'f', 5);
            logger(msg, "SUCCESS");
        }
    } else {
        // 未收敛时的回退：手动变换
        final_transform = init_guess;
        pcl::transformPointCloud(*cloud_source, *cloud_aligned, init_guess);
        if (logger) logger("ICP 未收敛，已回退至初始猜测。", "WARN");
    }

    return {cloud_aligned, final_transform};
}


/*
 * 作用：执行 NDT (正态分布变换) 点云精细配准
 * 功能：将源点云向目标点云进行高精度对齐，返回优化后的 4x4 变换矩阵。
 * 实现了什么：弥补硬件标定矩阵在实际物理安装中产生的微小偏差。
 * 怎么实现的：
 * 1. 使用 PCL 的 NormalDistributionsTransform 类。
 * 2. 接收预先标定好的矩阵作为 initial_guess（初始猜测），这是 NDT 收敛的关键。
 * 3. 将目标点云划分为指定 resolution 的体素网格并计算正态分布，然后通过牛顿法迭代寻找最优变换。
 */
Eigen::Matrix4f PointCloudAlgo::refineRegistrationNDT(
    const PointCloudT::Ptr& source_cloud,
    const PointCloudT::Ptr& target_cloud,
    const Eigen::Matrix4f& initial_guess,
    float resolution,
    float step_size,
    int max_iter,
    std::function<void(const QString&, const QString&)> logger)
{
    if (!source_cloud || source_cloud->empty() || !target_cloud || target_cloud->empty()) {
        if(logger) logger("NDT 配准失败：输入点云为空。", "ERROR");
        return initial_guess;
    }

    pcl::NormalDistributionsTransform<PointT, PointT> ndt;
    // 设置 NDT 算法参数
    ndt.setTransformationEpsilon(0.01); // 收敛条件：连续两次变换差异小于此值
    ndt.setStepSize(step_size);         // 莫尔-牛顿法优化的步长
    ndt.setResolution(resolution);      // 网格分辨率 (非常重要，通常设为被测物体尺寸的 1/10 左右)
    ndt.setMaximumIterations(max_iter); // 最大迭代次数

    ndt.setInputSource(source_cloud);
    ndt.setInputTarget(target_cloud);

    PointCloudT::Ptr output_cloud(new PointCloudT);
    
    // 执行配准，必须传入初始猜测矩阵
    ndt.align(*output_cloud, initial_guess);

    if (ndt.hasConverged()) {
        if(logger) {
            QString msg = QString("NDT 配准收敛！得分为: %1 (越小越好)").arg(ndt.getFitnessScore());
            logger(msg, "SUCCESS");
        }
        return ndt.getFinalTransformation();
    } else {
        if(logger) logger("NDT 未能收敛，回退使用原始标定矩阵。", "WARN");
        return initial_guess;
    }
}

// ==========================================
// [新增] 广义 ICP (G-ICP) 配准实现
// ==========================================
std::pair<PointCloudT::Ptr, Eigen::Matrix4d> PointCloudAlgo::alignGICP(
    const PointCloudT::Ptr& source_cloud,
    const PointCloudT::Ptr& target_cloud,
    const Eigen::Matrix4d& initial_guess,
    int max_iter,
    double max_dist,
    double transformation_epsilon,
    std::function<void(const QString&, const QString&)> logger)
{
    PointCloudT::Ptr aligned_cloud(new PointCloudT);
    if (!source_cloud || source_cloud->empty() || !target_cloud || target_cloud->empty()) {
        if (logger) logger("G-ICP 配准失败：输入点云为空。", "ERROR");
        return {source_cloud, initial_guess};
    }

    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> gicp;
    gicp.setInputSource(source_cloud);
    gicp.setInputTarget(target_cloud);

    // 设置算法参数
    gicp.setMaximumIterations(max_iter);
    gicp.setMaxCorrespondenceDistance(max_dist);
    gicp.setTransformationEpsilon(transformation_epsilon);

    // PCL 的 G-ICP 底层只接受 float 精度的初始矩阵
    Eigen::Matrix4f guess_f = initial_guess.cast<float>();
    
    // 启动优化对齐
    gicp.align(*aligned_cloud, guess_f);

    if (gicp.hasConverged()) {
        if (logger) {
            logger(QString("G-ICP 配准收敛！得分: %1").arg(gicp.getFitnessScore()), "SUCCESS");
        }
        // 返回对齐后的点云与优化后的矩阵 (转回 double 精度)
        Eigen::Matrix4f final_transform_f = gicp.getFinalTransformation();
        return {aligned_cloud, final_transform_f.cast<double>()};
    } else {
        if (logger) logger("G-ICP 未能收敛，回退使用初始矩阵。", "WARN");
        // 如果没收敛，就用初始矩阵直接做一次刚体变换返回
        pcl::transformPointCloud(*source_cloud, *aligned_cloud, guess_f);
        return {aligned_cloud, initial_guess};
    }
}


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


BodySizeResults PointCloudAlgo::calculateAllMeasurements(
    PointCloudT::Ptr cloud_body,    
    PointCloudT::Ptr cloud_merged,  
    const std::vector<Eigen::Vector3f>& keypoints_eigen,
    float girth_thick, 
    float skel_step, 
    float skel_radius, 
    float height_angle,
    std::function<void(const QString&, const QString&)> logger)
{
    BodySizeResults res;
    if (!cloud_body || cloud_body->empty() || !cloud_merged || cloud_merged->empty() || keypoints_eigen.size() < 6) {
        if (logger) logger("输入点云为空或关键点不足6个！", "ERROR");
        return res;
    }

    std::vector<PointT> kps;
    for (const auto& vec : keypoints_eigen) {
        kps.push_back(PointT(vec.x(), vec.y(), vec.z()));
    }

    // ==========================================================
    // 1. 基于干净的 Body 点云计算 PCA (极其稳定准确)
    // ==========================================================
    if (logger) logger("1. 基于主体计算 PCA 并同步变换环境空间...", "ALGO");
    PointCloudT::Ptr transformed_body(new PointCloudT);
    Eigen::Affine3f pca_trans = pca_transform(cloud_body, transformed_body, kps);
    
    // 同步变换关键点
    for (auto& pt : kps) {
        pt = pcl::transformPoint(pt, pca_trans);
    }
    res.aligned_keypoints = kps;

    // [核心] 同步变换包含了地面的 Merged 点云
    PointCloudT::Ptr transformed_merged(new PointCloudT);
    pcl::transformPointCloud(*cloud_merged, *transformed_merged, pca_trans);
    
    res.aligned_cloud = transformed_body; // UI 上显示干净的主体

    // ==========================================================
    // 2. 使用干净的 transformed_body 提取骨架
    // ==========================================================
    if (logger) logger("2. 正在提取背部表面骨架曲线...", "ALGO");
    res.skeleton_cloud = compute_surface_back_skeleton(transformed_body, kps, skel_step, skel_radius, 7);
    res.body_length = calculate_body_length(res.skeleton_cloud);

    // ==========================================================
    // 3. 使用 transformed_merged 寻找地面并计算体高
    // ==========================================================
    if (logger) logger("3. 在融合点云中搜索地面计算体高...", "ALGO");
    res.ground_polygon.reset(new PointCloudT);
    
    // 注意：我们将带有地面的 transformed_merged 传入
    res.body_height = calculate_body_height(
        transformed_merged, kps, res.skeleton_cloud, 
        res.height_top, res.height_bottom, res.ground_polygon, height_angle
    );

    // ==========================================================
    // 4. 全部使用 transformed_body 进行高精度尺寸计算
    // ==========================================================
    if (logger) logger("4. 正在计算腰围与腹部极值...", "ALGO");
    PointT p_max_waist;
    res.waist_contour.reset(new PointCloudT);
    res.waist_girth = calculate_waist_girth(transformed_body, girth_thick, kps, p_max_waist, res.skeleton_cloud, res.waist_contour);

    if (logger) logger("5. 基于腹部极值去除四肢...", "ALGO");
    // 此时的 transformed_body 已经没有地面了，去四肢只需要切掉肚子以下即可
    PointCloudT::Ptr cloud_no_limbs = remove_limbs(transformed_body, kps, p_max_waist);

    if (logger) logger("6. 正在计算体宽...", "ALGO");
    res.body_width = calculate_body_width(cloud_no_limbs, kps, res.skeleton_cloud, res.width_p1, res.width_p2);

    if (logger) logger("7. 正在计算胸围...", "ALGO");
    res.chest_contour.reset(new PointCloudT);
    res.chest_girth = calculate_chest_girth(cloud_no_limbs, girth_thick, kps, res.skeleton_cloud, res.chest_contour);

    if (logger) logger("8. 正在计算臀围...", "ALGO");
    res.hip_contour.reset(new PointCloudT);
    res.hip_girth = calculate_hip_girth(cloud_no_limbs, girth_thick, kps, res.skeleton_cloud, res.hip_contour);

    if (logger) logger("体尺计算流水线全部完成！", "SUCCESS");
    return res;
}

/*
 * 作用：计算猪只体高，并提取底面（地面）多边形用于可视化，同时从点云中剔除地面点。
 * 功能：
 * 1. 寻找骨架上距离关键点 P2 最近的点作为体高测量的背部顶点。
 * 2. 提取点云中 Z 值最大的 5% 的点（Top相机俯视，Z值越大代表距离相机越远，即越靠近地面）。
 * 3. 在这 5% 点集中使用迭代 RANSAC 拟合地面平面，通过设定法向量与 Z 轴的夹角阈值过滤错误平面。
 * 4. 计算背部顶点到该平面的垂直投影距离作为“体高”。
 * 5. 计算投影点，并根据主体点云包围盒生成一个表示该地面的 4 顶点多边形图元。
 * 6. 利用找到的平面系数，通过点云滤波（ExtractIndices）从输入的 pig_cloud 中彻底剔除地面点。
 * 怎么实现的：结合 KD-Tree 搜索顶点、PCL SACSegmentation 迭代拟合、矩阵几何投影以及 SampleConsensusModelPlane 点云滤波。
 */
double PointCloudAlgo::calculate_body_height(
    PointCloudT::Ptr pig_cloud, 
    const std::vector<PointT>& all_keypoints,
    const PointCloudT::Ptr& skeleton_cloud, 
    PointT& top_point_out,
    PointT& bottom_point_out,
    PointCloudT::Ptr& plane_polygon_out,
    double angle_threshold_deg)
{
    // 基础校验
    if (!pig_cloud || pig_cloud->empty() || all_keypoints.size() < 6 || !skeleton_cloud || skeleton_cloud->empty()) {
        return 0.0;
    }

    // ===================================================================
    // 1. 在骨架上找到距离 P2 最近的点作为最高点 (p_top)
    // ===================================================================
    const PointT& p2_keypoint = all_keypoints[1]; 
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(skeleton_cloud);
    
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    PointT p_top = p2_keypoint; // 默认使用 P2 作为回退
    
    if (kdtree.nearestKSearch(p2_keypoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
        p_top = skeleton_cloud->points[pointIdxNKNSearch[0]];
    }

    size_t num_points_original = pig_cloud->size();

    // ===================================================================
    // 2. 提取 Z 值最大的 5% 作为地面的候选搜索子集
    // ===================================================================
    PointCloudT::Ptr cloud_copy_for_sort(new PointCloudT(*pig_cloud));
    std::sort(cloud_copy_for_sort->points.begin(), cloud_copy_for_sort->points.end(),
        [](const PointT& a, const PointT& b) {
            return a.z > b.z; // Z值越大越靠前 (靠近地面)
        });

    int num_points = cloud_copy_for_sort->size();
    int num_top_points = static_cast<int>(num_points * 0.05); // 取 5%
    int min_plane_points = 20;
    if (num_top_points < min_plane_points) {
        num_top_points = std::min(min_plane_points, num_points);
    }
    if (num_top_points == 0 && num_points > 0) {
        num_top_points = num_points;
    }

    PointCloudT::Ptr search_subset_cloud(new PointCloudT);
    search_subset_cloud->points.assign(cloud_copy_for_sort->points.begin(), cloud_copy_for_sort->points.begin() + num_top_points);
    search_subset_cloud->width = search_subset_cloud->size();
    search_subset_cloud->height = 1;
    cloud_copy_for_sort.reset(); // 释放内存

    // ===================================================================
    // 3. 迭代 RANSAC 在候选集中寻找最佳地面平面
    // ===================================================================
    pcl::ModelCoefficients::Ptr best_coeffs(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr loop_inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr loop_coeffs(new pcl::ModelCoefficients);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    double distance_threshold = 10.0; // RANSAC 内点距离阈值 (mm)
    seg.setDistanceThreshold(distance_threshold);
    // 使用传入的角度阈值限制平面倾斜度
    seg.setEpsAngle(angle_threshold_deg * M_PI / 180.0);
    seg.setAxis(Eigen::Vector3f::UnitZ());

    pcl::ExtractIndices<PointT> extract;
    bool ground_found = false;
    int max_inliers_found = -1;
    size_t min_points_for_loop = 20;

    while (search_subset_cloud->size() > min_points_for_loop)
    {
        seg.setInputCloud(search_subset_cloud);
        seg.segment(*loop_inliers, *loop_coeffs);

        if (loop_inliers->indices.empty()) {
            break; // 找不到了，退出循环
        }

        float c = loop_coeffs->values[2];
        float c_clamped = std::max(-1.0f, std::min(1.0f, c));
        float angle_with_z_rad = std::acos(std::abs(c_clamped));

        // 检查法向量角度是否满足要求
        if (angle_with_z_rad <= (angle_threshold_deg * M_PI / 180.0)) {
            int current_inlier_count = loop_inliers->indices.size();
            if (current_inlier_count > max_inliers_found) {
                max_inliers_found = current_inlier_count;
                *best_coeffs = *loop_coeffs;
                ground_found = true;
            }
        }

        // 剔除本次找到的平面，继续寻找下一个，直到找到点数最多的合法平面
        extract.setInputCloud(search_subset_cloud);
        extract.setIndices(loop_inliers);
        extract.setNegative(true);

        PointCloudT::Ptr temp_cloud(new PointCloudT);
        extract.filter(*temp_cloud);
        search_subset_cloud->swap(*temp_cloud);
    }

    if (!ground_found || best_coeffs->values.empty() || best_coeffs->values.size() < 4) {
        // 未找到合适平面，返回 0
        return 0.0;
    }

    // 统一法向量方向朝上
    if (best_coeffs->values[2] < 0) {
        best_coeffs->values[0] *= -1; // a
        best_coeffs->values[1] *= -1; // b
        best_coeffs->values[2] *= -1; // c
        best_coeffs->values[3] *= -1; // d
    }
    float a = best_coeffs->values[0];
    float b = best_coeffs->values[1];
    float c = best_coeffs->values[2];
    float d = best_coeffs->values[3];

    // ===================================================================
    // 4. 计算体高与底部投影点
    // ===================================================================
    float signed_dist = a * p_top.x + b * p_top.y + c * p_top.z + d; 
    double body_height = std::abs(signed_dist);

    top_point_out = p_top;
    bottom_point_out.x = p_top.x - signed_dist * a; 
    bottom_point_out.y = p_top.y - signed_dist * b; 
    bottom_point_out.z = p_top.z - signed_dist * c; 

    // ===================================================================
    // 5. 生成用于可视化的地面多边形 (4 个顶点)
    // ===================================================================
    if (!plane_polygon_out) plane_polygon_out.reset(new PointCloudT);
    plane_polygon_out->clear();

    Eigen::Vector3f normal(a, b, c);
    Eigen::Vector4f centroid4f;
    pcl::compute3DCentroid(*pig_cloud, centroid4f);
    Eigen::Vector3f pig_centroid(centroid4f(0), centroid4f(1), centroid4f(2)); 
    
    // 将猪体质心投影到地面上，作为平面的几何中心
    double distance_to_centroid = normal.dot(pig_centroid) + d;
    Eigen::Vector3f plane_center = pig_centroid - distance_to_centroid * normal;
    
    // 计算主体包围盒，用于确定生成多边形的尺寸
    PointT min_pt, max_pt;
    pcl::getMinMax3D(*pig_cloud, min_pt, max_pt);
    double W = (max_pt.x - min_pt.x) * 1.5;
    double H = (max_pt.y - min_pt.y) * 1.5;
    if (W < 2000.0) W = 2000.0; // 设定最小宽度 2000mm
    if (H < 2000.0) H = 2000.0; // 设定最小长度 2000mm
    
    // 构建平面的局部坐标系 (u_vec, v_vec)
    Eigen::Vector3f v_vec = normal.cross(Eigen::Vector3f::UnitX());
    if (v_vec.norm() < 1e-4) { v_vec = normal.cross(Eigen::Vector3f::UnitY()); }
    v_vec.normalize();
    Eigen::Vector3f u_vec = normal.cross(v_vec);

    Eigen::Vector3f poly_p1 = plane_center + (W / 2.0) * u_vec + (H / 2.0) * v_vec;
    Eigen::Vector3f poly_p2 = plane_center - (W / 2.0) * u_vec + (H / 2.0) * v_vec;
    Eigen::Vector3f poly_p3 = plane_center - (W / 2.0) * u_vec - (H / 2.0) * v_vec;
    Eigen::Vector3f poly_p4 = plane_center + (W / 2.0) * u_vec - (H / 2.0) * v_vec;

    plane_polygon_out->push_back(PointT(poly_p1(0), poly_p1(1), poly_p1(2)));
    plane_polygon_out->push_back(PointT(poly_p2(0), poly_p2(1), poly_p2(2)));
    plane_polygon_out->push_back(PointT(poly_p3(0), poly_p3(1), poly_p3(2)));
    plane_polygon_out->push_back(PointT(poly_p4(0), poly_p4(1), poly_p4(2)));

    // ===================================================================
    // 6. 从 pig_cloud 中彻底移除所有的地面点
    // ===================================================================
    pcl::PointIndices::Ptr all_ground_inliers(new pcl::PointIndices);
    pcl::SampleConsensusModelPlane<PointT> model_plane(pig_cloud);
    
    Eigen::VectorXf coeff_vec = Eigen::Map<Eigen::VectorXf>(best_coeffs->values.data(), best_coeffs->values.size());
    // 寻找距离该平面 distance_threshold 以内的所有点
    model_plane.selectWithinDistance(coeff_vec, distance_threshold, all_ground_inliers->indices);

    if (!all_ground_inliers->indices.empty()) {
        pcl::ExtractIndices<PointT> extract_final;
        extract_final.setInputCloud(pig_cloud);
        extract_final.setIndices(all_ground_inliers);
        extract_final.setNegative(true); // 保留非地面点

        PointCloudT::Ptr cloud_without_ground(new PointCloudT);
        extract_final.filter(*cloud_without_ground);
        
        // 交换内存，原 pig_cloud 现在是一个没有地面的干净点云
        pig_cloud->swap(*cloud_without_ground);
    }

    return body_height;
}


// 1D Catmull-Rom样条插值
float PointCloudAlgo::evaluate_catmull_rom_spline_1d(float v0, float v1, float v2, float v3, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    float c0 = -0.5f * t3 + t2 - 0.5f * t;
    float c1 = 1.5f * t3 - 2.5f * t2 + 1.0f;
    float c2 = -1.5f * t3 + 2.0f * t2 + 0.5f * t;
    float c3 = 0.5f * t3 - 0.5f * t2;
    return c0 * v0 + c1 * v1 + c2 * v2 + c3 * v3;
}

// 平滑1D数据 (用于平滑骨架)
std::vector<float> PointCloudAlgo::smooth_data_moving_average(const std::vector<float>& data, int window_size) {
    if (window_size < 3) return data; 
    if (window_size % 2 == 0) window_size++; 

    std::vector<float> smoothed_data = data;
    int half_window = window_size / 2;
    int n = data.size();

    for (int i = 0; i < n; ++i) {
        float sum = 0;
        int count = 0;
        for (int j = -half_window; j <= half_window; ++j) {
            int idx = i + j;
            if (idx >= 0 && idx < n) {
                sum += data[idx];
                count++;
            }
        }
        if (count > 0) {
            smoothed_data[i] = sum / count;
        }
    }
    return smoothed_data;
}

// 查找骨架切线方向
std::pair<PointT, Eigen::Vector3f> PointCloudAlgo::get_skeleton_tangent_at_keypoint(
    const PointT& keypoint,
    const PointCloudT::Ptr& skeleton_cloud)
{
    if (skeleton_cloud->size() < 2) return { keypoint, Eigen::Vector3f::UnitX() };
    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(skeleton_cloud);
    std::vector<int> pointIdxNKNSearch(1);
    std::vector<float> pointNKNSquaredDistance(1);
    if (kdtree.nearestKSearch(keypoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) <= 0) {
        return { keypoint, Eigen::Vector3f::UnitX() };
    }
    int skel_idx = pointIdxNKNSearch[0];
    PointT skeleton_center = skeleton_cloud->points[skel_idx];
    Eigen::Vector3f N_spine;
    if (skel_idx == 0) {
        N_spine = (skeleton_cloud->points[skel_idx + 1].getVector3fMap() - skeleton_center.getVector3fMap()).normalized();
    }
    else if (skel_idx == skeleton_cloud->size() - 1) {
        N_spine = (skeleton_center.getVector3fMap() - skeleton_cloud->points[skel_idx - 1].getVector3fMap()).normalized();
    }
    else {
        N_spine = (skeleton_cloud->points[skel_idx + 1].getVector3fMap() - skeleton_cloud->points[skel_idx - 1].getVector3fMap()).normalized();
    }
    return { skeleton_center, N_spine };
}


// 1. PCA 主成分分析姿态对齐
Eigen::Affine3f PointCloudAlgo::pca_transform(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, const std::vector<PointT>& keypoints_for_orientation)
{
    pcl::PCA<PointT> pca;	
    pca.setInputCloud(cloud_in);	

    Eigen::Vector4f centroid = pca.getMean();	
    Eigen::Matrix3f eigenvectors = pca.getEigenVectors();	

    Eigen::Vector3f new_x = eigenvectors.col(0);
    Eigen::Vector3f new_z = eigenvectors.col(1); 

    if (keypoints_for_orientation.size() >= 6) {
        const PointT& p1 = keypoints_for_orientation[0];	
        const PointT& p6 = keypoints_for_orientation[5];	
        Eigen::Vector3f p1_p6_vec(p6.x - p1.x, p6.y - p1.y, p6.z - p1.z);
        p1_p6_vec.normalize();	

        if (new_x.dot(p1_p6_vec) < 0) {
            new_x = -new_x;
        }
    }

    if (new_z.dot(Eigen::Vector3f::UnitZ()) < 0) {
        new_z = -new_z;
    }

    Eigen::Vector3f new_y = new_z.cross(new_x);
    new_y.normalize();
    new_z = new_x.cross(new_y);
    new_z.normalize();

    Eigen::Matrix3f rotation_corrected;
    rotation_corrected.col(0) = new_x;
    rotation_corrected.col(1) = new_y;
    rotation_corrected.col(2) = new_z;

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(rotation_corrected.transpose());
    transform.translation() = -transform.rotation() * centroid.head<3>();

    pcl::transformPointCloud(*cloud_in, *cloud_out, transform);
    return transform;	
}

// 2. 提取背部表面骨架
PointCloudT::Ptr PointCloudAlgo::compute_surface_back_skeleton(
    const PointCloudT::Ptr& transformed_cloud,      
    const std::vector<PointT>& all_keypoints,       
    float step_mm,             
    float search_radius_mm,    
    int smoothing_window_size) 
{
    PointCloudT::Ptr skeleton_cloud(new PointCloudT);
    if (transformed_cloud->empty() || all_keypoints.size() < 6) return skeleton_cloud;

    const PointT& p1 = all_keypoints[0];
    const PointT& p2 = all_keypoints[1];
    const PointT& p3 = all_keypoints[2];
    const PointT& p4 = all_keypoints[3];
    const PointT& p5 = all_keypoints[4];
    const PointT& p6 = all_keypoints[5];

    std::vector<PointT> path_segments_starts = { p1, p2, p3, p4, p5 };
    std::vector<PointT> path_segments_ends   = { p2, p3, p4, p5, p6 };
    
    std::vector<PointT> raw_surface_points; 
    pcl::KdTreeFLANN<PointT>::Ptr kdtree(new pcl::KdTreeFLANN<PointT>);
    kdtree->setInputCloud(transformed_cloud);

    for (size_t seg = 0; seg < path_segments_starts.size(); ++seg) {    
        const PointT& start_pt = path_segments_starts[seg];
        const PointT& end_pt = path_segments_ends[seg];

        Eigen::Vector3f start_vec = start_pt.getVector3fMap();
        Eigen::Vector3f end_vec = end_pt.getVector3fMap();
        Eigen::Vector3f segment_dir = (end_vec - start_vec);
        float segment_length = segment_dir.norm();
        if (segment_length < 1e-3) continue; 
        segment_dir.normalize();

        int num_steps = static_cast<int>(segment_length / step_mm);
        if (num_steps < 1) num_steps = 1; 

        if (seg == 0) {
            std::vector<int> pointIdxRadiusSearch;  
            std::vector<float> pointRadiusSquaredDistance;  
            PointT start_surface_pt = start_pt; 
            start_surface_pt.z = std::numeric_limits<float>::max();
            bool found_start_neighbor = false;

            if (kdtree->radiusSearch(start_pt, search_radius_mm, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                for (int idx : pointIdxRadiusSearch) {
                    if (transformed_cloud->points[idx].z < start_surface_pt.z) {
                        start_surface_pt = transformed_cloud->points[idx];
                        found_start_neighbor = true;
                    }
                }
            }
            raw_surface_points.push_back(found_start_neighbor ? start_surface_pt : start_pt);
        }

        for (int i = 1; i <= num_steps; ++i) {
            float current_dist = i * step_mm;       
            PointT current_center; 
            Eigen::Vector3f current_center_vec;
            bool is_last_step = (i == num_steps);
            
            if (current_dist >= segment_length - step_mm / 2.0) {   
                current_center = end_pt; 
                current_center_vec = end_vec;
                is_last_step = true; 
                i = num_steps; 
            } else {
                current_center_vec = start_vec + current_dist * segment_dir;
                current_center.x = current_center_vec.x();
                current_center.y = current_center_vec.y();
                current_center.z = current_center_vec.z(); 
            }

            std::vector<int> pointIdxRadiusSearch;
            std::vector<float> pointRadiusSquaredDistance;
            PointT min_z_neighbor_pt = current_center; 
            min_z_neighbor_pt.z = std::numeric_limits<float>::max(); 
            bool found_neighbor = false;

            if (kdtree->radiusSearch(current_center, search_radius_mm, pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0) {
                for (int idx : pointIdxRadiusSearch) {
                    if (transformed_cloud->points[idx].z < min_z_neighbor_pt.z) {
                        min_z_neighbor_pt = transformed_cloud->points[idx];
                        found_neighbor = true;
                    }
                }
            } else {
                std::vector<int> pointIdxNKNSearch(1);
                std::vector<float> pointNKNSquaredDistance(1);
                if (kdtree->nearestKSearch(current_center, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
                    min_z_neighbor_pt = transformed_cloud->points[pointIdxNKNSearch[0]];
                    found_neighbor = true;
                }
            }

            if (found_neighbor) {
                PointT surface_pt = current_center;
                surface_pt.z = min_z_neighbor_pt.z;
                raw_surface_points.push_back(surface_pt);
            } else if (is_last_step) {
                raw_surface_points.push_back(end_pt);
            }
        }
    } 

    if (raw_surface_points.size() < 2) return skeleton_cloud; 

    std::vector<PointT> unique_raw_points;
    unique_raw_points.push_back(raw_surface_points[0]);
    for (size_t i = 1; i < raw_surface_points.size(); ++i) {
        if ((raw_surface_points[i].getVector3fMap() - unique_raw_points.back().getVector3fMap()).norm() > step_mm * 0.1) { 
            unique_raw_points.push_back(raw_surface_points[i]);
        }
    }

    if (unique_raw_points.size() >= 5) { 
        float tail_influence_dist_sq = 80.0f * 80.0f; 
        int reference_idx = -1;
        Eigen::Vector3f p6_vec = p6.getVector3fMap();
        for (int i = unique_raw_points.size() - 1; i >= 0; --i) {
            if ((unique_raw_points[i].getVector3fMap() - p6_vec).squaredNorm() > tail_influence_dist_sq) {
                reference_idx = i;
                break;
            }
        }
        if (reference_idx != -1 && reference_idx < unique_raw_points.size() - 1) {
            float reference_z = unique_raw_points[reference_idx].z;
            float max_allowed_z_deviation = 25.0f;
            for (size_t i = reference_idx + 1; i < unique_raw_points.size(); ++i) {
                if (unique_raw_points[i].z < reference_z - max_allowed_z_deviation) {
                    unique_raw_points[i].z = reference_z;
                }
            }
        }
    }

    if (unique_raw_points.size() < 3) {
        skeleton_cloud->points.assign(unique_raw_points.begin(), unique_raw_points.end());
        skeleton_cloud->width = skeleton_cloud->size();
        skeleton_cloud->height = 1;
        skeleton_cloud->is_dense = true;
        return skeleton_cloud;
    }

    std::vector<float> x_coords, y_raw, z_raw;
    for (const auto& pt : unique_raw_points) {
        x_coords.push_back(pt.x);
        y_raw.push_back(pt.y);
        z_raw.push_back(pt.z);
    }
    std::vector<float> y_smooth = smooth_data_moving_average(y_raw, smoothing_window_size);
    std::vector<float> z_smooth = smooth_data_moving_average(z_raw, smoothing_window_size);

    for (size_t i = 0; i < x_coords.size(); ++i) {
        PointT skel_pt;
        skel_pt.x = x_coords[i];
        skel_pt.y = y_smooth[i];
        skel_pt.z = z_smooth[i];
        skeleton_cloud->push_back(skel_pt);
    }

    if (p1.x < p6.x) {
        std::sort(skeleton_cloud->points.begin(), skeleton_cloud->points.end(), [](const PointT& a, const PointT& b) { return a.x < b.x; });
    } else {
        std::sort(skeleton_cloud->points.begin(), skeleton_cloud->points.end(), [](const PointT& a, const PointT& b) { return a.x > b.x; });
    }

    skeleton_cloud->width = skeleton_cloud->size();
    skeleton_cloud->height = 1;
    skeleton_cloud->is_dense = true;
    return skeleton_cloud;
}

// 3. 计算体长
double PointCloudAlgo::calculate_body_length(const PointCloudT::Ptr& skeleton_cloud_transformed)
{
    if (!skeleton_cloud_transformed || skeleton_cloud_transformed->size() < 2) return 0.0;
    
    double total_length = 0.0;
    for (size_t i = 0; i < skeleton_cloud_transformed->size() - 1; ++i) {
        total_length += (skeleton_cloud_transformed->points[i + 1].getVector3fMap() -
            skeleton_cloud_transformed->points[i].getVector3fMap()).norm();
    }
    return total_length;
}

// 4. 去除四肢
PointCloudT::Ptr PointCloudAlgo::remove_limbs(const PointCloudT::Ptr& pig_cloud, const std::vector<PointT>& keypoints, const PointT& p_max)
{
    float z_max = p_max.z;
    if (keypoints.size() >= 6) { 
        z_max = std::max(z_max, keypoints[2].z); // P3
        z_max = std::max(z_max, keypoints[3].z); // P4
        z_max = std::max(z_max, keypoints[4].z); // P5
        z_max = std::max(z_max, keypoints[5].z); // P6
    }
    PointCloudT::Ptr body_cloud(new PointCloudT);
    body_cloud->reserve(pig_cloud->size());
    for (const auto& point : pig_cloud->points) {
        if (point.z <= z_max) {
            body_cloud->push_back(point);
        }
    }
    return body_cloud;
}



// 5. 计算体宽
double PointCloudAlgo::calculate_body_width(
    const PointCloudT::Ptr& body_cloud,
    const std::vector<PointT>& all_keypoints,
    const PointCloudT::Ptr& skeleton_cloud,
    PointT& min_point_out, 
    PointT& max_point_out) 
{
    if (body_cloud->empty() || skeleton_cloud->size() < 2 || all_keypoints.size() < 6) return 0.0;
    
    const PointT& p2 = all_keypoints[1]; 
    auto tangent_info = get_skeleton_tangent_at_keypoint(p2, skeleton_cloud);
    PointT skeleton_center = tangent_info.first;
    Eigen::Vector3f N_spine = tangent_info.second;

    Eigen::Vector3f V_width = N_spine.cross(Eigen::Vector3f::UnitZ()).normalized();     
    if (V_width.squaredNorm() < 1e-6) {
        V_width = Eigen::Vector3f::UnitY();
    }
    float thickness = 10.0f;
    float min_proj = std::numeric_limits<float>::max();
    float max_proj = -std::numeric_limits<float>::max();
    PointT min_point, max_point;
    Eigen::Vector3f origin_vec = skeleton_center.getVector3fMap();   
       
    for (const auto& point : body_cloud->points) {      
        Eigen::Vector3f vec_to_point = point.getVector3fMap() - origin_vec;
        float dist_to_plane = vec_to_point.dot(N_spine);
        if (std::abs(dist_to_plane) <= thickness / 2.0f) {
            float proj = vec_to_point.dot(V_width);     
            if (proj < min_proj) { min_proj = proj; min_point = point; }
            if (proj > max_proj) { max_proj = proj; max_point = point; }
        }
    }
    if (min_proj == std::numeric_limits<float>::max()) return 0.0; // 防止找不到点

    min_point_out = min_point;
    max_point_out = max_point;
    return max_proj - min_proj;
}

// 从离散点计算周长基础函数
double PointCloudAlgo::calculate_girth_from_points(const PointCloudT::Ptr& points_cloud)
{
    if (points_cloud->size() < 3) return 0.0;
    double total_girth = 0.0;
    for (size_t i = 0; i < points_cloud->size() - 1; ++i) {
        const auto& p1 = points_cloud->points[i];
        const auto& p2 = points_cloud->points[i + 1];
        total_girth += (p1.getVector3fMap() - p2.getVector3fMap()).norm();
    }
    const auto& p_last = points_cloud->points.back();
    const auto& p_first = points_cloud->points.front();
    total_girth += (p_last.getVector3fMap() - p_first.getVector3fMap()).norm();
    return total_girth;
}

// 统一的鲁棒周长插值计算
double PointCloudAlgo::calculate_girth_unified_robust(
    const PointCloudT::Ptr& pig_cloud,           
    const PointCloudT::Ptr& slice_cloud_3d,      
    const PointT& slice_origin,                  
    const Eigen::Vector3f& slice_normal,         
    PointCloudT::Ptr contour_out)                
{
    if (slice_cloud_3d->size() < 20) return 0.0;

    Eigen::Vector3f Z_local = slice_normal;
    Eigen::Vector3f X_local = Z_local.cross(Eigen::Vector3f::UnitZ()).normalized();
    if (X_local.squaredNorm() < 1e-6) {
        X_local = Z_local.cross(Eigen::Vector3f::UnitX()).normalized();
    }
    Eigen::Vector3f Y_local = Z_local.cross(X_local).normalized();

    std::vector<Eigen::Vector2f> slice_points_2d; 
    Eigen::Vector2f centroid(0.0f, 0.0f);         
    Eigen::Vector3f origin_vec = slice_origin.getVector3fMap(); 

    for (const auto& p : slice_cloud_3d->points) {
        Eigen::Vector3f vec = p.getVector3fMap() - origin_vec; 
        float x_2d = vec.dot(X_local);
        float y_2d = vec.dot(Y_local);
        slice_points_2d.push_back(Eigen::Vector2f(x_2d, y_2d)); 
        centroid += slice_points_2d.back();                     
    }
    centroid /= slice_points_2d.size();

    std::vector<PolarPoint> polar_points; 
    for (const auto& p : slice_points_2d) {
        Eigen::Vector2f centered_p = p - centroid; 
        polar_points.push_back({ centered_p.norm(), std::atan2(centered_p.y(), centered_p.x()) });
    }
    std::sort(polar_points.begin(), polar_points.end());

    std::map<float, float> r_of_theta;
    for (const auto& p : polar_points) { r_of_theta[p.theta] = p.r; }
    if (polar_points.empty()) return 0.0;

    r_of_theta[polar_points.front().theta + 2.0 * M_PI] = polar_points.front().r;
    r_of_theta[polar_points.back().theta - 2.0 * M_PI] = polar_points.back().r;

    PointCloudT::Ptr resampled_cloud_3d(new PointCloudT); 
    int num_resamples = 144;                             

    for (int i = 0; i < num_resamples; ++i) {
        float target_theta = -M_PI + (2.0 * M_PI * i / num_resamples);
        auto it_upper = r_of_theta.upper_bound(target_theta);
        if (it_upper == r_of_theta.begin()) { it_upper = std::next(r_of_theta.begin()); }
        if (it_upper == r_of_theta.end()) { it_upper = std::prev(r_of_theta.end()); }
        auto it_lower = std::prev(it_upper);
        auto it_p0 = (it_lower == r_of_theta.begin()) ? it_lower : std::prev(it_lower);
        auto it_p3 = std::next(it_upper);
        if (it_p3 == r_of_theta.end()) { it_p3 = it_upper; }
        
        float r0 = it_p0->second;
        float r1 = it_lower->second; 
        float r2 = it_upper->second; 
        float r3 = it_p3->second;
        float theta1 = it_lower->first;
        float theta2 = it_upper->first;
        
        float t = 0.0f;
        if (theta2 - theta1 > 1e-6) {
            t = (target_theta - theta1) / (theta2 - theta1);
        }
        float resampled_r = evaluate_catmull_rom_spline_1d(r0, r1, r2, r3, t);
        float x_new_2d = centroid.x() + resampled_r * std::cos(target_theta);
        float y_new_2d = centroid.y() + resampled_r * std::sin(target_theta);
        Eigen::Vector3f p_3d_vec = origin_vec + (x_new_2d * X_local) + (y_new_2d * Y_local);
        
        PointT p;
        p.x = p_3d_vec.x();
        p.y = p_3d_vec.y();
        p.z = p_3d_vec.z();
        resampled_cloud_3d->push_back(p); 
    } 

    if (contour_out) {
        *contour_out = *resampled_cloud_3d;
    }
    return calculate_girth_from_points(resampled_cloud_3d);
}

// 计算腰围
double PointCloudAlgo::calculate_waist_girth(
    const PointCloudT::Ptr& pig_cloud,
    float thickness,
    const std::vector<PointT>& all_keypoints,
    PointT& p_max_out,
    const PointCloudT::Ptr& skeleton_cloud,
    PointCloudT::Ptr contour_out) 
{
    if (all_keypoints.size() < 6) return 0.0;
    const PointT& p4 = all_keypoints[3]; 

    auto tangent_info = get_skeleton_tangent_at_keypoint(p4, skeleton_cloud);
    PointT slice_origin = tangent_info.first;
    Eigen::Vector3f slice_normal = tangent_info.second;
    
    PointCloudT::Ptr slice_cloud_3d(new PointCloudT);
    Eigen::Vector3f origin_vec = slice_origin.getVector3fMap();
    for (const auto& point : pig_cloud->points) {
        float dist_to_plane = (point.getVector3fMap() - origin_vec).dot(slice_normal);
        if (std::abs(dist_to_plane) <= thickness / 2.0f) {
            slice_cloud_3d->push_back(point);
        }
    }
    if (slice_cloud_3d->empty()) return 0.0;
    
    p_max_out = slice_cloud_3d->points[0];
    for (const auto& point : slice_cloud_3d->points) {
        if (point.z > p_max_out.z) { p_max_out = point; }
    }

    return calculate_girth_unified_robust(pig_cloud, slice_cloud_3d, slice_origin, slice_normal, contour_out);
}

// 计算胸围
double PointCloudAlgo::calculate_chest_girth(
    const PointCloudT::Ptr& body_cloud,
    float thickness,
    const std::vector<PointT>& all_keypoints,
    const PointCloudT::Ptr& skeleton_cloud,
    PointCloudT::Ptr contour_out) 
{
    if (all_keypoints.size() < 6) return 0.0;
    const PointT& p3 = all_keypoints[2];
    
    auto tangent_info = get_skeleton_tangent_at_keypoint(p3, skeleton_cloud);
    PointT slice_origin = tangent_info.first;
    Eigen::Vector3f slice_normal = tangent_info.second;
    
    PointCloudT::Ptr slice_cloud_3d(new PointCloudT);
    Eigen::Vector3f origin_vec = slice_origin.getVector3fMap();
    for (const auto& point : body_cloud->points) {
        float dist_to_plane = (point.getVector3fMap() - origin_vec).dot(slice_normal);
        if (std::abs(dist_to_plane) <= thickness / 2.0f) {
            slice_cloud_3d->push_back(point);
        }
    }
    if (slice_cloud_3d->empty()) return 0.0;

    return calculate_girth_unified_robust(body_cloud, slice_cloud_3d, slice_origin, slice_normal, contour_out);
}

// 计算臀围
double PointCloudAlgo::calculate_hip_girth(
    const PointCloudT::Ptr& body_cloud,
    float thickness,
    const std::vector<PointT>& all_keypoints,
    const PointCloudT::Ptr& skeleton_cloud,
    PointCloudT::Ptr contour_out) 
{
    if (all_keypoints.size() < 6) return 0.0;
    const PointT& p5 = all_keypoints[4];
    
    auto tangent_info = get_skeleton_tangent_at_keypoint(p5, skeleton_cloud);
    PointT slice_origin = tangent_info.first;
    Eigen::Vector3f slice_normal = tangent_info.second;
    
    PointCloudT::Ptr slice_cloud_3d(new PointCloudT);
    Eigen::Vector3f origin_vec = slice_origin.getVector3fMap();
    for (const auto& point : body_cloud->points) {
        float dist_to_plane = (point.getVector3fMap() - origin_vec).dot(slice_normal);
        if (std::abs(dist_to_plane) <= thickness / 2.0f) {
            slice_cloud_3d->push_back(point);
        }
    }
    if (slice_cloud_3d->empty()) return 0.0;

    return calculate_girth_unified_robust(body_cloud, slice_cloud_3d, slice_origin, slice_normal, contour_out);
}



// 从 RAW 深度图生成点云
PointCloudT::Ptr PointCloudAlgo::convertRawDepthToPointCloud(const QString& rawFilePath, const CameraIntrinsics& intr) {
    PointCloudT::Ptr cloud(new PointCloudT);
    
    QFile file(rawFilePath);
    if (!file.open(QIODevice::ReadOnly)) return nullptr;
    QByteArray data = file.readAll();
    file.close();

    if (data.size() != intr.width * intr.height * 2) return nullptr;

    // 1. 将 RAW 数据加载为 OpenCV 的 16 位单通道图像 (CV_16UC1)
    cv::Mat distortedDepth(intr.height, intr.width, CV_16UC1, (void*)data.constData());
    
    // 2. 构建相机内参矩阵 (Camera Matrix)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        intr.fx, 0, intr.cx,
        0, intr.fy, intr.cy,
        0, 0, 1);
        
    // 3. 构建畸变系数向量 (Distortion Coefficients)
    // 顺序通常为: k1, k2, p1, p2, k3, k4, k5, k6
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 8) << 
        intr.k1, intr.k2, intr.p1, intr.p2, intr.k3, intr.k4, intr.k5, intr.k6);

    // 4. 计算去畸变映射表 (极速查表法)
    cv::Mat map1, map2;
    // 第三个参数是旋转矩阵 R（这里是单位阵），第四个是新的内参矩阵（用原来的即可）
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, 
                                distortedDepth.size(), CV_32FC1, map1, map2);

    // 5. 执行去畸变重映射 (极快，几毫秒内完成)
    cv::Mat undistortedDepth;
    // INTER_NEAREST 防止深度图在边缘处产生插值伪影
    cv::remap(distortedDepth, undistortedDepth, map1, map2, cv::INTER_NEAREST);

    // 6. 使用矫正后的、毫无扭曲的深度图生成高精度点云
    const uint16_t* depthPtr = (const uint16_t*)undistortedDepth.data;
    for (int v = 0; v < intr.height; ++v) {
        for (int u = 0; u < intr.width; ++u) {
            uint16_t z_raw = depthPtr[v * intr.width + u];
            if (z_raw == 0) continue; 

            float z = static_cast<float>(z_raw);
            // 此时的 (u, v) 已经是理想模型下的坐标了，可以直接用简单公式！
            float x = (u - intr.cx) * z / intr.fx;
            float y = (v - intr.cy) * z / intr.fy;

            cloud->push_back(PointT(x, y, z));
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}


// ==========================================
// [新增] 静态相机参数物理数据库 (出厂标定文件大全)
// ==========================================
CameraDeviceParams PointCloudAlgo::getCameraParams(const QString& camKey) {
    static QMap<QString, CameraDeviceParams> db;
    
    // 懒加载：仅在程序首次调用时初始化这庞大的数据库
    if (db.isEmpty()) {
        // 辅助 Lambda 1: 组装外参
        auto makeExt = [](const std::vector<float>& r, const std::vector<float>& t) {
            CameraExtrinsics ext;
            for(int i=0; i<9; ++i) ext.R[i] = r[i];
            for(int i=0; i<3; ++i) ext.T[i] = t[i];
            return ext;
        };

        // 辅助 Lambda 2: 组装内参及畸变 (传入的 dist 顺序严格为 k1, k2, p1, p2, k3, k4, k5, k6)
        auto makeInt = [](int w, int h, float fx, float fy, float cx, float cy, const std::vector<float>& d) {
            CameraIntrinsics i;
            i.width = w; i.height = h; 
            i.fx = fx; i.fy = fy; i.cx = cx; i.cy = cy;
            i.k1 = d[0]; i.k2 = d[1]; i.p1 = d[2]; i.p2 = d[3];
            i.k3 = d[4]; i.k4 = d[5]; i.k5 = d[6]; i.k6 = d[7];
            return i;
        };

        // ----------------- Top (005J) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "Top"; p.serialNumber = "CL8NB43005J";
            p.extrinsics = makeExt({0.994412f, 0.007653f, 0.005581f, -0.008199f, 0.994398f, 0.105381f, -0.004744f, -0.105422f, 0.994416f},
                                   {-32.241280f, -0.717793f, 2.434630f});
            std::vector<float> cd = {0.075733f, -0.106981f, -0.000049f, -0.000155f, 0.044980f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1124.053223f, 1123.337769f, 978.309692f, 528.144592f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  749.368835f,  748.891846f, 652.206482f, 352.096405f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  999.158447f,  998.522461f, 656.275269f, 469.461853f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1498.737671f, 1497.783691f, 1304.412964f, 704.192810f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2248.106445f, 2246.675537f, 1956.619385f, 1056.289185f, cd);

            std::vector<float> dd = {21.223101f, 9.922773f, 0.000056f, -0.000013f, 0.320325f, 21.530495f, 17.081924f, 2.168158f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.488007f, 504.548981f, 323.691956f, 336.099609f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.488007f, 504.548981f, 515.691956f, 516.099609f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.244003f, 252.274490f, 161.845978f, 168.049805f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.244003f, 252.274490f, 257.845978f, 258.049805f, dd);
            db["Top"] = p;
        }

        // ----------------- LB (00SE) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "LB"; p.serialNumber = "CL8NB4300SE";
            p.extrinsics = makeExt({0.994037f, 0.008991f, 0.001626f, -0.009114f, 0.993997f, 0.109027f, -0.000636f, -0.109037f, 0.994038f},
                                   {-32.539291f, -0.761012f, 2.362806f});
            std::vector<float> cd = {0.070404f, -0.097203f, -0.000234f, -0.000113f, 0.038830f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1125.550903f, 1124.178711f, 942.350952f, 538.263062f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  750.367249f,  749.452454f, 628.233948f, 358.842041f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960, 1000.489685f,  999.269958f, 624.311951f, 478.456055f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1500.734497f, 1498.904907f, 1256.467896f, 717.684082f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2251.101807f, 2248.357422f, 1884.701904f, 1076.526123f, cd);

            std::vector<float> dd = {23.504353f, 12.172994f, 0.000100f, -0.000031f, 0.456110f, 23.796473f, 20.142593f, 2.833089f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.507996f, 504.420441f, 331.245239f, 333.824890f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.507996f, 504.420441f, 523.245239f, 513.824890f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.253998f, 252.210220f, 165.622620f, 166.912445f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.253998f, 252.210220f, 261.622620f, 256.912445f, dd);
            db["LB"] = p;
        }

        // ----------------- LT (003W) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "LT"; p.serialNumber = "CL8NB43003W";
            p.extrinsics = makeExt({0.994410f, -0.006397f, 0.002888f, 0.006056f, 0.994393f, 0.105573f, -0.003547f, -0.105553f, 0.994407f},
                                   {-32.140503f, -0.938418f, 2.323613f});
            std::vector<float> cd = {0.070888f, -0.097229f, 0.000126f, -0.000605f, 0.038468f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1122.910767f, 1122.455688f, 954.760742f, 555.962830f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  748.607178f,  748.303772f, 636.507141f, 370.641876f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  998.142883f,  997.738403f, 635.342896f, 494.189178f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1497.214355f, 1496.607544f, 1273.014282f, 741.283752f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2245.821533f, 2244.911377f, 1909.521484f, 1111.925659f, cd);

            std::vector<float> dd = {13.572120f, 6.620557f, 0.000087f, -0.000010f, 0.229236f, 13.892242f, 11.192241f, 1.494901f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.837738f, 504.902527f, 345.253601f, 324.862488f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.837738f, 504.902527f, 537.253601f, 504.862488f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.418869f, 252.451263f, 172.626801f, 162.431244f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.418869f, 252.451263f, 268.626801f, 252.431244f, dd);
            db["LT"] = p;
        }

        // ----------------- RB (00YA) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "RB"; p.serialNumber = "CL8NB4300YA";
            p.extrinsics = makeExt({0.994235f, 0.007331f, 0.005806f, -0.007910f, 0.994224f, 0.107030f, -0.004988f, -0.107071f, 0.994239f},
                                   {-32.348194f, -0.746979f, 2.932103f});
            std::vector<float> cd = {0.073843f, -0.101794f, -0.000170f, 0.000267f, 0.041181f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1120.983398f, 1120.277466f, 957.993774f, 534.084900f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  747.322266f,  746.851624f, 638.662537f, 356.056610f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  996.429688f,  995.802185f, 638.216675f, 474.742126f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1494.644531f, 1493.703247f, 1277.325073f, 712.113220f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2241.966797f, 2240.554932f, 1915.987549f, 1068.169800f, cd);

            std::vector<float> dd = {22.002260f, 10.350877f, 0.000022f, 0.000035f, 0.329348f, 22.304708f, 17.792418f, 2.251812f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 505.278992f, 505.316833f, 327.487793f, 327.828064f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 505.278992f, 505.316833f, 519.487793f, 507.828064f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.639496f, 252.658417f, 163.743896f, 163.914032f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.639496f, 252.658417f, 259.743896f, 253.914032f, dd);
            db["RB"] = p;
        }

        // ----------------- RT (00X6) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "RT"; p.serialNumber = "CL8NB4300X6";
            p.extrinsics = makeExt({0.994587f, -0.007555f, 0.001551f, 0.007353f, 0.994560f, 0.103910f, -0.002327f, -0.103895f, 0.994586f},
                                   {-32.338169f, -1.139261f, 2.579680f});
            std::vector<float> cd = {0.071690f, -0.099641f, 0.000179f, -0.000450f, 0.040123f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1121.462646f, 1120.027466f, 955.431702f, 530.105225f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  747.641785f,  746.684998f, 636.954468f, 353.403473f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  996.855713f,  995.579956f, 635.939270f, 471.204651f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1495.283569f, 1493.369995f, 1273.908936f, 706.806946f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2242.925293f, 2240.054932f, 1910.863403f, 1060.210449f, cd);

            std::vector<float> dd = {25.453939f, 13.172181f, 0.000080f, 0.000028f, 0.483954f, 25.737801f, 21.811607f, 3.050446f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.503265f, 504.484131f, 339.310181f, 336.973572f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.503265f, 504.484131f, 531.310181f, 516.973572f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.251633f, 252.242065f, 169.655090f, 168.486786f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.251633f, 252.242065f, 265.655090f, 258.486786f, dd);
            db["RT"] = p;
        }
    }
    return db.value(camKey);
}

// ==========================================
// [新增] 便捷函数：查表直接提取出特定分辨率的内参
// ==========================================
CameraIntrinsics PointCloudAlgo::getCameraIntrinsics(const QString& camKey, SensorType type, int width, int height) {
    CameraDeviceParams params = getCameraParams(camKey);
    QString resKey = QString("%1x%2").arg(width).arg(height); // e.g. "1024x1024"
    
    if (type == SensorType::COLOR && params.colorIntrinsics.contains(resKey)) {
        return params.colorIntrinsics[resKey];
    } else if (type == SensorType::DEPTH && params.depthIntrinsics.contains(resKey)) {
        return params.depthIntrinsics[resKey];
    }
    
    // 如果没找到，返回一个容错的虚拟内参，防止程序崩溃
    return {width, height, 1000.f, 1000.f, width/2.f, height/2.f, 0,0,0,0,0,0,0,0};
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr PointCloudAlgo::applyTaubinSmoothing(
    const typename pcl::PointCloud<PointT>::ConstPtr& cloud_in,
    int num_iterations,
    double lambda,
    double mu,
    int k_neighbors)
{
    std::cout << "  -> 应用陶宾平滑..." << std::endl;
    typename pcl::PointCloud<PointT>::Ptr cloud_smoothed(new pcl::PointCloud<PointT>(*cloud_in));
    typename pcl::PointCloud<PointT>::Ptr cloud_temp(new pcl::PointCloud<PointT>(*cloud_in));
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    for (int iter = 0; iter < num_iterations; ++iter)
    {
        tree->setInputCloud(cloud_smoothed);
        pcl::copyPointCloud(*cloud_smoothed, *cloud_temp);
        for (size_t i = 0; i < cloud_temp->size(); ++i)
        {
            std::vector<int> indices;
            std::vector<float> sqr_distances;
            tree->nearestKSearch(cloud_temp->points[i], k_neighbors, indices, sqr_distances);
            Eigen::Vector3f centroid(0, 0, 0);
            for (const int& idx : indices) {
                centroid += cloud_temp->points[idx].getVector3fMap();
            }
            centroid /= indices.size();
            Eigen::Vector3f displacement = centroid - cloud_temp->points[i].getVector3fMap();
            cloud_smoothed->points[i].getVector3fMap() += lambda * displacement;
        }
        pcl::copyPointCloud(*cloud_smoothed, *cloud_temp);
        for (size_t i = 0; i < cloud_temp->size(); ++i)
        {
            std::vector<int> indices;
            std::vector<float> sqr_distances;
            tree->nearestKSearch(cloud_temp->points[i], k_neighbors, indices, sqr_distances);
            Eigen::Vector3f centroid(0, 0, 0);
            for (const int& idx : indices) {
                centroid += cloud_temp->points[idx].getVector3fMap();
            }
            centroid /= indices.size();
            Eigen::Vector3f displacement = centroid - cloud_temp->points[i].getVector3fMap();
            cloud_smoothed->points[i].getVector3fMap() += mu * displacement;
        }
    }
    return cloud_smoothed;
}