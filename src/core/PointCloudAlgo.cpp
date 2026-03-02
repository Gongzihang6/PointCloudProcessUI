#include "core/PointCloudAlgo.h"
#include <iostream>

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

// =========================================================
// 辅助函数：计算法线
// =========================================================
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
    PointCloudT::Ptr cloud_source,
    PointCloudT::Ptr cloud_target,
    const Eigen::Matrix4d& init_guess,
    int max_iter,
    double dist_thresh,
    int method, // 0: P2Point, 1: P2Plane
    std::function<void(const QString&, const QString&)> logger)
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


// 提取最大连通主体 (欧式聚类)
PointCloudT::Ptr PointCloudAlgo::extractLargestCluster(
    PointCloudT::Ptr input_cloud, 
    double tolerance, 
    int min_cluster_size,
    double plane_thresh,
    std::function<void(const QString&, const QString&)> logger)
{
    if (!input_cloud || input_cloud->empty()) return nullptr;

    // =======================================================
    // 步骤 1: RANSAC 地面/平面剔除
    // =======================================================
    if(logger) logger(QString("1. 开始 RANSAC 拟合地面 (阈值: %1mm)...").arg(plane_thresh), "ALGO");
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<PointT> seg;
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE); // 拟合平面
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(1000);
    seg.setDistanceThreshold(plane_thresh); // 到平面距离小于此值的点被视为地面
    seg.setInputCloud(input_cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
        if(logger) logger("未能估算出现有平面模型，跳过地面剔除。", "WARN");
    }

    PointCloudT::Ptr cloud_no_ground(new PointCloudT);
    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(input_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);      // [关键] 设置为 true，表示提取“除平面以外”的点
    extract.filter(*cloud_no_ground);

    if(logger) logger(QString("地面已剔除。剩余点数: %1").arg(cloud_no_ground->size()), "INFO");

    // =======================================================
    // 步骤 2: 欧式聚类 (寻找最大主体)
    // =======================================================
    if(logger) logger("2. 执行欧式聚类以提取主体...", "ALGO");
    
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud_no_ground);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(tolerance); 
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(cloud_no_ground->size());
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_no_ground);
    ec.extract(cluster_indices);

    if (cluster_indices.empty()) {
        if(logger) logger("聚类失败：未找到符合条件的独立主体！", "ERROR");
        return nullptr;
    }

    PointCloudT::Ptr body_cloud(new PointCloudT);
    for (const auto& idx : cluster_indices[0].indices) { // 索引0即为最大簇
        body_cloud->points.push_back(cloud_no_ground->points[idx]);
    }
    
    if(logger) logger(QString("主体提取成功。点数: %1").arg(body_cloud->size()), "INFO");

    // =======================================================
    // 步骤 3: SOR 滤波 (去除悬浮的离群噪点)
    // =======================================================
    if(logger) logger("3. 执行 SOR 滤波去噪...", "ALGO");
    PointCloudT::Ptr cloud_sor(new PointCloudT);
    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(body_cloud);
    sor.setMeanK(50);             // 邻域点数
    sor.setStddevMulThresh(3.0);  // 标准差倍数 (1.0 比较严格)
    sor.filter(*cloud_sor);

    // =======================================================
    // 步骤 4: MLS 平滑 (让猪体表面变得像丝绸一样光滑)
    // =======================================================
    if(logger) logger("4. 执行 MLS 表面平滑...", "ALGO");
    PointCloudT::Ptr cloud_smoothed(new PointCloudT);
    pcl::search::KdTree<PointT>::Ptr mls_tree(new pcl::search::KdTree<PointT>);
    pcl::MovingLeastSquares<PointT, PointT> mls;
    
    mls.setComputeNormals(false); 
    mls.setInputCloud(cloud_sor);
    mls.setPolynomialOrder(2);
    mls.setSearchMethod(mls_tree);
    mls.setSearchRadius(50.0);      // 搜索半径 30mm，视你的点云密度而定
    mls.process(*cloud_smoothed);

    cloud_smoothed->width = cloud_smoothed->points.size();
    cloud_smoothed->height = 1;
    cloud_smoothed->is_dense = true;

    if(logger) logger("精细提取与平滑处理全部完成！", "SUCCESS");

    return cloud_smoothed;
}