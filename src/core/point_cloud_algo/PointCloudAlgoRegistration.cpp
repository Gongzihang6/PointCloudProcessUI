/*
 * 文件说明：
 * 该文件实现 `PointCloudAlgo` 的点云配准能力。
 *
 * 包含内容：
 * 1. ICP 点到点 / 点到面配准；
 * 2. NDT 精配准；
 * 3. G-ICP 配准。
 */
#include "core/point_cloud_algo/PointCloudAlgoInternal.h"

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
