/*
 * 文件说明：
 * 该文件实现 `PointCloudAlgo` 的点云预处理基础能力。
 *
 * 包含内容：
 * 1. 下采样、离群点去除、距离裁剪、刚体变换；
 * 2. 点到面 ICP 所需的法线估计辅助函数。
 */
#include "core/point_cloud_algo/PointCloudAlgoInternal.h"

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
