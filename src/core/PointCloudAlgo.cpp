#include "core/PointCloudAlgo.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
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