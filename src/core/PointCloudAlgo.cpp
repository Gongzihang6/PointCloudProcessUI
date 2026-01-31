#include "core/PointCloudAlgo.h"
#include <pcl/filters/voxel_grid.h>
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