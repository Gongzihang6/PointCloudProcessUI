#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// 定义常用点云类型
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

class PointCloudAlgo {
public:
    /**
     * @brief 体素下采样 (Voxel Grid Filter)
     * @param cloud_in 输入点云
     * @param leaf_size_mm 体素大小 (单位: 毫米)
     * @return 下采样后的新点云
     */
    static PointCloudT::Ptr downsample(PointCloudT::Ptr cloud_in, float leaf_size_mm);
};
