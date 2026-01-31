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

    // 2. [新增] 统计离群点移除 (SOR)
    static PointCloudT::Ptr statisticalOutlierRemoval(PointCloudT::Ptr cloud_in, int mean_k, double std_dev_mul);

    // 3. [新增] 半径/距离裁剪 (保留原点距离小于 radius 的点)
    static PointCloudT::Ptr distanceClip(PointCloudT::Ptr cloud_in, float radius_mm);
};
