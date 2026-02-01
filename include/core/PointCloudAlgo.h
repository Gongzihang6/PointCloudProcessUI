#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h> 
#include <pcl/registration/icp.h>
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

    // [新增] 4. 应用 4x4 变换矩阵
    static PointCloudT::Ptr transformCloud(PointCloudT::Ptr cloud_in, const Eigen::Matrix4f& matrix);

    // [新增] 5. ICP 配准 (返回变换后的点云 + 最终的变换矩阵)
    // cloud_source: 待配准点云
    // cloud_target: 目标点云 (Top)
    // init_guess: 初始变换矩阵 (通常是手动输入那个)
    // max_iter: 最大迭代次数
    // dist_thresh: 对应点距离阈值
    static std::pair<PointCloudT::Ptr, Eigen::Matrix4f> alignICP(
        PointCloudT::Ptr cloud_source, 
        PointCloudT::Ptr cloud_target, 
        const Eigen::Matrix4f& init_guess,
        int max_iter = 50,
        double dist_thresh = 0.05
    );
};
