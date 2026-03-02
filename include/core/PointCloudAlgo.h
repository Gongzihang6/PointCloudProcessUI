#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h> 
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h> // 点到面通常是非线性的，或者用 WithNormals
#include <pcl/features/normal_3d_omp.h> // OMP 加速法线计算
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>  // 用于 pcl::transformPointCloud
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>


#include <functional> // 用于 std::function
#include <QString>

// 定义常用点云类型
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// [新增] 定义带法线的点云类型 (用于点到面 ICP)
using PointNormalT = pcl::PointNormal;
using PointCloudNormalT = pcl::PointCloud<PointNormalT>;

class PointCloudAlgo {
public:
    // [新增] 定义枚举，让代码更具可读性
    enum RegistrationMethod {
        P2Point = 0, // 点到点
        P2Plane = 1  // 点到面
    };

    /**
     * @brief 体素下采样 (Voxel Grid Filter)
     * @param cloud_in 输入点云
     * @param leaf_size_mm 体素大小 (单位: 毫米)
     * @return 下采样后的新点云
     */
    static PointCloudT::Ptr downsample(PointCloudT::Ptr cloud_in, float leaf_size_mm);

    // 2. 统计离群点移除 (SOR)
    static PointCloudT::Ptr statisticalOutlierRemoval(PointCloudT::Ptr cloud_in, int mean_k, double std_dev_mul);

    // 3. 半径/距离裁剪 (保留原点距离小于 radius 的点)
    static PointCloudT::Ptr distanceClip(PointCloudT::Ptr cloud_in, float radius_mm);

    // [新增] 4. 应用 4x4 变换矩阵
    static PointCloudT::Ptr transformCloud(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d& matrix);

    // [新增] 5. ICP 配准 (返回变换后的点云 + 最终的变换矩阵)
    // cloud_source: 待配准点云
    // cloud_target: 目标点云 (Top)
    // init_guess: 初始变换矩阵 (通常是手动输入那个)
    // max_iter: 最大迭代次数
    // dist_thresh: 对应点距离阈值
    // 增加 logger 参数，默认为 nullptr (不传就不打印，兼容性好)
    static std::pair<PointCloudT::Ptr, Eigen::Matrix4d> alignICP(
        PointCloudT::Ptr cloud_source, 
        PointCloudT::Ptr cloud_target, 
        const Eigen::Matrix4d& init_guess,
        int max_iter = 50,
        double dist_thresh = 0.05,
        int method = 0,     // 0: P2Point, 1: P2Plane
        std::function<void(const QString&, const QString&)> logger = nullptr
    );

    // [新增] 6. 提取最大连通主体 (欧式聚类)
    static PointCloudT::Ptr extractLargestCluster(
        PointCloudT::Ptr input_cloud, 
        double tolerance,       // 聚类容差 (mm)
        int min_cluster_size,   // 最小簇点个数
        double plane_thresh,    // RANSAC 平面厚度阈值 (mm)
        std::function<void(const QString&, const QString&)> logger = nullptr
    );


private:
    // 内部辅助函数：将点云转换为带法线的点云 (用于点到面 ICP)
    static PointCloudNormalT::Ptr computeNormals(PointCloudT::Ptr cloud_in, double radius);
};
