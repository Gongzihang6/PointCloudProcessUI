#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <pcl/common/transforms.h> 
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h> // 点到面通常是非线性的，或者用 WithNormals
#include <pcl/features/normal_3d_omp.h> // OMP 加速法线计算
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>  // 用于 pcl::transformPointCloud
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/surface/mls.h>
#include <pcl/common/pca.h>
#include <pcl/common/common.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/registration/ndt.h>
#include <algorithm>
#include <cmath>

#include <functional> // 用于 std::function
#include <QString>
#include <QFile>
#include <QByteArray>
#include <QMap>
#include <opencv2/opencv.hpp>
// 定义常用点云类型
using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;

// 定义带法线的点云类型 (用于点到面 ICP)
using PointNormalT = pcl::PointNormal;
using PointCloudNormalT = pcl::PointCloud<PointNormalT>;

// [新增] 传感器类型枚举
enum class SensorType {
    COLOR,
    DEPTH
};
// [新增] 相机外参 (Extrinsics: Depth to Color)
struct CameraExtrinsics {
    float R[9]; // 3x3 旋转矩阵
    float T[3]; // 1x3 平移向量 (单位: mm)
};
// 定义一个结构体来存储相机内参，用于深度图转点云时使用
struct CameraIntrinsics {
    int width, height;
    float fx, fy, cx, cy;
    float k1, k2, k3, k4, k5, k6;
    float p1, p2;
};
// [新增] 描述单台设备所有参数的数据包
struct CameraDeviceParams {
    QString roleName;     // 视角代号: Top, LB, LT, RB, RT
    QString serialNumber; // 物理序列号: CL8NB...
    
    CameraExtrinsics extrinsics; // 相机外参
    
    // 内参字典 (Key 格式如: "1920x1080" 或 "1024x1024")
    QMap<QString, CameraIntrinsics> colorIntrinsics; 
    QMap<QString, CameraIntrinsics> depthIntrinsics; 
};
// 封装体尺计算相关数据
struct BodySizeResults {
    // 1. 数值结果
    double body_length = 0.0;
    double body_height = 0.0;
    double body_width = 0.0;
    double chest_girth = 0.0;
    double waist_girth = 0.0;
    double hip_girth = 0.0;

    // 2. 变换后的数据 (供可视化用)
    PointCloudT::Ptr aligned_cloud;        // PCA对齐后的完整/主体点云
    std::vector<PointT> aligned_keypoints; // PCA对齐后的6个关键点
    PointCloudT::Ptr skeleton_cloud;       // 骨架点云

    // 3. 测量线/轮廓的端点图元 (供画线用)
    PointT height_top, height_bottom;      // 体高线段端点
    PointT width_p1, width_p2;             // 体宽线段端点
    PointCloudT::Ptr chest_contour;        // 胸围轮廓线
    PointCloudT::Ptr waist_contour;        // 腰围轮廓线
    PointCloudT::Ptr hip_contour;          // 臀围轮廓线
    PointCloudT::Ptr ground_polygon;       // 地面表示多边形 (4个顶点)
};


// 辅助结构体，用于周长计算时的极坐标点排序
struct PolarPoint {
    float r; // 半径
    float theta; // 角度 (弧度)
    bool operator < (const PolarPoint& other) const {
        return theta < other.theta;
    }
};


class PointCloudAlgo {
public:
    // 定义枚举，让代码更具可读性
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

    // 4. 应用 4x4 变换矩阵
    static PointCloudT::Ptr transformCloud(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d& matrix);

    // 5. ICP 配准 (返回变换后的点云 + 最终的变换矩阵)
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

    // NDT 配准 (正态分布变换，适合大规模点云)
    static Eigen::Matrix4f refineRegistrationNDT(
        const PointCloudT::Ptr& source_cloud,
        const PointCloudT::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        float resolution,
        float step_size,
        int max_iter,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    // [新增] G-ICP 配准函数声明
    static std::pair<PointCloudT::Ptr, Eigen::Matrix4d> alignGICP(
        const PointCloudT::Ptr& source_cloud,
        const PointCloudT::Ptr& target_cloud,
        const Eigen::Matrix4d& initial_guess,
        int max_iter,
        double max_dist,
        double transformation_epsilon,
        std::function<void(const QString&, const QString&)> logger = nullptr);


    // 6. 提取最大连通主体 (欧式聚类)
    static PointCloudT::Ptr extractLargestCluster(
        PointCloudT::Ptr input_cloud, 
        double tolerance,       // 聚类容差 (mm)
        int min_cluster_size,   // 最小簇点个数
        double plane_thresh,    // RANSAC 平面厚度阈值 (mm)
        std::function<void(const QString&, const QString&)> logger = nullptr
    );

    // 整合的体尺计算主干函数
    static BodySizeResults calculateAllMeasurements(
        PointCloudT::Ptr cloud_body,    // 干净的主体点云
        PointCloudT::Ptr cloud_merged,  // 包含地面的融合点云
        const std::vector<Eigen::Vector3f>& keypoints_eigen,
        float girth_thick, 
        float skel_step, 
        float skel_radius, 
        float height_angle,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    // [新增] 从 16-bit RAW 深度图转换为点云
    static PointCloudT::Ptr convertRawDepthToPointCloud(
        const QString& rawFilePath, 
        const CameraIntrinsics& intr);

    // [新增] 获取某一台相机的【全套】物理参数
    static CameraDeviceParams getCameraParams(const QString& camKey);
    // [新增] 便捷函数：直接获取特定类型、特定分辨率下的内参
    static CameraIntrinsics getCameraIntrinsics(const QString& camKey, SensorType type, int width, int height);

private:
    // 内部辅助函数：将点云转换为带法线的点云 (用于点到面 ICP)
    static PointCloudNormalT::Ptr computeNormals(PointCloudT::Ptr cloud_in, double radius);

    static Eigen::Affine3f pca_transform(PointCloudT::Ptr cloud_in, PointCloudT::Ptr& cloud_out, const std::vector<PointT>& kps);
    static PointCloudT::Ptr compute_surface_back_skeleton(const PointCloudT::Ptr& cloud, const std::vector<PointT>& kps, float step, float radius, int smooth_win);
    static double calculate_body_length(const PointCloudT::Ptr& skel);
    static double calculate_body_height(PointCloudT::Ptr pig_cloud, const std::vector<PointT>& kps, const PointCloudT::Ptr& skel, PointT& top_pt, PointT& bottom_pt, PointCloudT::Ptr& ground_poly, double angle_thresh);
    static PointCloudT::Ptr remove_limbs(const PointCloudT::Ptr& cloud, const std::vector<PointT>& kps, const PointT& p_max);
    static double calculate_body_width(const PointCloudT::Ptr& cloud, const std::vector<PointT>& kps, const PointCloudT::Ptr& skel, PointT& p1, PointT& p2);
    static double calculate_girth_from_points(const PointCloudT::Ptr& points_cloud);
    static double calculate_girth_unified_robust(const PointCloudT::Ptr& pig_cloud, const PointCloudT::Ptr& slice_cloud_3d, const PointT& slice_origin, const Eigen::Vector3f& slice_normal, PointCloudT::Ptr contour_out);
    static double calculate_waist_girth(const PointCloudT::Ptr& pig_cloud, float thickness, const std::vector<PointT>& all_keypoints, PointT& p_max_out, const PointCloudT::Ptr& skeleton_cloud, PointCloudT::Ptr contour_out);
    static double calculate_chest_girth(const PointCloudT::Ptr& body_cloud, float thickness, const std::vector<PointT>& all_keypoints, const PointCloudT::Ptr& skeleton_cloud, PointCloudT::Ptr contour_out);
    static double calculate_hip_girth(const PointCloudT::Ptr& body_cloud, float thickness, const std::vector<PointT>& all_keypoints, const PointCloudT::Ptr& skeleton_cloud, PointCloudT::Ptr contour_out);
    static float evaluate_catmull_rom_spline_1d(float v0, float v1, float v2, float v3, float t);
    static std::vector<float> smooth_data_moving_average(const std::vector<float>& data, int window_size);
    static std::pair<PointT, Eigen::Vector3f> get_skeleton_tangent_at_keypoint(const PointT& keypoint, const PointCloudT::Ptr& skeleton_cloud);
};



