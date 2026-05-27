/*
 * 文件说明：
 * 该文件声明统一的点云算法入口类 `PointCloudAlgo`。
 *
 * 重构说明：
 * 1. 保持原有 `core/PointCloudAlgo.h` 对外包含路径与 `PointCloudAlgo` 静态接口不变；
 * 2. 将公共类型拆分到 `PointCloudAlgoTypes.h`，将模板实现拆分到 `PointCloudAlgoSmoothing.tpp`；
 * 3. 让主头文件聚焦算法接口声明，降低页面与批处理模块阅读时的认知负担。
 */
#pragma once

#include <functional>
#include <utility>
#include <vector>

#include <Eigen/Dense>

#include "core/point_cloud_algo/PointCloudAlgoTypes.h"

class PointCloudAlgo {
public:
    // 定义枚举，让代码更具可读性。
    enum RegistrationMethod {
        P2Point = 0,
        P2Plane = 1
    };

    // 点云预处理相关接口。
    static PointCloudT::Ptr downsample(PointCloudT::Ptr cloud_in, float leaf_size_mm);
    static PointCloudT::Ptr statisticalOutlierRemoval(PointCloudT::Ptr cloud_in, int mean_k, double std_dev_mul);
    static PointCloudT::Ptr distanceClip(PointCloudT::Ptr cloud_in, float radius_mm);
    static PointCloudT::Ptr transformCloud(PointCloudT::Ptr cloud_in, const Eigen::Matrix4d& matrix);

    // 点云配准相关接口。
    static std::pair<PointCloudT::Ptr, Eigen::Matrix4d> alignICP(
        PointCloudT::Ptr cloud_source,
        PointCloudT::Ptr cloud_target,
        const Eigen::Matrix4d& init_guess,
        int max_iter = 50,
        double dist_thresh = 0.05,
        int method = 0,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    static Eigen::Matrix4f refineRegistrationNDT(
        const PointCloudT::Ptr& source_cloud,
        const PointCloudT::Ptr& target_cloud,
        const Eigen::Matrix4f& initial_guess,
        float resolution,
        float step_size,
        int max_iter,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    static std::pair<PointCloudT::Ptr, Eigen::Matrix4d> alignGICP(
        const PointCloudT::Ptr& source_cloud,
        const PointCloudT::Ptr& target_cloud,
        const Eigen::Matrix4d& initial_guess,
        int max_iter,
        double max_dist,
        double transformation_epsilon,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    // 主体提取相关接口。
    static PointCloudT::Ptr extractLargestCluster(
        PointCloudT::Ptr input_cloud,
        const ExtractionParams& params,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    // 体尺测量相关接口。
    static BodySizeResults calculateAllMeasurements(
        PointCloudT::Ptr cloud_body,
        PointCloudT::Ptr cloud_merged,
        const std::vector<Eigen::Vector3f>& keypoints_eigen,
        float girth_thick,
        float skel_step,
        float skel_radius,
        float height_angle,
        std::function<void(const QString&, const QString&)> logger = nullptr);

    // 深度图与相机参数相关接口。
    static PointCloudT::Ptr convertRawDepthToPointCloud(
        const QString& rawFilePath,
        const CameraIntrinsics& intr);

    static CameraDeviceParams getCameraParams(const QString& camKey);
    static CameraIntrinsics getCameraIntrinsics(const QString& camKey, SensorType type, int width, int height);

    template<typename TPoint>
    static typename pcl::PointCloud<TPoint>::Ptr applyTaubinSmoothing(
        const typename pcl::PointCloud<TPoint>::ConstPtr& cloud_in,
        int num_iterations,
        double lambda,
        double mu,
        int k_neighbors = 20);

private:
    // 配准内部辅助函数。
    static PointCloudNormalT::Ptr computeNormals(PointCloudT::Ptr cloud_in, double radius);

    // 测量内部辅助函数。
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

#include "core/point_cloud_algo/PointCloudAlgoSmoothing.tpp"
