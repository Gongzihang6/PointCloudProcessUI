/*
 * 文件说明：
 * 该文件定义 `PointCloudAlgo` 相关的公共类型、点云别名、相机参数结构与测量结果结构。
 *
 * 设计目的：
 * 1. 将算法类型与算法接口分离，降低主头文件耦合度；
 * 2. 供页面层、批处理层和算法实现层共享同一套数据模型；
 * 3. 保持原有类型名不变，避免影响现有业务代码调用方式。
 */
#pragma once

#include <map>
#include <vector>

#include <QString>
#include <QMap>

#include <Eigen/Dense>

#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZ;
using PointCloudT = pcl::PointCloud<PointT>;
using PointNormalT = pcl::PointNormal;
using PointCloudNormalT = pcl::PointCloud<PointNormalT>;

enum class SensorType {
    COLOR,
    DEPTH
};

struct CameraExtrinsics {
    float R[9];
    float T[3];
};

struct CameraIntrinsics {
    int width, height;
    float fx, fy, cx, cy;
    float k1, k2, k3, k4, k5, k6;
    float p1, p2;
};

struct CameraDeviceParams {
    QString roleName;
    QString serialNumber;
    CameraExtrinsics extrinsics;
    QMap<QString, CameraIntrinsics> colorIntrinsics;
    QMap<QString, CameraIntrinsics> depthIntrinsics;
};

struct BodySizeResults {
    double body_length = 0.0;
    double body_height = 0.0;
    double body_width = 0.0;
    double chest_girth = 0.0;
    double waist_girth = 0.0;
    double hip_girth = 0.0;

    PointCloudT::Ptr aligned_cloud;
    std::vector<PointT> aligned_keypoints;
    PointCloudT::Ptr skeleton_cloud;

    PointT height_top, height_bottom;
    PointT width_p1, width_p2;
    PointCloudT::Ptr chest_contour;
    PointCloudT::Ptr waist_contour;
    PointCloudT::Ptr hip_contour;
    PointCloudT::Ptr ground_polygon;
};

struct PolarPoint {
    float r;
    float theta;

    bool operator < (const PolarPoint& other) const {
        return theta < other.theta;
    }
};

struct ExtractionParams {
    float boxMinX = -1200.0f, boxMinY = -460.0f, boxMinZ = -500.0f;
    float boxMaxX = 600.0f,  boxMaxY = 170.0f,  boxMaxZ = 2100.0f;
    int minClusterSize = 5000;

    int methodIndex = 0;

    double euclideanTolerance = 40.0;

    int rgNeighbors = 30;
    double rgSmoothness = 7.0;
    double rgCurvature = 1.0;

    float boxRotZ = 33.0f;

    bool useRansac = true;
    double ransacDistThresh = 30.0;
    double ransacAngleThresh = 10.0;

    bool useMlsUpsampling = true;
    double mlsSearchRadius = 80.0;
    double mlsUpsamplingRadius = 25.0;
    double mlsUpsamplingStep = 25.0;
};
