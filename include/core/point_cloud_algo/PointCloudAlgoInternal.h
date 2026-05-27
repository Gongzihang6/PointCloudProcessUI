/*
 * 文件说明：
 * 该文件是 `PointCloudAlgo` 拆分实现后的内部公共头，仅供算法实现文件使用。
 *
 * 主要职责：
 * 1. 汇总各算法实现所需的 PCL / Eigen / Qt / OpenCV 头文件；
 * 2. 让多个 `.cpp` 文件共享同一套重型依赖，避免在对外头文件中暴露过多实现细节；
 * 3. 作为 `PointCloudAlgo` 模块拆分后的统一实现入口。
 */
#pragma once

#include "core/PointCloudAlgo.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>

#include <QByteArray>
#include <QFile>

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/mls.h>
