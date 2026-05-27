/*
 * 文件说明：
 * 该文件提供 `PointCloudAlgo::applyTaubinSmoothing` 的模板实现。
 *
 * 说明：
 * 模板函数必须在头文件可见范围内实现，因此单独拆分到 `.tpp` 文件并由主头文件包含。
 */
#pragma once

#include <iostream>

#include <pcl/common/io.h>
#include <pcl/search/kdtree.h>

template<typename TPoint>
typename pcl::PointCloud<TPoint>::Ptr PointCloudAlgo::applyTaubinSmoothing(
    const typename pcl::PointCloud<TPoint>::ConstPtr& cloud_in,
    int num_iterations,
    double lambda,
    double mu,
    int k_neighbors)
{
    std::cout << "  -> 应用陶宾平滑..." << std::endl;
    typename pcl::PointCloud<TPoint>::Ptr cloud_smoothed(new pcl::PointCloud<TPoint>(*cloud_in));
    typename pcl::PointCloud<TPoint>::Ptr cloud_temp(new pcl::PointCloud<TPoint>(*cloud_in));
    typename pcl::search::KdTree<TPoint>::Ptr tree(new pcl::search::KdTree<TPoint>());
    for (int iter = 0; iter < num_iterations; ++iter) {
        tree->setInputCloud(cloud_smoothed);
        pcl::copyPointCloud(*cloud_smoothed, *cloud_temp);
        for (size_t i = 0; i < cloud_temp->size(); ++i) {
            std::vector<int> indices;
            std::vector<float> sqr_distances;
            tree->nearestKSearch(cloud_temp->points[i], k_neighbors, indices, sqr_distances);
            Eigen::Vector3f centroid(0, 0, 0);
            for (const int& idx : indices) {
                centroid += cloud_temp->points[idx].getVector3fMap();
            }
            centroid /= indices.size();
            Eigen::Vector3f displacement = centroid - cloud_temp->points[i].getVector3fMap();
            cloud_smoothed->points[i].getVector3fMap() += lambda * displacement;
        }

        pcl::copyPointCloud(*cloud_smoothed, *cloud_temp);
        for (size_t i = 0; i < cloud_temp->size(); ++i) {
            std::vector<int> indices;
            std::vector<float> sqr_distances;
            tree->nearestKSearch(cloud_temp->points[i], k_neighbors, indices, sqr_distances);
            Eigen::Vector3f centroid(0, 0, 0);
            for (const int& idx : indices) {
                centroid += cloud_temp->points[idx].getVector3fMap();
            }
            centroid /= indices.size();
            Eigen::Vector3f displacement = centroid - cloud_temp->points[i].getVector3fMap();
            cloud_smoothed->points[i].getVector3fMap() += mu * displacement;
        }
    }
    return cloud_smoothed;
}
