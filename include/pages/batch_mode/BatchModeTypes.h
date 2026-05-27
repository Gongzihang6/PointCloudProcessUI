/*
 * 文件说明：定义批处理模式使用的统一参数结构 `BatchParams`。
 */
#pragma once
#include <QString>
struct BatchParams {
    QString inputDir;
    QString outputDir;
    float leafSize = 10.0f;
    double stdDev = 2.0;
    int meanK = 50;
    float clipRadius = 2500.0f;
    int regMethod = 0;
    int icpIter = 60;
    double icpDist = 100.0;
    float ndtRes = 100.0f;
    float ndtStep = 0.1f;
    int ndtIter = 35;
    int gicpIter = 50;
    double gicpDist = 50.0;
    double gicpEps = 1e-8;
    float boxMinX = -1200.0f, boxMinY = -460.0f, boxMinZ = -500.0f;
    float boxMaxX = 500.0f, boxMaxY = 170.0f, boxMaxZ = 2100.0f;
    float boxRotZ = 33.0f;
    int minClusterSize = 5000;
    int extMethodIndex = 0;
    double extEuclideanTol = 40.0;
    int extRgNeighbors = 30;
    double extRgSmoothness = 7.0;
    bool useRansac = false;
    double ransacDistThresh = 20.0;
    bool onlyExtractBody = false;
    bool useMlsUpsampling = true;
    double mlsSearchRadius = 80.0;
    double mlsUpsamplingRadius = 25.0;
    double mlsUpsamplingStep = 25.0;
    float girthThick = 10.0f;
    float skelStep = 20.0f;
    float skelRadius = 30.0f;
    float heightAngle = 15.0f;
    QString aiEndpoint = "http://127.0.0.1:8000/predict";
};
