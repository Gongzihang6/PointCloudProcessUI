/*
 * 文件说明：
 * 该文件实现 `PointCloudAlgo` 的深度图转点云与相机标定参数查询功能。
 *
 * 包含内容：
 * 1. RAW 深度图去畸变并转换为点云；
 * 2. 相机物理参数数据库；
 * 3. 分辨率级别的内参便捷查询。
 */
#include "core/point_cloud_algo/PointCloudAlgoInternal.h"

// 从 RAW 深度图生成点云
PointCloudT::Ptr PointCloudAlgo::convertRawDepthToPointCloud(const QString& rawFilePath, const CameraIntrinsics& intr) {
    PointCloudT::Ptr cloud(new PointCloudT);
    
    QFile file(rawFilePath);
    if (!file.open(QIODevice::ReadOnly)) return nullptr;
    QByteArray data = file.readAll();
    file.close();

    if (data.size() != intr.width * intr.height * 2) return nullptr;

    // 1. 将 RAW 数据加载为 OpenCV 的 16 位单通道图像 (CV_16UC1)
    cv::Mat distortedDepth(intr.height, intr.width, CV_16UC1, (void*)data.constData());
    
    // 2. 构建相机内参矩阵 (Camera Matrix)
    cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << 
        intr.fx, 0, intr.cx,
        0, intr.fy, intr.cy,
        0, 0, 1);
        
    // 3. 构建畸变系数向量 (Distortion Coefficients)
    // 顺序通常为: k1, k2, p1, p2, k3, k4, k5, k6
    cv::Mat distCoeffs = (cv::Mat_<double>(1, 8) << 
        intr.k1, intr.k2, intr.p1, intr.p2, intr.k3, intr.k4, intr.k5, intr.k6);

    // 4. 计算去畸变映射表 (极速查表法)
    cv::Mat map1, map2;
    // 第三个参数是旋转矩阵 R（这里是单位阵），第四个是新的内参矩阵（用原来的即可）
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, 
                                distortedDepth.size(), CV_32FC1, map1, map2);

    // 5. 执行去畸变重映射 (极快，几毫秒内完成)
    cv::Mat undistortedDepth;
    // INTER_NEAREST 防止深度图在边缘处产生插值伪影
    cv::remap(distortedDepth, undistortedDepth, map1, map2, cv::INTER_NEAREST);

    // 6. 使用矫正后的、毫无扭曲的深度图生成高精度点云
    const uint16_t* depthPtr = (const uint16_t*)undistortedDepth.data;
    for (int v = 0; v < intr.height; ++v) {
        for (int u = 0; u < intr.width; ++u) {
            uint16_t z_raw = depthPtr[v * intr.width + u];
            if (z_raw == 0) continue; 

            float z = static_cast<float>(z_raw);
            // 此时的 (u, v) 已经是理想模型下的坐标了，可以直接用简单公式！
            float x = (u - intr.cx) * z / intr.fx;
            float y = (v - intr.cy) * z / intr.fy;

            cloud->push_back(PointT(x, y, z));
        }
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}


// ==========================================
// [新增] 静态相机参数物理数据库 (出厂标定文件大全)
// ==========================================
CameraDeviceParams PointCloudAlgo::getCameraParams(const QString& camKey) {
    static QMap<QString, CameraDeviceParams> db;
    
    // 懒加载：仅在程序首次调用时初始化这庞大的数据库
    if (db.isEmpty()) {
        // 辅助 Lambda 1: 组装外参
        auto makeExt = [](const std::vector<float>& r, const std::vector<float>& t) {
            CameraExtrinsics ext;
            for(int i=0; i<9; ++i) ext.R[i] = r[i];
            for(int i=0; i<3; ++i) ext.T[i] = t[i];
            return ext;
        };

        // 辅助 Lambda 2: 组装内参及畸变 (传入的 dist 顺序严格为 k1, k2, p1, p2, k3, k4, k5, k6)
        auto makeInt = [](int w, int h, float fx, float fy, float cx, float cy, const std::vector<float>& d) {
            CameraIntrinsics i;
            i.width = w; i.height = h; 
            i.fx = fx; i.fy = fy; i.cx = cx; i.cy = cy;
            i.k1 = d[0]; i.k2 = d[1]; i.p1 = d[2]; i.p2 = d[3];
            i.k3 = d[4]; i.k4 = d[5]; i.k5 = d[6]; i.k6 = d[7];
            return i;
        };

        // ----------------- Top (005J) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "Top"; p.serialNumber = "CL8NB43005J";
            p.extrinsics = makeExt({0.994412f, 0.007653f, 0.005581f, -0.008199f, 0.994398f, 0.105381f, -0.004744f, -0.105422f, 0.994416f},
                                   {-32.241280f, -0.717793f, 2.434630f});
            std::vector<float> cd = {0.075733f, -0.106981f, -0.000049f, -0.000155f, 0.044980f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1124.053223f, 1123.337769f, 978.309692f, 528.144592f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  749.368835f,  748.891846f, 652.206482f, 352.096405f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  999.158447f,  998.522461f, 656.275269f, 469.461853f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1498.737671f, 1497.783691f, 1304.412964f, 704.192810f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2248.106445f, 2246.675537f, 1956.619385f, 1056.289185f, cd);

            std::vector<float> dd = {21.223101f, 9.922773f, 0.000056f, -0.000013f, 0.320325f, 21.530495f, 17.081924f, 2.168158f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.488007f, 504.548981f, 323.691956f, 336.099609f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.488007f, 504.548981f, 515.691956f, 516.099609f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.244003f, 252.274490f, 161.845978f, 168.049805f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.244003f, 252.274490f, 257.845978f, 258.049805f, dd);
            db["Top"] = p;
        }

        // ----------------- LB (00SE) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "LB"; p.serialNumber = "CL8NB4300SE";
            p.extrinsics = makeExt({0.994037f, 0.008991f, 0.001626f, -0.009114f, 0.993997f, 0.109027f, -0.000636f, -0.109037f, 0.994038f},
                                   {-32.539291f, -0.761012f, 2.362806f});
            std::vector<float> cd = {0.070404f, -0.097203f, -0.000234f, -0.000113f, 0.038830f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1125.550903f, 1124.178711f, 942.350952f, 538.263062f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  750.367249f,  749.452454f, 628.233948f, 358.842041f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960, 1000.489685f,  999.269958f, 624.311951f, 478.456055f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1500.734497f, 1498.904907f, 1256.467896f, 717.684082f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2251.101807f, 2248.357422f, 1884.701904f, 1076.526123f, cd);

            std::vector<float> dd = {23.504353f, 12.172994f, 0.000100f, -0.000031f, 0.456110f, 23.796473f, 20.142593f, 2.833089f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.507996f, 504.420441f, 331.245239f, 333.824890f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.507996f, 504.420441f, 523.245239f, 513.824890f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.253998f, 252.210220f, 165.622620f, 166.912445f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.253998f, 252.210220f, 261.622620f, 256.912445f, dd);
            db["LB"] = p;
        }

        // ----------------- LT (003W) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "LT"; p.serialNumber = "CL8NB43003W";
            p.extrinsics = makeExt({0.994410f, -0.006397f, 0.002888f, 0.006056f, 0.994393f, 0.105573f, -0.003547f, -0.105553f, 0.994407f},
                                   {-32.140503f, -0.938418f, 2.323613f});
            std::vector<float> cd = {0.070888f, -0.097229f, 0.000126f, -0.000605f, 0.038468f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1122.910767f, 1122.455688f, 954.760742f, 555.962830f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  748.607178f,  748.303772f, 636.507141f, 370.641876f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  998.142883f,  997.738403f, 635.342896f, 494.189178f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1497.214355f, 1496.607544f, 1273.014282f, 741.283752f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2245.821533f, 2244.911377f, 1909.521484f, 1111.925659f, cd);

            std::vector<float> dd = {13.572120f, 6.620557f, 0.000087f, -0.000010f, 0.229236f, 13.892242f, 11.192241f, 1.494901f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.837738f, 504.902527f, 345.253601f, 324.862488f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.837738f, 504.902527f, 537.253601f, 504.862488f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.418869f, 252.451263f, 172.626801f, 162.431244f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.418869f, 252.451263f, 268.626801f, 252.431244f, dd);
            db["LT"] = p;
        }

        // ----------------- RB (00YA) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "RB"; p.serialNumber = "CL8NB4300YA";
            p.extrinsics = makeExt({0.994235f, 0.007331f, 0.005806f, -0.007910f, 0.994224f, 0.107030f, -0.004988f, -0.107071f, 0.994239f},
                                   {-32.348194f, -0.746979f, 2.932103f});
            std::vector<float> cd = {0.073843f, -0.101794f, -0.000170f, 0.000267f, 0.041181f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1120.983398f, 1120.277466f, 957.993774f, 534.084900f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  747.322266f,  746.851624f, 638.662537f, 356.056610f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  996.429688f,  995.802185f, 638.216675f, 474.742126f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1494.644531f, 1493.703247f, 1277.325073f, 712.113220f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2241.966797f, 2240.554932f, 1915.987549f, 1068.169800f, cd);

            std::vector<float> dd = {22.002260f, 10.350877f, 0.000022f, 0.000035f, 0.329348f, 22.304708f, 17.792418f, 2.251812f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 505.278992f, 505.316833f, 327.487793f, 327.828064f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 505.278992f, 505.316833f, 519.487793f, 507.828064f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.639496f, 252.658417f, 163.743896f, 163.914032f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.639496f, 252.658417f, 259.743896f, 253.914032f, dd);
            db["RB"] = p;
        }

        // ----------------- RT (00X6) -----------------
        {
            CameraDeviceParams p;
            p.roleName = "RT"; p.serialNumber = "CL8NB4300X6";
            p.extrinsics = makeExt({0.994587f, -0.007555f, 0.001551f, 0.007353f, 0.994560f, 0.103910f, -0.002327f, -0.103895f, 0.994586f},
                                   {-32.338169f, -1.139261f, 2.579680f});
            std::vector<float> cd = {0.071690f, -0.099641f, 0.000179f, -0.000450f, 0.040123f, 0.f, 0.f, 0.f};
            p.colorIntrinsics["1920x1080"] = makeInt(1920, 1080, 1121.462646f, 1120.027466f, 955.431702f, 530.105225f, cd);
            p.colorIntrinsics["1280x720"]  = makeInt(1280,  720,  747.641785f,  746.684998f, 636.954468f, 353.403473f, cd);
            p.colorIntrinsics["1280x960"]  = makeInt(1280,  960,  996.855713f,  995.579956f, 635.939270f, 471.204651f, cd);
            p.colorIntrinsics["2560x1440"] = makeInt(2560, 1440, 1495.283569f, 1493.369995f, 1273.908936f, 706.806946f, cd);
            p.colorIntrinsics["3840x2160"] = makeInt(3840, 2160, 2242.925293f, 2240.054932f, 1910.863403f, 1060.210449f, cd);

            std::vector<float> dd = {25.453939f, 13.172181f, 0.000080f, 0.000028f, 0.483954f, 25.737801f, 21.811607f, 3.050446f};
            p.depthIntrinsics["640x576"]   = makeInt( 640,  576, 504.503265f, 504.484131f, 339.310181f, 336.973572f, dd);
            p.depthIntrinsics["1024x1024"] = makeInt(1024, 1024, 504.503265f, 504.484131f, 531.310181f, 516.973572f, dd);
            p.depthIntrinsics["320x288"]   = makeInt( 320,  288, 252.251633f, 252.242065f, 169.655090f, 168.486786f, dd);
            p.depthIntrinsics["512x512"]   = makeInt( 512,  512, 252.251633f, 252.242065f, 265.655090f, 258.486786f, dd);
            db["RT"] = p;
        }
    }
    return db.value(camKey);
}

// ==========================================
// [新增] 便捷函数：查表直接提取出特定分辨率的内参
// ==========================================
CameraIntrinsics PointCloudAlgo::getCameraIntrinsics(const QString& camKey, SensorType type, int width, int height) {
    CameraDeviceParams params = getCameraParams(camKey);
    QString resKey = QString("%1x%2").arg(width).arg(height); // e.g. "1024x1024"
    
    if (type == SensorType::COLOR && params.colorIntrinsics.contains(resKey)) {
        return params.colorIntrinsics[resKey];
    } else if (type == SensorType::DEPTH && params.depthIntrinsics.contains(resKey)) {
        return params.depthIntrinsics[resKey];
    }
    
    // 如果没找到，返回一个容错的虚拟内参，防止程序崩溃
    return {width, height, 1000.f, 1000.f, width/2.f, height/2.f, 0,0,0,0,0,0,0,0};
}
