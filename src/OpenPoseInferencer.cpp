#include "OpenPoseInferencer.h"

#include <QDebug>

#include <algorithm>
#include <cmath>
#include <limits>

OpenPoseInferencer::OpenPoseInferencer() = default;

OpenPoseInferencer::~OpenPoseInferencer() {
    if (m_session) {
        delete m_session;
        m_session = nullptr;
    }
    if (m_env) {
        delete m_env;
        m_env = nullptr;
    }
}

void OpenPoseInferencer::setThresholds(float confThreshold, float iouThreshold, float kptThreshold) {
    m_confThreshold = confThreshold;
    m_iouThreshold = iouThreshold;
    m_kptThreshold = kptThreshold;
}

void OpenPoseInferencer::setNumClasses(int numClasses) {
    m_numClasses = std::max(1, numClasses);
}

bool OpenPoseInferencer::initModel(const QString& modelPath, QString& outErrorMsg, bool preferGpu) {
    const OrtApi& api = Ort::GetApi();

    if (m_session) {
        delete m_session;
        m_session = nullptr;
    }
    if (m_env) {
        delete m_env;
        m_env = nullptr;
    }

    std::wstring wModelPath = modelPath.toStdWString();

    OrtEnv* cEnv = nullptr;
    OrtStatus* envStatus = api.CreateEnv(ORT_LOGGING_LEVEL_WARNING, "OpenPoseInferencer", &cEnv);
    if (envStatus != nullptr) {
        outErrorMsg = QString::fromStdString(api.GetErrorMessage(envStatus));
        api.ReleaseStatus(envStatus);
        return false;
    }
    m_env = new Ort::Env(cEnv);

    Ort::SessionOptions sessionOptions;
    sessionOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
    sessionOptions.SetIntraOpNumThreads(4);

    bool useGpu = preferGpu;
    if (useGpu) {
        OrtCUDAProviderOptions cudaOptions{};
        OrtStatus* cudaStatus = api.SessionOptionsAppendExecutionProvider_CUDA(sessionOptions, &cudaOptions);
        if (cudaStatus != nullptr) {
            qDebug() << "CUDA 挂载失败，自动降级 CPU:" << api.GetErrorMessage(cudaStatus);
            api.ReleaseStatus(cudaStatus);
            useGpu = false;
        }
    }

    OrtSession* cSession = nullptr;
    OrtStatus* sessionStatus = api.CreateSession(*m_env, wModelPath.c_str(), sessionOptions, &cSession);
    if (sessionStatus != nullptr && useGpu) {
        api.ReleaseStatus(sessionStatus);
        qDebug() << "GPU Session 创建失败，改用 CPU 重试";

        useGpu = false;
        Ort::SessionOptions cpuOptions;
        cpuOptions.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        cpuOptions.SetIntraOpNumThreads(4);
        sessionStatus = api.CreateSession(*m_env, wModelPath.c_str(), cpuOptions, &cSession);
    }

    if (sessionStatus != nullptr) {
        outErrorMsg = QString::fromStdString(api.GetErrorMessage(sessionStatus));
        api.ReleaseStatus(sessionStatus);
        return false;
    }

    m_session = new Ort::Session(cSession);
    outErrorMsg = useGpu ? "成功 (CUDA)" : "成功 (CPU)";

    try {
        Ort::AllocatorWithDefaultOptions allocator;

        auto inputName = m_session->GetInputNameAllocated(0, allocator);
        auto outputName = m_session->GetOutputNameAllocated(0, allocator);
        m_inputName = inputName.get();
        m_outputName = outputName.get();

        Ort::TypeInfo inputTypeInfo = m_session->GetInputTypeInfo(0);
        auto inputTensorInfo = inputTypeInfo.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> inputDims = inputTensorInfo.GetShape();

        if (inputDims.size() == 4) {
            if (inputDims[2] > 0) {
                m_inputHeight = static_cast<int>(inputDims[2]);
            }
            if (inputDims[3] > 0) {
                m_inputWidth = static_cast<int>(inputDims[3]);
            }
        }

        Ort::TypeInfo outTypeInfo = m_session->GetOutputTypeInfo(0);
        auto outTensorInfo = outTypeInfo.GetTensorTypeAndShapeInfo();
        std::vector<int64_t> outDims = outTensorInfo.GetShape();

        if (outDims.size() == 3) {
            int channels = static_cast<int>(std::min(outDims[1], outDims[2]));
            int inferredClassCount = 1;
            int inferredKeypointCount = 0;
            if (inferHeadLayout(channels, inferredClassCount, inferredKeypointCount)) {
                m_numClasses = inferredClassCount;
                m_numKeypoints = inferredKeypointCount;
            }
        }

        qDebug() << "输入尺寸:" << m_inputWidth << "x" << m_inputHeight
                 << "类别数:" << m_numClasses << "关键点数:" << m_numKeypoints;
    } catch (const std::exception& e) {
        qDebug() << "读取模型输入输出信息失败:" << e.what();
    }

    return true;
}

cv::Mat OpenPoseInferencer::preprocess(const cv::Mat& inputImage, LetterBoxInfo& info) const {
    const float scale = std::min(static_cast<float>(m_inputWidth) / static_cast<float>(inputImage.cols),
                                 static_cast<float>(m_inputHeight) / static_cast<float>(inputImage.rows));

    const int resizedW = static_cast<int>(std::round(inputImage.cols * scale));
    const int resizedH = static_cast<int>(std::round(inputImage.rows * scale));

    info.scale = scale;
    info.left = (m_inputWidth - resizedW) / 2;
    info.top = (m_inputHeight - resizedH) / 2;

    cv::Mat resized;
    cv::resize(inputImage, resized, cv::Size(resizedW, resizedH));

    cv::Mat padded(m_inputHeight, m_inputWidth, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(padded(cv::Rect(info.left, info.top, resizedW, resizedH)));

    cv::Mat rgb;
    cv::cvtColor(padded, rgb, cv::COLOR_BGR2RGB);

    cv::Mat floatImg;
    rgb.convertTo(floatImg, CV_32FC3, 1.0 / 255.0);

    cv::Mat blob;
    cv::dnn::blobFromImage(floatImg, blob, 1.0, cv::Size(m_inputWidth, m_inputHeight), cv::Scalar(), false, false);
    return blob;
}

float OpenPoseInferencer::iou(const cv::Rect2f& a, const cv::Rect2f& b) {
    const float x1 = std::max(a.x, b.x);
    const float y1 = std::max(a.y, b.y);
    const float x2 = std::min(a.x + a.width, b.x + b.width);
    const float y2 = std::min(a.y + a.height, b.y + b.height);

    const float w = std::max(0.0f, x2 - x1);
    const float h = std::max(0.0f, y2 - y1);
    const float inter = w * h;
    const float unionArea = a.area() + b.area() - inter;
    if (unionArea <= 0.0f) {
        return 0.0f;
    }
    return inter / unionArea;
}

bool OpenPoseInferencer::inferHeadLayout(int channels, int& classCount, int& keypointCount) const {
    const int fixedPart = 4;
    int desiredClass = std::max(1, m_numClasses);

    if (channels > fixedPart + desiredClass && ((channels - fixedPart - desiredClass) % 3 == 0)) {
        classCount = desiredClass;
        keypointCount = (channels - fixedPart - desiredClass) / 3;
        return keypointCount > 0;
    }

    for (int cls = 1; cls <= 8; ++cls) {
        if (channels > fixedPart + cls && ((channels - fixedPart - cls) % 3 == 0)) {
            classCount = cls;
            keypointCount = (channels - fixedPart - cls) / 3;
            return keypointCount > 0;
        }
    }

    return false;
}

bool OpenPoseInferencer::decodePoseOutput(const Ort::Value& outputTensor,
                                          const LetterBoxInfo& letterboxInfo,
                                          const cv::Size& originalSize,
                                          std::vector<cv::Point3f>& keypoints) const {
    auto tensorInfo = outputTensor.GetTensorTypeAndShapeInfo();
    std::vector<int64_t> outDims = tensorInfo.GetShape();
    if (outDims.size() != 3) {
        qDebug() << "不支持的输出维度数量:" << outDims.size();
        return false;
    }

    const int64_t d1 = outDims[1];
    const int64_t d2 = outDims[2];

    const bool channelsFirst = d1 < d2;
    const int channels = static_cast<int>(channelsFirst ? d1 : d2);
    const int numPreds = static_cast<int>(channelsFirst ? d2 : d1);

    int classCount = m_numClasses;
    int keypointCount = m_numKeypoints;
    if (!inferHeadLayout(channels, classCount, keypointCount)) {
        qDebug() << "无法从输出通道推断姿态头布局, channels=" << channels;
        return false;
    }

    const float* out = outputTensor.GetTensorData<float>();

    auto valueAt = [&](int predIdx, int channelIdx) -> float {
        if (channelsFirst) {
            return out[static_cast<size_t>(channelIdx) * static_cast<size_t>(numPreds) + static_cast<size_t>(predIdx)];
        }
        return out[static_cast<size_t>(predIdx) * static_cast<size_t>(channels) + static_cast<size_t>(channelIdx)];
    };

    std::vector<Candidate> candidates;
    candidates.reserve(numPreds);

    const int kptBase = 4 + classCount;

    for (int i = 0; i < numPreds; ++i) {
        float bestClsScore = -std::numeric_limits<float>::infinity();
        for (int c = 0; c < classCount; ++c) {
            bestClsScore = std::max(bestClsScore, valueAt(i, 4 + c));
        }

        if (bestClsScore < m_confThreshold) {
            continue;
        }

        const float cx = valueAt(i, 0);
        const float cy = valueAt(i, 1);
        const float w = valueAt(i, 2);
        const float h = valueAt(i, 3);

        Candidate cand;
        cand.confidence = bestClsScore;
        cand.box = cv::Rect2f(cx - 0.5f * w, cy - 0.5f * h, w, h);
        cand.keypoints.resize(static_cast<size_t>(keypointCount), cv::Point3f(-1.0f, -1.0f, 0.0f));

        for (int k = 0; k < keypointCount; ++k) {
            const int base = kptBase + k * 3;
            const float x = valueAt(i, base);
            const float y = valueAt(i, base + 1);
            const float s = valueAt(i, base + 2);
            if (s >= m_kptThreshold) {
                cand.keypoints[static_cast<size_t>(k)] = cv::Point3f(x, y, s);
            }
        }

        candidates.push_back(std::move(cand));
    }

    if (candidates.empty()) {
        keypoints.assign(static_cast<size_t>(keypointCount), cv::Point3f(-1.0f, -1.0f, 0.0f));
        return true;
    }

    std::vector<cv::Rect> nmsBoxes;
    std::vector<float> nmsScores;
    nmsBoxes.reserve(candidates.size());
    nmsScores.reserve(candidates.size());

    for (const auto& c : candidates) {
        nmsBoxes.emplace_back(static_cast<int>(std::round(c.box.x)),
                              static_cast<int>(std::round(c.box.y)),
                              static_cast<int>(std::round(c.box.width)),
                              static_cast<int>(std::round(c.box.height)));
        nmsScores.push_back(c.confidence);
    }

    std::vector<int> keep;
    cv::dnn::NMSBoxes(nmsBoxes, nmsScores, m_confThreshold, m_iouThreshold, keep);

    if (keep.empty()) {
        keypoints.assign(static_cast<size_t>(keypointCount), cv::Point3f(-1.0f, -1.0f, 0.0f));
        return true;
    }

    int bestIdx = keep[0];
    float bestScore = candidates[static_cast<size_t>(bestIdx)].confidence;
    for (int idx : keep) {
        if (candidates[static_cast<size_t>(idx)].confidence > bestScore) {
            bestScore = candidates[static_cast<size_t>(idx)].confidence;
            bestIdx = idx;
        }
    }

    const auto& best = candidates[static_cast<size_t>(bestIdx)];
    keypoints = best.keypoints;

    for (auto& kp : keypoints) {
        if (kp.z < m_kptThreshold || kp.x < 0.0f || kp.y < 0.0f) {
            kp = cv::Point3f(-1.0f, -1.0f, 0.0f);
            continue;
        }

        float x = (kp.x - static_cast<float>(letterboxInfo.left)) / letterboxInfo.scale;
        float y = (kp.y - static_cast<float>(letterboxInfo.top)) / letterboxInfo.scale;

        x = std::clamp(x, 0.0f, static_cast<float>(originalSize.width - 1));
        y = std::clamp(y, 0.0f, static_cast<float>(originalSize.height - 1));

        kp.x = x;
        kp.y = y;
    }

    return true;
}

std::vector<cv::Point3f> OpenPoseInferencer::predict(const cv::Mat& inputImage) {
    std::vector<cv::Point3f> keypoints;

    if (!m_session || inputImage.empty()) {
        return keypoints;
    }

    try {
        LetterBoxInfo letterboxInfo;
        cv::Mat blob = preprocess(inputImage, letterboxInfo);

        std::vector<int64_t> inputDims = {1, 3, m_inputHeight, m_inputWidth};
        const size_t inputTensorSize = static_cast<size_t>(blob.total());

        Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
        Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
            memoryInfo,
            reinterpret_cast<float*>(blob.data),
            inputTensorSize,
            inputDims.data(),
            inputDims.size());

        const char* inputNames[] = {m_inputName.c_str()};
        const char* outputNames[] = {m_outputName.c_str()};

        auto outputTensors = m_session->Run(Ort::RunOptions{nullptr}, inputNames, &inputTensor, 1, outputNames, 1);
        if (outputTensors.empty()) {
            return keypoints;
        }

        if (!decodePoseOutput(outputTensors[0], letterboxInfo, inputImage.size(), keypoints)) {
            qDebug() << "后处理解码失败";
            keypoints.clear();
        }
    } catch (const Ort::Exception& e) {
        qDebug() << "ONNX Runtime 异常:" << e.what();
        keypoints.clear();
    } catch (const std::exception& e) {
        qDebug() << "标准异常:" << e.what();
        keypoints.clear();
    }

    return keypoints;
}
