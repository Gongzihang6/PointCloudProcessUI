#pragma once

#include <QString>
#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include <string>
#include <vector>

class OpenPoseInferencer {
public:
    OpenPoseInferencer();
    ~OpenPoseInferencer();

    bool initModel(const QString& modelPath, QString& outErrorMsg, bool preferGpu = true);
    std::vector<cv::Point3f> predict(const cv::Mat& inputImage);

    void setThresholds(float confThreshold, float iouThreshold, float kptThreshold = 0.3f);
    void setNumClasses(int numClasses);

    int inputWidth() const { return m_inputWidth; }
    int inputHeight() const { return m_inputHeight; }
    int numKeypoints() const { return m_numKeypoints; }

private:
    struct LetterBoxInfo {
        float scale = 1.0f;
        int left = 0;
        int top = 0;
    };

    struct Candidate {
        float confidence = 0.0f;
        cv::Rect2f box;
        std::vector<cv::Point3f> keypoints;
    };

    cv::Mat preprocess(const cv::Mat& inputImage, LetterBoxInfo& info) const;
    bool decodePoseOutput(const Ort::Value& outputTensor,
                          const LetterBoxInfo& letterboxInfo,
                          const cv::Size& originalSize,
                          std::vector<cv::Point3f>& keypoints) const;

    static float iou(const cv::Rect2f& a, const cv::Rect2f& b);
    bool inferHeadLayout(int channels, int& classCount, int& keypointCount) const;

private:
    Ort::Env* m_env = nullptr;
    Ort::Session* m_session = nullptr;

    std::string m_inputName;
    std::string m_outputName;

    int m_inputWidth = 640;
    int m_inputHeight = 640;
    int m_numClasses = 1;
    int m_numKeypoints = 6;

    float m_confThreshold = 0.35f;
    float m_iouThreshold = 0.45f;
    float m_kptThreshold = 0.3f;
};
