/*
 * 文件说明：实现 `SingleModePage` 的主体提取模块，包括 WSL 分割与传统规则提取。
 */
#include "pages/single_mode/SingleModePageInternal.h"

namespace {

// #region debug-point A:qt-wsl-seg-debug-helper
void postQtSegDebugEvent(const QString& hypothesisId,
                         const QString& location,
                         const QString& message,
                         const QJsonObject& data = QJsonObject(),
                         const QString& runId = "pre-fix") {
    QString debugServerUrl = "http://127.0.0.1:7777/event";
    QString sessionId = "qt-wsl-seg-crash";

    QFile envFile(".dbg/qt-wsl-seg-crash.env");
    if (envFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QTextStream stream(&envFile);
        while (!stream.atEnd()) {
            const QString line = stream.readLine().trimmed();
            if (line.startsWith("DEBUG_SERVER_URL=")) {
                debugServerUrl = line.section('=', 1);
            } else if (line.startsWith("DEBUG_SESSION_ID=")) {
                sessionId = line.section('=', 1);
            }
        }
        envFile.close();
    }

    QJsonObject payload;
    payload["sessionId"] = sessionId;
    payload["runId"] = runId;
    payload["hypothesisId"] = hypothesisId;
    payload["location"] = location;
    payload["msg"] = "[DEBUG] " + message;
    payload["data"] = data;
    payload["ts"] = QString::number(QDateTime::currentMSecsSinceEpoch());

    auto *debugManager = new QNetworkAccessManager();
    QNetworkRequest request{QUrl(debugServerUrl)};
    request.setHeader(QNetworkRequest::ContentTypeHeader, "application/json");
    QNetworkReply *reply = debugManager->post(request, QJsonDocument(payload).toJson(QJsonDocument::Compact));
    QObject::connect(reply, &QNetworkReply::finished, debugManager, [reply, debugManager]() {
        reply->deleteLater();
        debugManager->deleteLater();
    });
}
// #endregion

/*
 * 作用：
 * 将 Qt 内存中的点云直接打包为 WSL 服务约定的裸 float32 xyz 二进制。
 * 这样前端无需落盘为 PCD，和 `PigSegPrediction_Qt.py` / `OffsetKeyPointPrediction_Qt.py`
 * 的协议保持一致。
 */
QByteArray serializePointCloudToFloat32Bytes(const PointCloudT::Ptr& cloud, QString& errorMessage) {
    if (!cloud || cloud->empty()) {
        errorMessage = "待上传的融合点云为空。";
        return QByteArray();
    }

    QByteArray bytes;
    bytes.resize(static_cast<int>(cloud->size() * 3 * sizeof(float)));
    float *ptr = reinterpret_cast<float*>(bytes.data());
    for (const auto& point : cloud->points) {
        *ptr++ = point.x;
        *ptr++ = point.y;
        *ptr++ = point.z;
    }
    return bytes;
}

/*
 * 作用：
 * 将 JSON 数组解析为点云，兼容三种常见返回格式：
 * 1. 扁平数组：[x1, y1, z1, x2, y2, z2, ...]
 * 2. 二维数组：[[x, y, z], [x, y, z], ...]
 * 3. 对象数组：[{"x":..., "y":..., "z":...}, ...]
 */
PointCloudT::Ptr jsonArrayToPointCloud(const QJsonArray& pointsArray, QString& errorMessage) {
    PointCloudT::Ptr cloud(new PointCloudT);
    if (pointsArray.isEmpty()) {
        errorMessage = "后端返回的点云数组为空。";
        return nullptr;
    }

    // 情况 1：扁平浮点数组
    if (pointsArray.at(0).isDouble()) {
        if (pointsArray.size() % 3 != 0) {
            errorMessage = "扁平点云数组长度不是 3 的整数倍。";
            return nullptr;
        }

        cloud->reserve(pointsArray.size() / 3);
        for (int i = 0; i < pointsArray.size(); i += 3) {
            PointT pt;
            pt.x = static_cast<float>(pointsArray.at(i).toDouble());
            pt.y = static_cast<float>(pointsArray.at(i + 1).toDouble());
            pt.z = static_cast<float>(pointsArray.at(i + 2).toDouble());
            cloud->points.push_back(pt);
        }
    } else {
        // 情况 2/3：二维数组或对象数组
        cloud->reserve(pointsArray.size());
        for (const QJsonValue& pointValue : pointsArray) {
            PointT pt;
            bool ok = false;

            if (pointValue.isArray()) {
                QJsonArray triple = pointValue.toArray();
                if (triple.size() >= 3) {
                    pt.x = static_cast<float>(triple.at(0).toDouble());
                    pt.y = static_cast<float>(triple.at(1).toDouble());
                    pt.z = static_cast<float>(triple.at(2).toDouble());
                    ok = true;
                }
            } else if (pointValue.isObject()) {
                QJsonObject obj = pointValue.toObject();
                if (obj.contains("x") && obj.contains("y") && obj.contains("z")) {
                    pt.x = static_cast<float>(obj.value("x").toDouble());
                    pt.y = static_cast<float>(obj.value("y").toDouble());
                    pt.z = static_cast<float>(obj.value("z").toDouble());
                    ok = true;
                }
            }

            if (!ok) {
                errorMessage = "点云 JSON 中存在无法解析的点结构。";
                return nullptr;
            }

            cloud->points.push_back(pt);
        }
    }

    cloud->width = static_cast<std::uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

/*
 * 作用：
 * 将后端直接返回的 PCD 二进制内容写入临时文件，再复用 PCL 的标准加载流程读取。
 * 这样前端不需要关心 PCD 头部或编码细节，最大程度兼容后端导出的点云格式。
 */
PointCloudT::Ptr loadPointCloudFromPcdBytes(const QByteArray& pcdBytes, QString& errorMessage) {
    if (pcdBytes.isEmpty()) {
        errorMessage = "后端返回的点云文件为空。";
        return nullptr;
    }

    QTemporaryFile tempFile(QDir::tempPath() + "/segmented_body_XXXXXX.pcd");
    tempFile.setAutoRemove(false);
    if (!tempFile.open()) {
        errorMessage = "无法创建临时文件用于读取语义分割结果。";
        return nullptr;
    }

    const QString tempPath = tempFile.fileName();
    if (tempFile.write(pcdBytes) != pcdBytes.size()) {
        tempFile.close();
        QFile::remove(tempPath);
        errorMessage = "无法写入语义分割结果到临时文件。";
        return nullptr;
    }
    tempFile.close();

    PointCloudT::Ptr cloud(new PointCloudT);
    int loadCode = -1;
    try {
        // #region debug-point D:pcd-load-attempt
        postQtSegDebugEvent("D",
                            "SingleModePage.cpp:loadPointCloudFromPcdBytes",
                            "尝试将响应按 PCD 文件解析",
                            QJsonObject{
                                {"tempPath", tempPath},
                                {"byteCount", static_cast<qint64>(pcdBytes.size())}
                            });
        // #endregion
        loadCode = pcl::io::loadPCDFile<PointT>(tempPath.toStdString(), *cloud);
    } catch (const std::exception& ex) {
        // #region debug-point D:pcd-load-exception
        postQtSegDebugEvent("D",
                            "SingleModePage.cpp:loadPointCloudFromPcdBytes",
                            "按 PCD 解析响应时抛出异常",
                            QJsonObject{
                                {"exception", QString::fromLocal8Bit(ex.what())},
                                {"byteCount", static_cast<qint64>(pcdBytes.size())}
                            });
        // #endregion
        QFile::remove(tempPath);
        throw;
    }
    QFile::remove(tempPath);

    if (loadCode != 0 || !cloud || cloud->empty()) {
        errorMessage = "后端返回的 PCD 无法解析或点数为空。";
        return nullptr;
    }

    return cloud;
}

/*
 * 作用：
 * 尝试从后端响应中解析主体点云。
 * 优先支持 JSON，其次兼容直接返回的 PCD 二进制。
 */
PointCloudT::Ptr parseSegmentationReply(const QByteArray& responseData,
                                        const QString& contentType,
                                        QString& errorMessage) {
    if (responseData.isEmpty()) {
        errorMessage = "后端返回内容为空。";
        return nullptr;
    }

    const bool looksLikeJson =
        contentType.contains("json", Qt::CaseInsensitive) ||
        responseData.trimmed().startsWith('{') ||
        responseData.trimmed().startsWith('[');

    // #region debug-point B:parse-branch
    postQtSegDebugEvent("B",
                        "SingleModePage.cpp:parseSegmentationReply",
                        "开始解析语义分割响应",
                        QJsonObject{
                            {"contentType", contentType},
                            {"byteCount", static_cast<qint64>(responseData.size())},
                            {"looksLikeJson", looksLikeJson}
                        });
    // #endregion

    if (looksLikeJson) {
        QJsonParseError parseError;
        QJsonDocument doc = QJsonDocument::fromJson(responseData, &parseError);
        if (parseError.error == QJsonParseError::NoError) {
            if (doc.isArray()) {
                return jsonArrayToPointCloud(doc.array(), errorMessage);
            }

            if (doc.isObject()) {
                QJsonObject root = doc.object();
                if (root.contains("success") && !root.value("success").toBool(true)) {
                    errorMessage = root.value("message").toString("后端语义分割返回失败状态。");
                    return nullptr;
                }

                // 兼容后端直接返回 Base64 编码的 PCD 文件。
                if (root.contains("pcd_base64")) {
                    QByteArray pcdBytes = QByteArray::fromBase64(root.value("pcd_base64").toString().toUtf8());
                    return loadPointCloudFromPcdBytes(pcdBytes, errorMessage);
                }

                const QStringList pointKeys = {"body_points", "segmented_points", "points", "cloud_points"};
                for (const QString& key : pointKeys) {
                    if (root.contains(key) && root.value(key).isArray()) {
                        return jsonArrayToPointCloud(root.value(key).toArray(), errorMessage);
                    }
                }

                if (root.contains("data") && root.value("data").isObject()) {
                    QJsonObject dataObj = root.value("data").toObject();
                    for (const QString& key : pointKeys) {
                        if (dataObj.contains(key) && dataObj.value(key).isArray()) {
                            return jsonArrayToPointCloud(dataObj.value(key).toArray(), errorMessage);
                        }
                    }
                }

                errorMessage = root.value("message").toString("JSON 中未找到可解析的主体点云字段。");
                return nullptr;
            }
        }
    }

    // 优先兼容 WSL 服务返回的裸 float32 xyz 字节流。
    const bool looksLikePcdHeader =
        responseData.startsWith("# .PCD") ||
        responseData.startsWith("VERSION") ||
        responseData.startsWith("FIELDS") ||
        responseData.startsWith("DATA");

    if (!looksLikePcdHeader &&
        responseData.size() >= static_cast<int>(3 * sizeof(float)) &&
        responseData.size() % static_cast<int>(3 * sizeof(float)) == 0) {
        PointCloudT::Ptr rawCloud(new PointCloudT);
        const float *coords = reinterpret_cast<const float*>(responseData.constData());
        const int pointCount = responseData.size() / static_cast<int>(3 * sizeof(float));
        rawCloud->reserve(pointCount);
        for (int i = 0; i < pointCount; ++i) {
            PointT pt;
            pt.x = coords[i * 3 + 0];
            pt.y = coords[i * 3 + 1];
            pt.z = coords[i * 3 + 2];
            rawCloud->points.push_back(pt);
        }
        rawCloud->width = static_cast<std::uint32_t>(rawCloud->size());
        rawCloud->height = 1;
        rawCloud->is_dense = true;
        return rawCloud;
    }

    // 最后再兜底：把响应当作 PCD 文件处理。
    return loadPointCloudFromPcdBytes(responseData, errorMessage);
}

} // namespace
/*
 * 作用：
 * 当主体点云被重新提取后，旧的测量结果已经不再可信，因此需要统一清理缓存与图层状态。
 */
void SingleModePage::resetMeasurementState() {
    m_hasResults = false;
    m_latestResults = BodySizeResults();
    clearMeasurements();

    if (m_layerChecks.contains("Measurements")) {
        QSignalBlocker blocker(m_layerChecks["Measurements"]);
        m_layerChecks["Measurements"]->setChecked(false);
    }
}

/*
 * 作用：
 * 将内存中的点云落盘为临时 PCD 文件，便于以标准文件上传形式发送给 WSL 后端。
 * 这样后端可以像处理普通 PCD 文件一样执行读取、预处理和语义分割推理。
 */
QString SingleModePage::writePointCloudToTempPcd(const PointCloudT::Ptr& cloud, QString& errorMessage) const {
    if (!cloud || cloud->empty()) {
        errorMessage = "待上传的融合点云为空。";
        return QString();
    }

    QTemporaryFile tempFile(QDir::tempPath() + "/merged_cloud_upload_XXXXXX.pcd");
    tempFile.setAutoRemove(false);
    if (!tempFile.open()) {
        errorMessage = "无法创建用于上传的临时 PCD 文件。";
        return QString();
    }

    const QString tempPath = tempFile.fileName();
    tempFile.close();

    if (pcl::io::savePCDFileBinary(tempPath.toStdString(), *cloud) != 0) {
        QFile::remove(tempPath);
        errorMessage = "融合点云写入临时 PCD 文件失败。";
        return QString();
    }

    return tempPath;
}

/*
 * 作用：
 * 统一处理主体提取成功后的 UI 回填逻辑，避免传统提取和 WSL 语义分割各自维护一份重复代码。
 */
void SingleModePage::applyBodyCloudResult(const PointCloudT::Ptr& bodyCloud, const QString& sourceLabel) {
    if (!bodyCloud || bodyCloud->empty()) {
        log(sourceLabel + "返回了空主体点云。", "ERROR");
        QMessageBox::warning(this, "错误", sourceLabel + "失败，返回结果为空。");
        return;
    }

    resetMeasurementState();
    m_cloudData["Body"] = bodyCloud;

    // 关闭融合点云显示，让用户聚焦主体提取结果。
    if (m_layerChecks.contains("Merged")) {
        QSignalBlocker blocker(m_layerChecks["Merged"]);
        m_layerChecks["Merged"]->setChecked(false);
    }
    if (m_viewer) {
        m_viewer->removePointCloud("Merged");
    }

    if (m_layerChecks.contains("Body")) {
        m_layerChecks["Body"]->setChecked(true);
        onLayerToggle("Body", true);
    } else {
        update3DView();
    }

    log(QString("%1成功！主体点云点数：%2").arg(sourceLabel).arg(bodyCloud->size()), "SUCCESS");
}

// 实现主体精细提取的槽函数
void SingleModePage::onExtractBody() {
    if (!m_cloudData.contains("Merged") || m_cloudData["Merged"]->empty()) {
        QMessageBox::warning(this, "警告", "没有可用的融合点云！请先执行配准与融合。");
        return;
    }

    const int extractMode = m_comboBodyExtractMode ? m_comboBodyExtractMode->currentIndex() : 1;

    // =====================================================
    // A. 默认模式：调用 WSL 语义分割后端
    // =====================================================
    if (extractMode == 0) {
        QString serviceUrlText = m_leSegServiceUrl ? m_leSegServiceUrl->text().trimmed() : QString();
        if (serviceUrlText == "http://127.0.0.1:8000/segment") {
            serviceUrlText = "http://127.0.0.1:8002/predict";
            if (m_leSegServiceUrl) {
                m_leSegServiceUrl->setText(serviceUrlText);
            }
            log("检测到旧版分割服务地址，已自动切换为 WSL 分割默认地址: http://127.0.0.1:8002/predict", "INFO");
        }

        const QUrl serviceUrl(serviceUrlText);
        if (!serviceUrl.isValid() || serviceUrl.scheme().isEmpty()) {
            QMessageBox::warning(this, "错误", "WSL 语义分割服务地址无效，请检查后重试。");
            return;
        }

        QString serializeError;
        const QByteArray fileBytes = serializePointCloudToFloat32Bytes(m_cloudData["Merged"], serializeError);
        if (fileBytes.isEmpty()) {
            log(serializeError, "ERROR");
            QMessageBox::warning(this, "错误", serializeError);
            return;
        }

        log("开始调用 WSL 语义分割服务，上传融合后的完整点云...", "ALGO");
        // #region debug-point A:request-meta
        postQtSegDebugEvent("A",
                            "SingleModePage.cpp:onExtractBody",
                            "准备发送 WSL 语义分割请求",
                            QJsonObject{
                                {"serviceUrl", serviceUrl.toString()},
                                {"uploadByteCount", static_cast<qint64>(fileBytes.size())},
                                {"uploadFileName", "merged_cloud.bin"},
                                {"uploadFormat", "float32_xyz_binary"}
                            });
        // #endregion

        QHttpMultiPart *multiPart = new QHttpMultiPart(QHttpMultiPart::FormDataType);
        QHttpPart filePart;
        filePart.setHeader(QNetworkRequest::ContentDispositionHeader,
                           QVariant("form-data; name=\"file\"; filename=\"merged_cloud.bin\""));
        filePart.setHeader(QNetworkRequest::ContentTypeHeader, QVariant("application/octet-stream"));
        filePart.setBody(fileBytes);
        multiPart->append(filePart);

        QNetworkRequest request(serviceUrl);
        QNetworkReply *reply = m_networkManager->post(request, multiPart);
        multiPart->setParent(reply);

        if (m_btnExtractBody) {
            m_btnExtractBody->setEnabled(false);
            m_btnExtractBody->setText("WSL 语义分割中...");
        }
        if (m_comboBodyExtractMode) {
            m_comboBodyExtractMode->setEnabled(false);
        }
        if (m_leSegServiceUrl) {
            m_leSegServiceUrl->setEnabled(false);
        }

        connect(reply, &QNetworkReply::finished, this, [this, reply]() {
            // #region debug-point C:response-meta
            postQtSegDebugEvent("C",
                                "SingleModePage.cpp:onExtractBody:reply",
                                "收到 WSL 语义分割响应",
                                QJsonObject{
                                    {"networkError", static_cast<int>(reply->error())},
                                    {"httpStatus", reply->attribute(QNetworkRequest::HttpStatusCodeAttribute).toInt()},
                                    {"contentType", reply->header(QNetworkRequest::ContentTypeHeader).toString()},
                                    {"byteCount", static_cast<qint64>(reply->bytesAvailable())}
                                });
            // #endregion
            if (reply->error() == QNetworkReply::NoError) {
                const QByteArray responseData = reply->readAll();
                const QString contentType = reply->header(QNetworkRequest::ContentTypeHeader).toString();

                try {
                    QString parseError;
                    PointCloudT::Ptr bodyCloud = parseSegmentationReply(responseData, contentType, parseError);
                    if (bodyCloud && !bodyCloud->empty()) {
                        applyBodyCloudResult(bodyCloud, "WSL 语义分割提取");
                    } else {
                        log("WSL 语义分割结果解析失败: " + parseError, "ERROR");
                        QMessageBox::warning(this, "错误", "WSL 语义分割结果解析失败，请检查后端返回格式。");
                    }
                } catch (const std::exception& ex) {
                    const QString errorText = "WSL 语义分割结果解析时发生异常: " + QString::fromLocal8Bit(ex.what());
                    log(errorText, "ERROR");
                    QMessageBox::warning(this, "错误", errorText);
                } catch (...) {
                    log("WSL 语义分割结果解析时发生未知异常。", "ERROR");
                    QMessageBox::warning(this, "错误", "WSL 语义分割结果解析时发生未知异常。");
                }
            } else {
                const QString errorText = "WSL 语义分割请求失败: " + reply->errorString();
                log(errorText, "ERROR");
                QMessageBox::warning(this, "错误", errorText);
            }

            reply->deleteLater();

            if (m_btnExtractBody) {
                m_btnExtractBody->setEnabled(true);
                m_btnExtractBody->setText(m_comboBodyExtractMode && m_comboBodyExtractMode->currentIndex() == 0
                                              ? "🐷 调用 WSL 语义分割提取主体"
                                              : "🐷 执行传统规则主体提取");
            }
            if (m_comboBodyExtractMode) {
                m_comboBodyExtractMode->setEnabled(true);
            }
            if (m_leSegServiceUrl) {
                m_leSegServiceUrl->setEnabled(true);
            }
        });
        return;
    }

    // =====================================================
    // B. 备用模式：传统规则提取
    // =====================================================
    log("开始执行传统规则主体提取与平滑处理...", "ALGO");

    // 组装所有 UI 参数
    ExtractionParams params;
    params.boxMinX = m_spinBoxMinX->value();
    params.boxMinY = m_spinBoxMinY->value();
    params.boxMinZ = m_spinBoxMinZ->value();
    params.boxMaxX = m_spinBoxMaxX->value();
    params.boxMaxY = m_spinBoxMaxY->value();
    params.boxMaxZ = m_spinBoxMaxZ->value();
    params.boxRotZ = m_spinBoxRotZ->value();
    params.minClusterSize = m_spinExtMinPts->value();

    params.methodIndex = m_comboExtMethod->currentIndex();
    params.euclideanTolerance = m_spinEuclideanTol->value();
    params.rgNeighbors = m_spinRgNeighbors->value();
    params.rgSmoothness = m_spinRgSmoothness->value();

    // [新增] 收集 RANSAC 参数
    params.useRansac = m_chkUseRansac->isChecked();
    params.ransacDistThresh = m_spinRansacDist->value();

    params.useMlsUpsampling = m_chkMlsUpsampling->isChecked();
    params.mlsSearchRadius = m_spinMlsRadius->value();
    params.mlsUpsamplingRadius = m_spinMlsUpsampleRadius->value();
    params.mlsUpsamplingStep = m_spinMlsUpsampleStep->value();

    auto logBridge = [this](const QString& msg, const QString& level) {
        // 使用 QMetaObject::invokeMethod 保证跨线程 UI 调用的安全性
        QMetaObject::invokeMethod(this, [this, msg, level]() {
            this->log(msg, level);
        });
    };

    // 如果为了防止界面卡顿，你可以把这一步也放到 QtConcurrent::run 中
    // 这里为了简便和配准逻辑统一，直接在主线程/或使用异步调用
    PointCloudT::Ptr body_cloud = PointCloudAlgo::extractLargestCluster(m_cloudData["Merged"], params, logBridge);

    if (body_cloud && !body_cloud->empty()) {
        applyBodyCloudResult(body_cloud, "传统规则主体提取");
    } else {
        log("主体提取失败。", "ERROR");
        QMessageBox::warning(this, "错误", "主体提取失败，调整参数后再试。");
    }
}
