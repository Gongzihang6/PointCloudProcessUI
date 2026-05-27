# [OPEN] qt-wsl-seg-crash

## 背景
- 症状：Qt 程序在执行 WSL 语义分割时直接闪退。
- 已知现象：调试器弹窗显示 `pcl::IOException` 未处理异常。
- 目标：定位是 Qt 上传、WSL 返回、Qt 解析返回、还是异常未捕获导致的崩溃。

## 当前假设
1. Qt 端把后端返回内容按 PCD 解析，但后端实际返回的是 JSON / 文本错误页 / 非 PCD 二进制，触发 `pcl::IOException`。
2. Qt 端请求的接口地址、字段名或协议与 `PigSegPrediction_Qt.py` 不一致，服务端返回 4xx/5xx 文本，随后被 Qt 错误解析。
3. WSL 后端虽然收到了文件，但读取上传内容的格式假设与 Qt 上传格式不一致，导致服务端异常，Qt 又未对错误响应做健壮处理。
4. Qt 端在网络回调里直接调用 PCL 读取函数，异常没有被 `try/catch` 包裹，导致即便只是解析失败也会直接闪退。
5. 分割服务返回的是点数组/自定义 JSON，但 Qt 当前优先走“按 PCD 字节流落盘再加载”的分支，分支判断条件不正确。

## 证据计划
- 阅读 `PigSegPrediction_Qt.py`，确认 `/segment` 的请求协议与返回格式。
- 阅读 Qt 端 `onExtractBody()`、`parseSegmentationReply()`、`loadPointCloudFromPcdBytes()`，确认异常点。
- 对比关键点服务 `OffsetKeyPointPrediction_Qt.py` 的接口风格，看两端协议是否统一。
- 如静态证据不足，再补充最小化日志插桩并指导复现。

## 状态
- 进行中：静态证据收集

## 已收集证据
- `PigSegPrediction_Qt.py` 的 `/predict` 返回 `application/octet-stream`，内容明确写的是 `(M, 3)` 的 `float32` 猪体点云坐标，不是 PCD 文件。
- Qt 端 `parseSegmentationReply()` 对非 JSON 响应会直接走 `loadPointCloudFromPcdBytes()`，把收到的字节写成临时 `.pcd` 再调用 `pcl::io::loadPCDFile`。
- 若收到的是裸 `float32` 数组，`pcl::io::loadPCDFile` 会抛出 `pcl::IOException`；这与用户截图中的未处理异常类型一致。
- Qt 端上传时当前发送的是临时 `merged_cloud.pcd` 文件内容，而 `PigSegPrediction_Qt.py` 的 `parse_point_cloud_buffer()` 期望的是纯 `float32 xyz` 二进制，不是 PCD 文件。

## 当前结论
- 高概率根因 1：返回协议不一致。WSL 返回裸 `float32` 点云，Qt 却按 PCD 解析，导致未捕获的 `pcl::IOException` 直接崩溃。
- 高概率根因 2：请求协议也不一致。Qt 上传的是 PCD 文件，而 WSL 服务按裸 `float32 xyz` 解析，存在服务端 400/500 风险。

## 已实施修复
- Qt 默认 WSL 语义分割服务地址从旧的 `http://127.0.0.1:8000/segment` 调整为 `http://127.0.0.1:8002/predict`。
- 若用户界面中仍保留旧地址，运行时会自动迁移到新地址，避免“后端无反应”。
- Qt 上传格式已从临时 `.pcd` 文件改为裸 `(N,3)` `float32 xyz` 二进制，和 `PigSegPrediction_Qt.py` 的 `parse_point_cloud_buffer()` 对齐。
- Qt 返回解析已优先兼容裸 `(M,3)` `float32 xyz` 二进制，仅在检测到 PCD 头时才回退到 PCD 解析。
- Qt 分割结果解析增加 `try/catch`，避免再因 `pcl::IOException` 直接闪退。

## 待验证
- 重新编译并运行后，验证 WSL 分割是否能正常触发后端、返回主体点云，并且不会清空当前会话中的其它数据。

## 新证据（用户复现后）
- WSL 控制台出现 `POST /predict HTTP/1.1" 200 OK`，说明 Qt 请求已经成功打到 `PigSegPrediction_Qt.py`。
- Qt 端报错变为“后端返回内容为空”，说明当前问题已从“协议错误/崩溃”收敛为“模型推理结果为空”。
- 根据 `PigSegPrediction_Qt.py` 的返回逻辑，只有 `pig_points.astype(np.float32).tobytes()` 长度为 0 时，Qt 才会收到空响应体。

## 更新结论
- “地址错误 / 请求未到后端”假设已排除。
- 当前高概率根因是：模型最终没有筛出任何 `pig_points`，即 `pig_points.shape[0] == 0`。
- 最可能的具体原因：
  1. `pig_label` 与训练时真实猪类别编号不一致；
  2. 模型预测类别里根本没有落到当前 `pig_label`；
  3. 开启阈值分支时，`preds = (...).long()` 与 `point_preds == pig_label` 的组合存在潜在标签逻辑错误；
  4. 返回空结果时后端仍返回 `200 + 空 body`，Qt 只能看到“内容为空”，不利于定位。
