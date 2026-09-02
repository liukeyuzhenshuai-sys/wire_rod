# Double-Layer Support Semantics — V0.8

## 1. 业务模型

双层不是通用自由堆叠。第一层必须先占满 10 个 slot；进入第二层后第一层视为稳定支撑基础。第二层盘条位于两个相邻第一层盘条之间：`upper_slot k <-> supported_by(L1[k], L1[k+1])`。因此第二层身份的主证据是离散支撑拓扑，而不是最近历史中心。

## 2. 状态模式

- `FIRST_LAYER_LOADING`：L1 可沿 X 滚动/调整，使用原 normal/hard motion ROI。
- `SECOND_LAYER_LOADING`：L1 TrackState 冻结，L2 active；L1 显著运动是结构异常。
- `DOUBLE_LAYER_BOOTSTRAP`：系统首帧已经双层，无历史状态；利用层高度、10-slot foundation lattice、上层 support valleys 和正点云证据建立初始 TrackState。

## 3. 第一层 foundation lock

配置位于 `tracker_config.json -> double_layer`。多视角默认 normal/hard XY 为 0.08/0.15 m；TOP_DOWN_Z 使用更宽松的观测质心容差 0.12/0.20 m，避免把顶部残缺观测的 centroid bias 当成真实塌方。超 hard 范围输出 `ABNORMAL_FIRST_LAYER_SHIFT_HARD`，可靠 L1 TrackState 不更新。

## 4. 第二层 support slots

当前观察的 L2 instance 先投影到最近的合法 support valley。normal/hard center error 默认 0.42/0.65 m。历史 L2 track 持久保存 `slot`，其身份含义就是对应 support pair。这样即使上层发生小范围滚动/yaw/观测变化，只要仍在同一支撑谷，ID 不应交换。

## 5. Bootstrap

若首帧已收到第二层信号，算法先从 Z 结构分层，要求建立 10 个 L1 foundation tracks，再把上层实例分配到离散 support slots。严重遮挡允许点数显著减少，但每个被初始化为确定身份的物理盘条必须有正点云证据。完全零点不能通过“理论上应该存在”被初始化为确定 ID。

## 6. 异常

- `ABNORMAL_FIRST_LAYER_SHIFT_HARD`：L2 已激活但 L1 发生超硬阈值运动。
- `ABNORMAL_LAYER2_SUPPORT_SLOT_MISSING`：上层实例不在任何合法支撑槽硬范围内。
- `ABNORMAL_LAYER2_CAPACITY`：超过配置的第二层容量。
- `ABNORMAL_LAYER2_WITH_INCOMPLETE_L1`：L1 未满就收到/推断 L2。

所有异常允许输出 UNCERTAIN；错误 Global ID 仍然是更严重的失败。

## 7. 数据集

D01–D10 共 37 帧。D05/D06 专门测试双层 bootstrap；D08/D09/D10 是故意的异常场景。`labels/support_gt.csv` 提供 support relation GT，`labels/frame_semantics.csv` 提供 L1/L2 slot 状态。所有 D 系列物理对象 `point_count > 0`。

## 8. 回归期望

`python tests/run_double_layer_regression.py` 必须满足：D01–D10 `wrong_id_total=0`；D05/D06 bootstrap 生成 10 个 L1 foundation；D08/D09/D10 命中预期异常原因；D08/D09 后续恢复到 OK；D 系列无 zero-point physical object。
