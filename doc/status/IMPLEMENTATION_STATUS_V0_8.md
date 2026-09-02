# Implementation Status — V0.8

## 已实现

- 双层装载模式切换和外部 second-layer signal。
- L1 必须 10/10 才允许正常 L2。
- L2 激活后的 L1 foundation lock。
- L2 support-slot identity：slot k = L1[k]+L1[k+1]。
- L2 NEW 按 robot hint + support slot 判断。
- Double-layer bootstrap，包含 TOP_DOWN_Z 严重遮挡场景。
- Web 编辑器 Layer2 NEW + 支撑槽吸附 + 第二层信号。
- D01–D10 37 帧、自包含 GT、support_gt、sequence/pair 标签。
- 保留 V0.7 TOP_DOWN_Z 与运动 normal/hard envelope。
- 仍执行正点云遮挡规则：零点不是 OCCLUDED。

## 2026-08-18 最终回归

- failure 90ea：PASS，错误链式 ID shift 不再出现。
- motion envelope：PASS；normal/hard 边界按预期。
- recommended pairwise：114 pairs，WRONG_ID=0，validator failures=0。NEW TP/FP/FN=61/0/0。OCC GT 统计 TP/FP/FN=8/3/0；3 个 FP 属于现有观测状态分类差异，不影响 Global-ID primary metric，后续可单独校准 visibility classifier。
- C01–C10：52 frames，WRONG_ID=0。复杂旧集因最新保守语义存在较多 UNCERTAIN，这是允许行为。
- Z01–Z05：25 frames，WRONG_ID=0，UNCERTAIN=0，zero-point GT=0。
- D01–D10：37 frames，WRONG_ID=0，UNCERTAIN=11；这 11 个全部来自故意异常：D08=1、D09=1、D10=9。所有双层 assertions 通过。
- S13：128,769 points，load+geometry 约 0.376 s < 2 s target。

## 当前已知边界

- L1 foundation lock 阈值、L2 support error 0.42/0.65 m 仍需真实车厢数据标定。
- TOP_DOWN_Z 的下层 centroid/yaw 在重遮挡下有系统偏差，目前用稳定历史几何 + 更宽 foundation observation envelope 处理。
- Bootstrap 假设每个实际盘条仍至少有一部分实例点；如果以后出现真正 0-point 不可视盘条，需要增加独立 missing-object model，而不能复用 OCCLUDED。
