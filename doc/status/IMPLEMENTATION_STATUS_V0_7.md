# Implementation Status — V0.7

## 已实现

### A. 语义身份 + 运动异常解耦

- normal X envelope：2.0 m。
- hard X envelope：2.4 m。
- normal < |dx| <= hard 时，如果 order/cardinality 唯一、无第二合法身份解释、D/L/yaw 与 Y/Z 约束通过，可以 `MATCHED`，但 frame 标记 `ABNORMAL_X_MOTION_SEMANTIC_RESCUE`。
- 超过 hard envelope 后身份必须 `UNCERTAIN`，且不能用后续 ID 补位。
- reason 包含 dx、normal/hard limit、超限米数和百分比。
- Web 编辑器读取 session config 实时显示 normal/hard 越界预警，只警告不禁止。

### B. TOP_DOWN_Z

- 全场 XY z-buffer 生成顶视点云。
- Web 创建下一帧支持 `常规多视角 / 仅顶部 -Z`。
- Matcher 仅用 XYZ 自动判断 top-only observation。
- TOP_DOWN_Z center Z 使用 `top Z - radius` 恢复。
- 历史 stable D/L 用于残缺 top-only 匹配。
- 两层 top-only 使用 semantic cardinality + top-Z 解决 layer assignment。
- `VIEW_PARTIAL` 与真实 `OCCLUDED` 分离。
- Z01–Z05 共 25 帧，无 zero-point GT object。

## 当前回归

- Failure case 90ea：`WRONG_ID=0`；ID5 `dx≈2.245m` 保持 `MATCHED` 并显式 ABNORMAL。
- Motion hard-envelope test：再推 0.30 m 后 ID5 `UNCERTAIN`，ID0–ID4 不发生 suffix shift。
- 87 pairwise：`WRONG_ID=0`，validator failure=0。
- C01–C10：52 continuous frames，`WRONG_ID=0`；旧复杂数据因历史 zero-point/旧语义定义会产生较多保守 UNCERTAIN，这是预期行为。
- Z01–Z05：25 continuous frames，`WRONG_ID=0`，`UNCERTAIN=0`，zero-point GT=0。

## 仍需真实数据校准

- 2.0 / 2.4 m X 包络来自当前工程假设和压力测试，不应视为最终生产参数。
- top-down z-buffer 的 XY cell / surface thickness 需由真实相机密度和重建噪声校准。
- `Z span / D` top-only detector 阈值需用真实多相机/纯顶相机样本做 ROC/误判评估。
- 如果未来允许一个历史盘条完全不可见，需要新增独立 `UNOBSERVED` 业务模式；当前版本明确不支持把零点解释成正常 OCCLUDED。
