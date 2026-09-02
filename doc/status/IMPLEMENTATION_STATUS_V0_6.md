# Semantic Guard V0.6 实现状态

## 目标

V0.6 针对用户保存的失败案例 `failure_20260818_163534_90ea5def0107_004.zip` 重构生产匹配路径。核心目标不是降低局部几何 cost，而是把已确认的业务语义变成求解空间硬约束，并允许 `UNCERTAIN / ABNORMAL` 保护 Global ID 精度。

## 已实现

1. **语义先于几何**：同层历史 Track 先按 `slot` 排序，当前 instance 按空间顺序排序；机器人 NEW hint 决定 expected NEW 数量并枚举合法 NEW 插入位置。去掉 NEW 后，历史与当前旧实例做保序一一对应。
2. **正常模式不允许 `SKIP_HISTORY`**：当 `current_count != history_count + expected_new_count` 时直接进入 `ABNORMAL_CARDINALITY`，不再用“零点隐藏盘条”补齐序列。
3. **OCCLUDED 必须有正观测**：只有匹配到非空 current instance 且可见点比例显著降低时才允许 `OCCLUDED/PARTIAL_VISIBLE`。零点历史对象不能称为 OCCLUDED。
4. **几何 Anchor 降级为诊断**：Anchor 不再直接提交 Global ID；必须先通过语义布局。
5. **几何只验证语义映射**：ROI / D / L / yaw 等用于确认或降级。若语义 rank 唯一但几何超出安全范围，保留当前 instance 证据并输出 `UNCERTAIN`，不允许后续 ID 补位。
6. **异常状态不污染历史**：UNCERTAIN 当前 instance 会显示给用户，但 persistent TrackState 保留最后可靠状态，不用不安全结果覆盖。
7. **Web 显示帧级状态与原因**：`frame_status`、`semantic_reason_codes`、UNCERTAIN/UNOBSERVED 均可在结果和失败案例包中保存。
8. **失败案例包增强**：新增 `failure_summary.json`，保存帧状态、语义原因码、历史/当前数量、UNCERTAIN/UNOBSERVED 统计。

## 用户失败案例回归

上一帧：`S05_F03_yaw_motion_3.ply`

当前帧：`USER_90ea5def0107_F004.ply`

V0.5 错误：跳过 ID2，造成 ID3/ID4/ID5 整链错位，并生成一个没有当前点云的历史代理。

V0.6：

- ID0 -> instance109 MATCHED
- ID1 -> instance613 MATCHED
- ID2 -> instance850 MATCHED
- ID3 -> instance744 MATCHED
- ID4 -> instance533 MATCHED
- ID5 -> instance292 UNCERTAIN（实际存在点云，但 X 位移约 2.25m，超过 2.0m absolute ROI）
- NEW6 -> instance700 NEW
- committed WRONG_ID = 0

运行：

```bash
python tests/run_failure_case_90ea.py
```

结果：`results/failure_case_90ea_v0_6.json`

## 回归结果

### 推荐 pairwise

87 pairs：

- WRONG_ID = 0
- validator failure = 0
- NEW TP/FP/FN = 39/0/0
- legacy OCC GT TP/FP/FN = 8/1/0
- UNCERTAIN = 1

注意：旧数据中的零点遮挡标签属于 legacy GT；V0.6 的生产定义不再把零点对象当 OCCLUDED，因此 OCC 指标只能作为旧数据兼容统计，不能覆盖新业务定义。

### 复杂连续序列

C01-C10，52 帧：

- cumulative WRONG_ID = 0
- 在部分旧复杂序列中 UNCERTAIN 数量明显上升，这是新安全策略主动拒绝旧版“零点隐藏/不满足数量守恒”解释的结果，不应调参强行消除。

### 交互式 smoke

创建下一帧 + 移动 + yaw + partial occlusion + NEW + 实际 tracker 流程通过，WRONG_ID=0。

## 尚未声称完成的事项

- “中间空间能否容纳一个完全不可见历史盘条”的 physical-fit 检查不再属于正常求解路径，因为 V0.6 已禁止正常零点隐藏盘条；后续若业务重新允许完全不可视对象，再单独实现异常模式的 hidden-object physical-fit validator。
- 现有 V3/V4 数据中存在旧版零点 OCCLUDED GT，后续建议重生成一套完全符合新遮挡定义的数据集，而不是修改生产算法去迎合旧标签。
