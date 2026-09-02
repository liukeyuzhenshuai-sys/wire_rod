# 复杂组合数据集 V4（Web V0.3）

## 目的

原 V3/V3.1 继续保留，作为单因素回归集。本扩展新增 C01-C10，目标是验证算法是否真正依赖 Layer / Slot / Cluster / 历史 ROI / NEW hint / 遮挡语义，而不是因为旧数据过于简单而碰巧正确。

## 新增场景

- **C01**：左侧增长；所有旧盘条都移动；NEW 大 gap + yaw；Y/Z 小漂移；partial + roll 组合。
- **C02**：右侧镜像复杂增长。
- **C03**：左右双 Cluster 交替/同时新增；所有盘条移动；遮挡；正常中间大空区。
- **C04**：同规格/同原型最近邻陷阱；整组移动超过半个槽距；同规格 NEW；partial/OCCLUDED。
- **C05**：NEW + OCCLUDED + 大 gap + yaw + 全体移动同时发生。
- **C06**：第一层满 10 + 连续第二层新增；上下层均移动；下层 partial/OCCLUDED/恢复。
- **C07**：左右两侧一次同时新增多个盘条；后续遮挡恢复。
- **C08**：机器人预计落点误差 + 大位移；整组单帧平移约 1.05 m；Y/Z 漂移；反向回弹。
- **C09**：多盘条 partial + 一只完全 OCCLUDED + 恢复时同时 NEW。
- **C10**：非单向运动/回弹；X/Y 运动、yaw、rolling mismatch，然后 far-gap NEW。

## 物理规则仍然冻结

1. 第一层最多 10 个 Slot。
2. 第一层从左右边界向中间增长，可存在两个 Cluster；中间大空区不是异常 gap。
3. 同 Cluster 顺序不交换，不换层。
4. 旧盘条不删除；`OCCLUDED` 必须仍有正 current instance。完全零点不属于正常遮挡；当前业务模型将其视为异常/UNCERTAIN，并禁止靠隐藏盘条造成后续 ID 补位。
5. 滚动观测变化不通过绕自身轴刚体旋转模拟，不制造随机中央大洞。
6. 邻居遮挡从左右边缘产生；partial 与 rolling observation change 可以组合。
7. NEW 的 target 是近似值，不等于 GT 中心；本扩展故意加入 0.1-0.4 m 级 target 偏差。

## 推荐目检顺序

C04 -> C05 -> C03 -> C08 -> C06 -> C07 -> C01/C02 -> C09 -> C10。

Web 中选择对应 Sequence 第一帧，然后一直点击“下一帧”。不要跳帧，因为 C 系列专门用于验证持久 TrackState 的累计稳定性。

## 标注

- `labels/object_gt.csv`：逐帧逐盘条真值。
- `labels/transition_gt.csv`：前后帧 SAME_OBJECT / NEW、instance 对应、位移与 visibility event。
- `labels/frame_semantics.csv`：Layer1 Slot、左右 Cluster、中间正常 free slots。
- `labels/neighbor_gt.csv`：仅连续 Cluster 内 direct neighbor 的 model gap。
- `labels/complex_sequence_catalog.csv`：C01-C10 每一帧的组合因素和难度。
