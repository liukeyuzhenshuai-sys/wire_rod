# 场景/帧关系说明

文件命名：`<Sequence>_F<frame>_<scene>.ply`。相同 Sequence 不一定都表示真实连续时间；必须查看下面的关系模式。

| Sequence | 关系 | 作用 |
|---|---|---|
| P00 | 独立 | 7 个真实原型归一化结果，不是时序 |
| P01 | 每个变体对 F00 | 单盘条滚动观测、yaw、移动、partial、视角变化压力 |
| P02 | 每个变体对 F00 | 专门检查左右边缘遮挡与“滚动不挖洞” |
| S01 | 连续 | 左侧 Cluster 3→4→5→6，逐步 NEW |
| S02 | 连续 | 右侧 Cluster 3→4→5，向中间增长 |
| S03 | 连续 | 左右两端轮流增长，中间大空区为正常 |
| S04 | 每个变体对 F00 | 同一 Cluster 内 gap 从 2cm→10cm→28cm→45cm 的参数压力，不代表实际时间移动 |
| S05 | 连续 | 历史盘条 X 位移 + yaw + rolling observation mismatch |
| S06 | 连续 | ID2：VISIBLE→PARTIAL→OCCLUDED→PARTIAL→VISIBLE |
| S07 | 连续 | 一次同时出现 NEW 和历史盘条 OCCLUDED |
| S08 | 连续 | 两层，第二层新增，底层 partial；绝不换层 |
| S09 | 连续 | 一次新增两个盘条，左右 Cluster 同时增长 |
| S10 | 连续 | 多个实例使用相同/相似真实几何原型，专门压测 ID swap |
| S11 | 连续 | 第一层已满 10 个；再新增必须进入第二层 |
| S12 | 每个变体对 F00 | clean/top/side/degraded 跨视角观测压力 |
| S13 | 独立 | 10+8、约 12.9 万点性能压力 |
| S14 | 独立 | NEW+yaw+partial+occluded+gap 组合快照；没有前置帧，不用于单独 ID 评测 |

## 推荐测试顺序

1. P01/P02：先验证几何/观测鲁棒性。
2. S01/S02/S03：验证 NEW 与 Slot/Cluster 语义。
3. S06/S07：验证“先锁定可视组合，再解释 OCCLUDED/NEW”。
4. S10：验证绝不能 ID 交换。
5. S08/S11：验证 Layer 约束。
6. S13：最后测耗时。

具体可测试 pair 已写入 `labels/frame_pairs.csv`，不要自行用文件名猜 pair。
