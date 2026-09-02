# 盘条语义状态跟踪器 —— 设计接手入口

如果没有原聊天记录，从这里开始。

## 当前状态

- 数据集：V3.1，自包含、可复现，68 个生成 PLY 全部位于本目录根层。
- 数据生成逻辑已冻结并有源码/标注。
- 跟踪算法：需求和详细设计已冻结，尚未正式实现 semantic matcher。
- 下一步：按 `docs/IMPLEMENTATION_AND_TEST_PLAN.md` 从 M0 开始编码。

## 必读顺序

1. `HANDOFF.md` —— 数据集与场景定义。
2. `docs/TRACKER_REQUIREMENTS_V1.md` —— 要做什么、什么不能做。
3. `docs/TRACKER_DETAILED_DESIGN_V1.md` —— 模块、数据结构、算法和 DP。
4. `docs/IMPLEMENTATION_AND_TEST_PLAN.md` —— 实施顺序和每阶段 Exit 条件。
5. `docs/DECISIONS_AND_TBDS.md` —— 哪些已确认、哪些必须真实数据标定。
6. `docs/ALGORITHM_TEST_GUIDE.md` —— 如何读取 V3.1 与评测。
7. `design/tracker_config.example.json` —— 配置字段；所有 `TBD_*` 禁止凭空写死成生产参数。

## 最核心的五句话

1. global ID 是长期身份；current instance 只是观测。
2. 先锁高置信可视组合，再解释 NEW/OCCLUDED。
3. 同 Cluster Slot 顺序是硬约束，不能为了距离更近而交换 ID。
4. 允许 UNCERTAIN，不允许错绑 ID。
5. 当前残缺点云的几何必须接受历史 D/L/ROI 反向约束。

## 直接开始开发

当前仓库已有：

```text
code/dataset.py
code/ply_io.py
code/evaluate_tracking.py
labels/frame_pairs.csv
labels/transition_gt.csv
```

新建 `tracker/` 包，不修改数据生成器。第一个可运行目标：

```text
S01 + S02 + S10
```

实现：

```text
load -> geometry -> TrackState -> ROI -> candidates -> ordered DP -> predictions.csv
```

先要求 `S10 WRONG_ID=0`，再扩展 S06/S07 的遮挡与 NEW。

## 发生冲突时的裁决优先级

```text
硬物理/语义约束
> 已确认历史状态与 Motion ROI
> 机器人 NEW 先验
> 稳定 D/L
> 当前几何观测
> 残缺/遮挡点云外观
```

证据不足：`UNCERTAIN`。


## V0.8 current design delta

The executable reference implementation is now the double-layer build. Once second-layer loading is active, L1 is a locked foundation, and L2 identity is keyed primarily by support slot `k = supported_by(L1[k], L1[k+1])`. Startup can enter `DOUBLE_LAYER_BOOTSTRAP`; severe occlusion is supported only with positive current point evidence. See `docs/DOUBLE_LAYER_SUPPORT_V0_8.md` and `results/FINAL_VALIDATION_V0_8.json`.
