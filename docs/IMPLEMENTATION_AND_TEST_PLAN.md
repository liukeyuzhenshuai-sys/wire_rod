# V1 实施与测试计划

## 1. 开发原则

按“先保证 0 错 ID，再提高 coverage”的顺序开发。任何阶段如果需要通过降低 UNCERTAIN 阈值来换 coverage，必须先验证没有产生新 WRONG_ID。

---

## 2. Milestone M0：工程骨架与输入隔离

### 实现

- 建立 `tracker/` 包；
- 读取 PLY 仅返回 XYZ + instance_id；
- 读取 `frame_pairs.csv`；
- 产生空的 prediction/result；
- 加阶段计时与日志。

### 测试

- `test_no_gt_leak.py`：断言 pipeline 对 PLY 的 GT 属性不可见；
- `P00/P01` 能加载和拆实例。

### Exit

- 任意 V3.1 PLY 可稳定加载；
- 相同 seed/config 输出可复现。

---

## 3. M1：实例几何观测

### 实现

- PCA axis；
- robust projected length；
- 截面 D/center 初版；
- geometry_quality；
- 历史 D/L prior 接口。

### 测试

- P01：rollObs / yaw / move；
- P02：左右遮挡不应把中心拉到中间缺口模型；
- S12：跨视角 observation 变化。

### Exit

- 能为所有可视实例输出合法 center/yaw/D/L/quality；
- partial 不因 bbox 缺失导致 D/L 大幅异常；
- 失败时 quality 下降，而不是异常值无提示。

---

## 4. M2：历史状态、ROI、Candidate Graph

### 实现

- TrackState；
- LayerState；
- MotionROI；
- position/D/L/yaw gating；
- candidate audit。

### 测试

- S05：移动 + yaw；
- S10：相似几何下候选不能仅靠 D/L 唯一化。

### Exit

- 候选图稀疏；
- 真值对应不能被正常阈值错误 gate 掉；
- 明显不可能的跨层/超 ROI 候选被拒绝。

---

## 5. M3：Anchor + Ordered DP

### 实现

- Anchor cost + margin；
- 一对一；
- MATCH / SKIP_HISTORY / INSERT_CURRENT DP；
- Anchor 分区；
- 同 Cluster 顺序硬约束。

### 测试

- S01/S02：单端增长；
- S10：ID swap 压力。

### Exit

- S10 `WRONG_ID=0`；
- 无法确定时输出 UNCERTAIN，不允许顺序反转。

---

## 6. M4：第一层双端 Cluster + NEW

### 实现

- left/right Cluster；
- normal middle free region；
- 合法 inward growth；
- NEW hint 接口；
- 多 NEW 支持。

### 测试

- S03：双端增长；
- S09：一次两个 NEW；
- S11：第一层满 10 后第二层 NEW。

### Exit

- S03 中间大空区不触发 gap 异常；
- 不产生第 11 个 layer-1 Slot；
- NEW 不抢占旧 global ID。

---

## 7. M5：遮挡生命周期

### 实现

- unmatched history 解释器；
- PARTIAL/OCCLUDED/UNCERTAIN；
- Track 保留与恢复；
- 不允许 OCCLUDED ID 被复用。

### 测试

- S06：visible→partial→occluded→return；
- S07：NEW + OCCLUDED 同帧。

### Exit

- S06/S07 `WRONG_ID=0`；
- 完全遮挡期间 global ID 持续存在；
- 恢复后回到原 ID。

---

## 8. M6：Layer 2 与 gap

### 实现

- layer 不变约束；
- layer 2 NEW；
- direct semantic neighbors；
- yaw-aware model gap；
- gap uncertainty。

### 测试

- S04：内部 gap sweep；
- S08：两层；
- S11：满层转 layer2。

### Exit

- 不跨层；
- 只对同 Cluster 直接邻居报告 gap；
- 正常中间空区不报告成 gap 异常。

---

## 9. M7：完整评测、置信度与性能

### 实现

- best/second hypothesis margin；
- conservative confidence；
- frame_result.json；
- match_audit.csv；
- 扩展 evaluator；
- stage timing。

### 测试

- 所有 `temporal_adjacent` pair；
- S13 100k+ 点。

### 硬 Exit

```text
WRONG_ID = 0
S10 no swap
S11 layer1 <= 10
S06/S07 occluded ID not reused
S13 total < 2 s on documented hardware
```

覆盖率、NEW/OCC 指标同时记录，但在真实数据基线建立前不为了追 coverage 牺牲上述硬门槛。

---

## 10. M8：真实生产数据标定

数据到位后按以下顺序统计，不先凭经验改算法：

1. 同一静止盘条跨视角 center/D/L 漂移；
2. 历史盘条典型 X/Y/Z 位移；
3. NEW 周边连锁移动分布；
4. robot target error；
5. layer Z/支持关系；
6. gap 真实分布；
7. geometry quality 与真实误差关系。

然后冻结生产 `tracker_config.json`。

---

## 11. 回归测试顺序

每次改 matcher 必跑：

```text
S10 -> 先看是否出现 ID swap
S06 -> 遮挡期间是否污染 ID
S07 -> NEW + OCC 是否互相抢 ID
S03 -> 双端 Cluster / 中间空区
S11 -> layer1 capacity
S01/S02/S05/S08/S09 -> 功能覆盖
S13 -> 性能
```

若 S10 出现任何 WRONG_ID，本次改动直接视为回归失败，不继续调 coverage。

## V0.7 回归新增

- `tests/run_motion_envelope_regression.py`：验证 normal/hard 双包络、ABNORMAL motion 与 identity 解耦、hard 越界不导致 suffix ID shift。
- `tests/run_topdown_regression.py`：连续运行 Z01–Z05，检查 `WRONG_ID`、`UNCERTAIN` 和 zero-point GT object。
- `code/generate_topdown_z_dataset.py`：从已有真实原型/场景派生 TOP_DOWN_Z 与 view-switch 序列；必须可幂等重建。
- 每次修改 matcher / geometry / topdown view model 后至少重跑 failure 90ea、motion envelope、87 pairwise、C01–C10、Z01–Z05。



## V0.8 回归新增
新增 `tests/run_double_layer_regression.py` 和 D01–D10。正常序列 D01–D07 必须无异常且 WRONG_ID=0；D08 必须捕获 L1 foundation hard shift；D09 必须捕获 L2 off-support；D10 必须捕获第二层容量超限；D05/D06 必须通过双层 bootstrap；全部 D-series physical object 必须 point_count>0。
