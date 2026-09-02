# 盘条语义状态跟踪器 V1 —— 软件需求规格说明书（SRS）

**文档版本**：1.0  
**设计基线**：`coil_semantic_dataset_v3_1_handoff`  
**状态**：需求冻结，可进入 V0 实现  
**首要业务原则**：**global ID 绝不能错绑；证据不足时允许 `UNCERTAIN`。**

---

## 1. 文档目的

本项目不是通用点云 ReID，也不是逐点 ICP 配准。系统目标是：

\[
\boxed{
历史状态 + 装载语义 + 运动限制 + 当前实例点云 + 可选机器人新增先验
\rightarrow 当前最可信的盘条 global\ ID 布局
}
\]

本文冻结需求边界、输入输出、硬约束、异常策略和验收方法，使后续开发者在没有原聊天记录的情况下仍可继续实现。

详细算法见 `TRACKER_DETAILED_DESIGN_V1.md`；实施顺序见 `IMPLEMENTATION_AND_TEST_PLAN.md`。

---

## 2. 已确认的业务场景

### 2.1 点云与采集

1. 每次采集发生在盘条放置后，属于低帧率静态快照，而非连续运动视频。
2. 点云由多相机三维重建获得；不同帧可能由不同视角组合主导，但坐标系基本一致。
3. 单点存在约 3–5 cm 量级重建噪声；整帧坐标系整体变化较小。
4. 每个点已具有单帧 `instance_id`；同一个 `instance_id` 在不同帧没有身份意义。
5. 实例分割可基本认为 1 个 instance 对应 1 个真实盘条；merge/split 不是 V1 主问题。
6. 当前测试数据算法合法输入仅为 `XYZ + instance_id`。PLY 内 RGB、global_id、layer、slot、visibility 都属于可视化/GT，不得用于推理。

### 2.2 盘条物理与几何

1. 盘条近似横放圆柱，轴通常接近 Y 方向，但允许 XY 平面小幅 yaw。
2. 盘条主要沿 X 方向滚动/移动，历史对象最大 X 搜索范围暂定上限约 2 m；实际 ROI 应可配置并可按对象自适应缩小。
3. Z 方向不会发生换层式移动；同一个盘条跨帧 Z 变化应明显小于 X 变化。
4. 盘条不会从第一层换到第二层或反之。
5. 历史盘条正常不会被移走；若出现“无遮挡且历史盘条消失”，应输出异常，而不能用其他盘条顶替其 ID。
6. 外径业务范围大致 1.1–1.5 m，必须可配置。
7. 长度业务范围大致 0.5–1.7 m，必须可配置。
8. 点云通常只看到上部/部分表面；端面可能完全不可见，但沿轴向长度通常相对更完整。
9. 盘条滚动后可见物理表面会改变，因此前后表面点云不是稳定身份指纹。

### 2.3 装载语义

1. 第一层最多 10 个语义 Slot，不允许出现第 11 个第一层盘条。
2. 第一层可按三种方式增长：
   - 左边缘向中间；
   - 右边缘向中间；
   - 左右两边轮流向中间。
3. 第一层可能同时存在左、右两个连续 Cluster；两个 Cluster 之间的大空区是正常状态。
4. **只在同一连续 Cluster 内检查直接相邻 Slot 的 gap。** 左右两个 Cluster 中间的正常空区不能判为异常 gap。
5. 同一层、同一连续 Cluster 内的物理顺序不能交换，即盘条不能互相穿越完成 ID 反转。
6. 第二层通常约 8 个盘条；当前只作为业务规模/性能参考，不把“8”写成未经确认的硬容量上限。
7. 一次通常新增 1 个盘条，但算法必须兼容一次新增多个。
8. 机器人可提供新增盘条名义外径、长度和大致目标位置；这些是合法的强先验，但其误差范围仍待生产数据标定。

---

## 3. 系统目标与非目标

### 3.1 V1 必须完成

系统接收前一时刻已确认历史状态和当前帧实例点云，输出：

- 当前可见实例对应的稳定 `global_id`；
- `MATCHED / NEW / UNCERTAIN`；
- 对没有当前 instance 的历史 ID 输出 `OCCLUDED / UNCERTAIN / ABNORMAL_MISSING`；
- 每个盘条当前估计位置 `center_xyz`；
- 轴方向 / yaw；
- 稳定外径与长度；
- 同一 Cluster 直接邻居及估计 gap；
- layer / slot / cluster 状态；
- 匹配置信度和主要证据；
- 每阶段耗时。

### 3.2 V1 明确不要求

- 不要求恢复盘条绕自身轴滚动了多少角度；
- 不要求利用局部表面纹理/点云外观做 ReID；
- 不要求完整 6DoF ICP；
- 不要求处理旧盘条正常移除；检测到此类情况应报异常；
- 不要求处理高频运动轨迹；
- 不要求在 V1 完成真实多相机 ray-tracing 遮挡建模；相机姿态接口预留给后续版本。

---

## 4. 关键术语

| 术语 | 定义 |
|---|---|
| `instance_id` | 当前帧实例分割 ID，仅单帧有效 |
| `global_id` | 盘条长期身份，跨帧必须保持真实物理一致 |
| `Track` | 一个 global ID 的长期状态 |
| `Slot` | 同层语义顺序位置，不等价于固定 XYZ 坐标 |
| `Cluster` | 从车厢左/右边缘向中间形成的连续装载簇 |
| `Observation` | 当前 frame 中由一个 instance 得到的几何观测 |
| `Motion ROI` | 某历史 ID 当前允许出现的空间区域 |
| `Anchor Match` | 当前帧中证据足够强、可先锁定的历史 ID ↔ instance 匹配 |
| `OCCLUDED` | 历史 ID 仍存在，但当前帧没有可靠 instance 观测 |
| `UNCERTAIN` | 当前证据不足，系统主动拒绝做不可逆身份决定 |
| `NEW Hint` | 机器人提供的新盘条 D/L/目标位置等先验 |

---

## 5. 输入需求

### FR-IN-001 当前帧点云

必须支持 PLY，至少读取：

```text
x: float32
y: float32
z: float32
instance_id: int32
```

系统必须按 `instance_id` 聚合为实例点云。

### FR-IN-002 历史状态

除第一帧初始化外，每次处理必须输入上一次**已提交**的 tracker 状态，至少包含：

```text
global_id
layer
slot
cluster
stable_diameter
stable_length
last_confirmed_center
last_confirmed_yaw
visibility_state
left/right semantic neighbors
motion_roi
confidence
```

### FR-IN-003 新增盘条先验

接口必须支持 0..N 个：

```text
hint_id
nominal_diameter
nominal_length
target_center_xyz
position_tolerance_xyz   # 可选；未知时使用配置
expected_layer           # 可选
expected_yaw             # 可选
```

系统必须支持无 Hint 模式，但有 Hint 时应把它作为强证据，而不是直接当 Ground Truth。

### FR-IN-004 相机信息（预留）

后续可以输入相机内外参与本帧实际参与重建的相机集合，用于 `VISIBLE/OCCLUDED/ABNORMAL_MISSING` 精细判断。V0 可以不使用。

### FR-IN-005 标签隔离

推理代码严禁读取当前帧：

```text
global_id
RGB
layer GT
slot GT
visibility GT
```

测试代码必须把算法输入和 Ground Truth 分开加载。

---

## 6. 功能需求

### FR-GEO-001 实例几何观测

每个当前 instance 至少产生：

```text
center_xyz
axis_xyz / yaw
length_est
diameter_est
point_count
geometry_quality
visibility_quality
```

算法不得仅使用点云质心作为盘条中心；需要对部分可见和不规则表面具有鲁棒性。

### FR-GEO-002 历史几何反哺

对历史盘条，稳定 `D/L` 和 Motion ROI 必须可作为当前残缺观测的先验。低质量当前帧不得大幅改写长期稳定 D/L。

### FR-ROI-001 历史 Motion ROI

每个 Track 独立维护搜索范围。至少支持 X/Y/Z 范围；X 上限可配置到 2 m。允许根据：

- 与本次 NEW 目标位置距离；
- 历史运动；
- 连续 Cluster 中相邻盘条；
- 当前质量

动态收紧/放宽。

### FR-CAND-001 稀疏候选图

系统不得把每个 instance 与所有历史 ID 无条件全连接。候选至少由：

- layer 可行性；
- Motion ROI；
- D/L 兼容性；
- yaw/轴向兼容性；
- Slot/Cluster 顺序可行性

进行 gating。

### FR-MATCH-001 先锁定高置信可视组合

系统必须优先确定高质量、低歧义的可视对象作为 Anchor；低质量遮挡 observation 的优先级低于高置信可视对象。

### FR-MATCH-002 全局组合而非逐对象最近邻

最终匹配必须对整层/整 Cluster 的配置做一致性求解，禁止仅做“每个对象选最近历史 ID”。

### FR-MATCH-003 顺序保持硬约束

同层同 Cluster 内，历史 global ID 的 Slot 顺序不得反转。任何导致 `... ID2, ID1 ...` 的 hypothesis 必须拒绝。

### FR-MATCH-004 一对一约束

一个历史 ID 最多匹配一个当前 instance；一个当前 instance 最多继承一个历史 global ID。

### FR-MATCH-005 允许跳过历史 ID

求解器必须允许历史 ID 暂时无当前 instance，即 `SKIP_HISTORY`，由后续模块解释为 `OCCLUDED / UNCERTAIN / ABNORMAL_MISSING`。不能为了实现全覆盖而把别的 instance 强绑给它。

### FR-MATCH-006 允许插入当前 instance

求解器必须允许当前 instance 暂时没有历史 ID，即 `INSERT_CURRENT`，由后续模块解释为 `NEW / UNCERTAIN`。

### FR-NEW-001 NEW 判定

NEW 必须在旧 ID 的高可信组合求解之后最终确认。至少满足：

1. 无历史 ID 能在硬约束下合理解释该 instance；
2. 插入位置满足 Slot/Cluster 增长规则；
3. 有机器人 Hint 时，与 D/L/目标位置兼容；
4. 不会导致已有历史 ID 被迫错绑。

### FR-NEW-002 多 NEW

一次新增多个必须可表示和求解；V0 可针对 `new_count=1` 优化搜索，但不得把单 NEW 写成不可修改的算法假设。

### FR-OCC-001 遮挡后解释

只有在高置信可视组合基本确定后，才解释剩余历史 ID。若历史 Slot 位于两个确认邻居之间且当前无可靠 observation，可优先形成 OCCLUDED 假设。

### FR-OCC-002 历史 ID 不删除

`OCCLUDED` 时 Track 必须继续存在，且不得把其 global ID 释放给其他当前 instance。

### FR-OCC-003 无遮挡消失异常

当未来接入相机可见性模块后，如果历史 ROI 在有效视角中应清晰可见却完全无观测，状态应为 `ABNORMAL_MISSING` 或 `UNCERTAIN`，不能默认成 OCCLUDED。

### FR-STATE-001 保守提交

若最佳方案与次佳方案差距不足、关键证据冲突或存在硬约束风险，必须输出 `UNCERTAIN`，不能硬选 global ID。

### FR-STATE-002 低质量帧不得污染历史

`UNCERTAIN/OCCLUDED` 不得大幅更新稳定 D/L、Slot、Layer 等长期属性；位置可保留历史值/预测值并降低置信度。

### FR-GAP-001 相邻 gap

只对同一 Cluster 的直接语义邻居计算/输出 gap。禁止跨 `normal_middle_free_region` 计算异常 gap。

### FR-GAP-002 gap 置信度

gap 输出必须带质量/置信信息；接近误差尺度的 gap 不应强制二值判断。阈值必须配置化。

### FR-OUT-001 帧结果

每帧至少输出：

```text
frame_id
global_id
current_instance_id | null
state
center_xyz
displacement_xyz (若有可靠历史对应)
yaw
stable_diameter
stable_length
layer
slot
cluster
visibility_state
confidence
reason_codes
```

### FR-OUT-002 审计信息

每个已提交 MATCHED/NEW 决策必须可追溯：至少记录候选分数、硬约束检查结果、最佳/次佳 margin、使用的 Anchor/邻居信息。

---

## 7. 状态枚举

建议 V1 固定为：

```text
MATCHED_VISIBLE
MATCHED_PARTIAL
NEW
OCCLUDED
UNCERTAIN
ABNORMAL_MISSING
```

内部还可记录：

```text
TEMP_UNASSIGNED
```

用于当前有 instance 但暂未获得永久 global ID 的情况。

---

## 8. 非功能需求

### NFR-001 安全性优先

**`wrong_id_count = 0` 是首要验收门槛。** 覆盖率不能以牺牲 ID precision 为代价。

### NFR-002 性能

第一版目标：单帧约 10 万点、约 20 个盘条，从已得到 instance_id 的点云开始，**完整实例判断流程不超过 2 秒**。

测试必须记录运行硬件与阶段耗时，不能只给总耗时。

### NFR-003 可复现

同一输入、同一配置必须得到可重复结果；随机过程必须显式 seed。

### NFR-004 可配置

至少以下参数不得硬编码：

- D/L 合法范围；
- X/Y/Z Motion ROI；
- yaw 范围；
- geometry quality 阈值；
- Anchor 分数/margin；
- gap/contact/warning 阈值；
- NEW Hint 位置/尺寸容差；
- layer 高度范围/分界；
- UNCERTAIN 门槛。

### NFR-005 可诊断

任何 `UNCERTAIN / ABNORMAL_MISSING / WRONG_CONSTRAINT` 都必须输出 reason code 和候选信息，方便生产问题复盘。

---

## 9. 第一帧初始化

### FR-INIT-001

第一帧没有历史 global ID 时允许按稳定、可重复的语义规则初始化：

1. 推断 Layer；
2. 同层按 X/Slot 语义排序；
3. 从边缘 Cluster 建立 Slot；
4. 分配新的 global ID；
5. 保存稳定 D/L、center、yaw、邻接关系。

若第一帧本身存在无法可靠分层/排序的实例，可标 TEMP/UNCERTAIN，不得伪造稳定身份。

---

## 10. 数据集验收映射

| 需求 | V3.1 场景 |
|---|---|
| 滚动观测变化不依赖表面一致 | P01 / P02 |
| 左侧增长 + NEW | S01 |
| 右侧增长 + NEW | S02 |
| 双端 Cluster + 正常中间空区 | S03 |
| Cluster 内 gap | S04 |
| X 移动 + yaw | S05 |
| partial / occluded / 恢复 | S06 |
| NEW + OCCLUDED 同时出现 | S07 |
| 两层 + 不换层 | S08 |
| 一次多个 NEW | S09 |
| 相似几何防 ID swap | S10 |
| 第一层最多 10；满后进入第二层 | S11 |
| 跨视角观测变化 | S12 |
| 10+8 / >100k 性能 | S13 |
| 组合困难快照 | S14 |

---

## 11. V0/M1 验收门槛

### 硬门槛

1. 在 V3.1 所有 `temporal_adjacent` pair 上：`WRONG_ID = 0`。
2. S10 不允许 ID swap；如果无法判断必须 `UNCERTAIN`。
3. S03 不得把左右 Cluster 中间正常空区报告为异常邻居 gap。
4. S11 不得产生第 11 个第一层 Slot。
5. S06/S07 中 OCCLUDED global ID 不得被其他 instance 继承。
6. 推理不能读取当前帧 GT/RGB。
7. S13 约 100k+ 点性能测试满足 `< 2s`；必须记录硬件与详细阶段耗时。

### 优化指标（先测基线，再冻结数值）

- assigned coverage；
- UNCERTAIN rate；
- NEW precision/recall；
- OCCLUDED precision/recall；
- center error；
- gap error；
- geometry quality calibration。

**原因**：项目明确允许 UNCERTAIN，当前尚无生产数据基线，不应在实现前凭空固定覆盖率数字。

---

## 12. 尚待生产数据标定的参数

以下不是需求缺失，而是必须通过真实 Sequence 标定：

1. 机器人 target position 的实际误差范围；
2. 普通历史盘条典型 X/Y/Z 位移分布；
3. NEW 附近第 1/2/3 邻居的受挤位移分布；
4. 同一盘条不同视角下 center/D/L 的系统性偏差；
5. contact / warning / abnormal gap 阈值；
6. layer Z 分布及第二层支撑关系阈值；
7. 当前几何 quality 与真实 center error 的映射；
8. 相机可见性接入后 OCCLUDED 与 ABNORMAL_MISSING 的判据。

这些项目统一记录在 `DECISIONS_AND_TBDS.md`，不得由开发者静默猜测并写死。

## 14. Web 连续目检器需求（2026-08-18 冻结）

### FR-UI-001 连续 Session
Web 必须支持“选择 Sequence 初始帧 → 初始化 TrackManager → 连续下一帧 → Sequence 结束 → 重置并重新选择初始帧”的真实 tracking session。下一帧必须使用算法上一帧自行提交的 Track State，不得用 GT 重新 bootstrap。

### FR-UI-002 首帧几何可视化
第一帧必须独立检测并展示每个盘条的 `global_id / center / axis(yaw) / radius(diameter) / length / layer / slot / quality`。

### FR-UI-003 身份颜色一致性
每个 `global_id` 在 session 内必须绑定固定颜色；相邻帧同 ID 同颜色。`NEW` 首帧用特殊强调状态，下一帧转回该 ID 固定颜色；`OCCLUDED` 保留原 ID 色并用半透明/虚线状态表达；未安全继承 ID 的 `UNCERTAIN` 当前实例不得冒用历史 ID 颜色。

### FR-UI-004 Gap 展示
只显示同一连续 Cluster 内直接语义邻居的间距。左右 Cluster 的正常中间空区不得被画成邻居 gap 或异常 gap。

### FR-UI-005 GT 隔离
`显示 Ground Truth` 默认关闭；开启后只允许评测层叠加显示，不能改变 matcher 输入、候选或结果。

### FR-UI-006 结束行为
最后一帧后再次点击 `下一帧` 必须弹出序列完成提示，不循环、不跳其他 Sequence；用户可选择新的初始帧，操作会清空当前 TrackManager。

完整交互、颜色和验收规则见 `docs/WEB_PRODUCT_REQUIREMENTS_V1.md`。

---

## V0.4 追加冻结需求：交互式场景创建与失败案例

1. Web 在当前 Sequence 没有下一帧时必须支持“创建下一帧”。
2. 用户必须能够为每个已有 global ID 设置 `ΔX/ΔY/ΔZ/ΔYaw`。
3. 用户必须能够为已有盘条设置滚动观测扰动，但不得通过大块中间挖洞模拟滚动。
4. 用户必须能够新增一个或多个盘条，并设置 D/L/XYZ/Yaw/Layer 和 robot target 偏差。
5. 遮挡编辑只允许 left/right/both edge partial occlusion，必须生成非零点。
6. **OCCLUDED 状态必须有当前 instance 且 point_count > 0。**
7. 历史 ID 当前完全零点时必须使用 `UNOBSERVED`（或上层异常），不得输出 OCCLUDED。
8. 用户创建的 PLY 与原有 PLY 一样平铺在工程根目录，方便直接目检。
9. 用户创建帧后必须立即使用上一帧算法 TrackState 运行 matcher，禁止重新从 GT bootstrap。
10. Web 必须允许“一键保存问题案例”，保存前后 PLY、用户编辑动作、GT、robot hint、随机 seed、tracker state、完整算法 trace、配置和用户备注。
11. 保存的问题案例应能作为后续算法修改的永久 regression 输入。

---

## V0.6 Semantic Guard 覆盖说明

以下规则覆盖本文早期关于正常生产模式 `SKIP_HISTORY` / 零点遮挡的旧描述：

1. 正常业务中历史盘条不会移除，并且即使遮挡也必须保留部分 current instance 点云。
2. `OCCLUDED` 必须有非空 current instance；零点不能解释为正常 OCCLUDED。
3. 已知 robot NEW 数量时，正常层内要求 `current_count = history_count + expected_new_count`。不满足时输出 `ABNORMAL_CARDINALITY`，不得用零点隐藏历史对象补齐。
4. 同层同语义 Cluster 的历史顺序不可反转；选出合法 NEW 后，历史对象与当前旧实例使用保序一一对应。
5. Anchor / 最近位置 / 单体几何不得绕过上述语义硬约束提交 Global ID。
6. 若语义唯一但几何安全验证失败，允许输出 `UNCERTAIN`，保留当前点云证据但不更新可靠 TrackState。
7. 安全目标：`UNCERTAIN / ABNORMAL > wrong Global ID`。

## V0.7 追加要求：运动异常与 TOP_DOWN_Z

1. 身份置信度与运动异常必须分离。X 运动至少支持 normal envelope 与 hard envelope；normal 越界但 hard 未越界时，若语义身份唯一，可保留 Global ID 并显式输出 `ABNORMAL_X_MOTION`，而不是仅因轻微越界将身份清零。
2. hard envelope 仍是身份安全边界；越过 hard 且无更高层业务授权时必须 `UNCERTAIN/ABNORMAL`，不得造成后续 ID 顺移。
3. Web 场景编辑器必须读取同一 tracker config 显示运动包络预警；预警不应阻止用户构造压力测试。
4. 系统必须支持 `TOP_DOWN_Z`：相机位于场景上方，视线沿世界 `-Z`，点云主要只包含顶部可见表面。
5. TOP_DOWN_Z synthetic data 必须使用全场投影可见性（z-buffer 或等价方法），禁止用随机砍点或简单 `z > center` 代替。
6. `VIEW_PARTIAL`（视角天然不完整）与 `OCCLUDED`（被其它物体额外遮挡）必须区分。
7. 当前业务假设下，历史盘条即使遮挡也必须有正 current instance；zero-point 不能解释为正常 OCCLUDED。
8. TOP_DOWN_Z 下必须优先利用 stable D/L、layer/slot/cluster/order/cardinality 和 robot NEW hint；不得要求当前帧重新完整拟合圆柱。



## V0.8 追加要求：双层支撑语义
- 第二层信号为强业务先验；正常第二层必须建立在第一层 10/10 满载之上。
- 第二层激活后第一层为锁定 foundation；显著运动输出结构异常而非普通位移。
- 第二层 slot k 由 L1[k]/L1[k+1] 支撑对唯一定义，support topology 优先于几何最近邻。
- 双层 bootstrap 必须支持严重遮挡，但任何确定 track 均要求正点云证据；零点不作为 OCCLUDED。
