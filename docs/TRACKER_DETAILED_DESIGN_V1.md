# 盘条语义状态跟踪器 V1 —— 详细设计说明书（SDD）

**版本**：1.0  
**对应需求**：`TRACKER_REQUIREMENTS_V1.md`  
**实现目标**：先在 V3.1 合成集建立可解释、保守、零错 ID 的 V0，再接入真实连续生产数据标定。

---

## 1. 总体架构

```text
Current PLY (XYZ + instance_id)
              │
              ▼
      [1] Instance Aggregator
              │
              ▼
      [2] Geometry Observer
  center / axis / D / L / quality
              │
              ├──────────────┐
              │              │
Historical Track State     NEW Hints
              │              │
              ▼              ▼
      [3] Motion ROI / Layer / Slot Prior
              │
              ▼
      [4] Sparse Candidate Graph
              │
              ▼
      [5] High-confidence Anchors
              │
              ▼
      [6] Ordered Semantic Solver
   MATCH / SKIP_HISTORY / INSERT_CURRENT
              │
              ▼
      [7] NEW + OCCLUSION Interpreter
              │
              ▼
      [8] Global Consistency Validator
              │
        ┌─────┴─────┐
        ▼           ▼
     COMMIT       UNCERTAIN
        │
        ▼
      [9] Track Manager Update
        │
        ▼
 gap / output / timing / audit log
```

设计核心：**当前几何只产生 observation；global ID 的决定由历史状态和整车语义一致性共同完成。**

---

## 2. 模块划分

建议 Python V0 目录：

```text
tracker/
├── models.py
├── config.py
├── geometry.py
├── layer.py
├── roi.py
├── candidates.py
├── anchors.py
├── ordered_solver.py
├── new_inference.py
├── occlusion.py
├── validator.py
├── track_manager.py
├── gap.py
├── confidence.py
├── io.py
└── pipeline.py
```

测试：

```text
tests/
├── test_geometry.py
├── test_slot_rules.py
├── test_ordered_solver.py
├── test_new.py
├── test_occlusion.py
├── test_gap.py
├── test_no_gt_leak.py
└── test_sequences.py
```

---

## 3. 核心数据结构

### 3.1 FrameInput

```python
@dataclass
class FrameInput:
    frame_id: str
    timestamp: float | None
    xyz: np.ndarray              # [N,3], float32
    instance_id: np.ndarray      # [N], int32
    new_hints: list[NewCoilHint]
    camera_state: CameraState | None = None
```

禁止把 PLY 中 `global_id/RGB/layer/slot/visibility` 放入此结构。

### 3.2 InstanceObservation

```python
@dataclass
class InstanceObservation:
    instance_id: int
    points: np.ndarray
    point_count: int

    center: np.ndarray
    axis: np.ndarray
    yaw_deg: float
    diameter_est: float
    length_est: float

    center_sigma: np.ndarray
    diameter_sigma: float
    length_sigma: float

    geometry_quality: float      # [0,1]
    partial_score: float         # [0,1]
    layer_hypotheses: list[tuple[int,float]]
```

`*_sigma` 初期允许由启发式质量映射给出；真实数据到位后标定。

### 3.3 TrackState

```python
@dataclass
class TrackState:
    global_id: int
    layer: int
    slot: int
    cluster: str                 # left/right/layer2/...

    stable_diameter: float
    stable_length: float
    stable_size_confidence: float

    center: np.ndarray
    yaw_deg: float
    center_confidence: float

    visibility: str
    last_instance_id: int | None
    last_confirmed_frame: str

    left_neighbor_id: int | None
    right_neighbor_id: int | None

    motion_roi: MotionROI
    age: int
    uncertain_count: int
    occluded_count: int
```

### 3.4 LayerState

第一层显式保存：

```python
@dataclass
class LayerState:
    layer: int
    max_slots: int | None
    occupied: dict[int, int]         # slot -> global_id
    left_cluster_slots: list[int]
    right_cluster_slots: list[int]
    normal_middle_free_slots: list[int]
```

第一层 `max_slots=10`。

### 3.5 NewCoilHint

```python
@dataclass
class NewCoilHint:
    hint_id: str
    nominal_diameter: float
    nominal_length: float
    target_center: np.ndarray
    tolerance_xyz: np.ndarray | None
    expected_layer: int | None
    expected_yaw_deg: float | None
```

### 3.6 MatchCandidate

```python
@dataclass
class MatchCandidate:
    global_id: int
    instance_id: int
    hard_valid: bool
    total_cost: float
    component_costs: dict[str, float]
    reason_codes: list[str]
```

### 3.7 FrameDecision

```python
@dataclass
class FrameDecision:
    global_id: int | None
    instance_id: int | None
    state: str
    confidence: float
    center: np.ndarray | None
    reason_codes: list[str]
```

---

## 4. Step 1：实例聚合

复杂度必须为 O(N)。

```python
instances = {}
for p, iid in zip(xyz, instance_id):
    instances[iid].append(p)
```

实际实现避免 Python 逐点 list append，可采用：

1. `np.argsort(instance_id)`；
2. 按连续段切分；或
3. `np.unique(return_inverse=True)` 后索引。

N≈100k，不应成为 2 秒目标的主要瓶颈。

---

## 5. Step 2：几何观测器

### 5.1 设计原则

不能假设完整圆柱，也不能依赖端面。当前观测目标不是获得毫米级真实圆柱，而是得到足够稳定的：

```text
center / axis / L / D / quality
```

供语义 matcher 使用。

### 5.2 轴方向估计

V0：对实例点云做鲁棒 PCA。

1. 先使用分位数裁除极端点（例如每轴 0.5%–99.5%，参数化）；
2. 计算 covariance；
3. 最大特征向量作为候选轴；
4. 利用“轴大体位于 XY 平面、通常接近 Y”做方向消歧；
5. 若存在历史 Track 候选，用历史 axis/yaw 选择符号与合理方向。

约束：

\[
|a_z| \le a_{z,max}
\]

超过阈值时降低 `geometry_quality`，不直接强行修正成水平。

### 5.3 长度估计

将点投影到轴：

\[
s_k = (p_k-c_0)\cdot a
\]

使用鲁棒分位数：

\[
L_{obs}=Q_{high}(s)-Q_{low}(s)
\]

不使用 min/max，避免重建毛刺把长度拉长。

### 5.4 截面与直径

自由拟合完整圆对部分上表面会不稳定，因此分两条路径。

#### A. NEW / 高质量 observation

- 把点投影到轴垂直平面；
- 使用 robust circle/cylinder residual；
- D 受业务合法范围约束；
- 输出拟合 residual 和 coverage。

#### B. 历史 Track / partial observation

优先使用 `stable_diameter` 作为先验：

\[
D \sim \mathcal{N}(D_{stable},\sigma_D^2)
\]

主要优化当前截面中心，而不是允许当前残缺点云随意改变 D。

### 5.5 中心估计

不能把可见点质心直接当盘条中心。

建议目标函数：

\[
E(C,a,D,L)=
E_{surface}
+\lambda_pE_{history\_position}
+\lambda_D E_{stable\_D}
+\lambda_L E_{stable\_L}
+\lambda_a E_{axis}
\]

对 NEW，历史项置零，使用 robot hint / 当前高质量观测替代。

V0 为降低实现复杂度，可先使用：

- PCA axis；
- 轴向分位数中点作为轴向中心；
- 横截面固定半径/robust circle 的二维中心；
- 历史 ROI 对横截面解做 gate。

后续再换成统一非线性优化器，接口不变。

### 5.6 Geometry Quality

建议由以下量组成：

```text
point_count score
axis eigengap score
length coverage score
cross-section residual
cross-section angular coverage
agreement with stable D/L
```

输出 [0,1]，但初始映射只作为工程 heuristic；真实数据需校准。

---

## 6. Step 3：Layer 推断与历史约束

### 6.1 历史对象

Layer 是长期硬属性：

```text
track.layer 不允许改变
```

当前 observation 只需要判断“是否可能来自该 layer”。

### 6.2 NEW

NEW layer 依次参考：

1. robot expected_layer（若有）；
2. 当前 center_z；
3. 第一层是否已满 10；
4. 支撑拓扑（后续增强）。

S11 中第一层已有 10 个时，新对象不得进入 layer 1。

---

## 7. Step 4：Motion ROI

### 7.1 基础 ROI

对历史 i：

```text
x ∈ [x_prev - rx_left, x_prev + rx_right]
y ∈ [y_prev - ry, y_prev + ry]
z ∈ [z_prev - rz, z_prev + rz]
```

X 的绝对最大上限配置不超过当前业务暂定的 2 m；实际应比 2 m 更小，并按对象动态化。

### 7.2 NEW 影响下的自适应 ROI

若机器人目标位置 `P_new` 已知：

```text
distance(track_i, P_new) 小 -> ROI 可以稍大
distance(track_i, P_new) 大 -> ROI 收紧
```

不要在 V0 写死“第一个邻居 50cm”等具体生产数字；这些必须由真实数据标定。

### 7.3 ROI 反哺几何

严重 partial 时，center fit 的搜索域限制在该 Track 的 ROI 内。这样历史约束参与“当前圆心在哪”，而不只是事后匹配。

---

## 8. Step 5：候选图构建

每个 `(track_i, obs_j)` 先做 Hard Gate：

```text
G_layer
G_motion_roi
G_size_coarse
G_axis_coarse
G_order_possible
```

任一明确失败：不建立候选边。

### 8.1 Match Cost

通过 gate 后：

\[
C_{ij}=
 w_p C_{pos}
+w_D C_D
+w_L C_L
+w_y C_{yaw}
+w_q C_{quality}
+w_n C_{neighbor}
\]

建议归一化：

\[
C_{pos}=\left(\frac{\Delta x}{r_x}\right)^2+
\left(\frac{\Delta y}{r_y}\right)^2+
\left(\frac{\Delta z}{r_z}\right)^2
\]

D/L cost 应根据 observation quality 调权：partial 越重，越信历史稳定 D/L，越少用当前 D/L 差异去否定 ID。

### 8.2 不使用的核心证据

V1 主 matcher 不把以下作为身份主特征：

```text
raw ICP score
raw Chamfer distance
surface local descriptor similarity
RGB
```

它们可在未来作为低置信二次证据，但不得推翻硬语义约束。

---

## 9. Step 6：高置信 Anchor

目标：先锁住明显的可视 ID，避免遮挡 observation 干扰。

对于 observation j：

1. 候选数量为 1，或 best cost 明显优于 second；
2. geometry_quality ≥ `anchor_quality_min`；
3. 通过 layer / ROI / order 硬约束；
4. 与已锁 Anchor 不冲突；

则形成 Anchor。

Margin：

\[
margin=C_{second}-C_{best}
\]

Anchor 只有在 cost 和 margin 同时满足时才锁定。

锁定 Anchor 后，将该 track 和 instance 从剩余竞争中移除，并把 Anchor 作为左右区间边界。

---

## 10. Step 7：有序语义求解器

这是 V1 matcher 核心。

### 10.1 为什么不用纯 Hungarian

Hungarian 只保证一对一，不知道：

- 同 Cluster 顺序不能反转；
- 历史 ID 可以被遮挡；
- 当前 instance 可以是 NEW；
- 第一层有两个边缘 Cluster 和正常中间空区。

因此 V1 使用 **Anchor + Ordered Sequence Alignment / Dynamic Programming**。

### 10.2 单 Cluster DP

历史 track 按 Slot 排序：

\[
H=(h_1,\ldots,h_m)
\]

当前 observation 按该 Cluster 的 X 语义方向排序：

\[
O=(o_1,\ldots,o_n)
\]

状态：

\[
DP[i,j]
\]

表示解释前 i 个历史对象与前 j 个当前 observation 的最小代价。

转移：

#### MATCH

\[
DP[i,j]\leftarrow DP[i-1,j-1]+C_{match}(h_i,o_j)
\]

仅当候选边存在。

#### SKIP_HISTORY

\[
DP[i,j]\leftarrow DP[i-1,j]+C_{missing}(h_i)
\]

代表历史 ID 暂时没有当前 observation；后续再判 OCCLUDED/UNCERTAIN。

#### INSERT_CURRENT

\[
DP[i,j]\leftarrow DP[i,j-1]+C_{insert}(o_j)
\]

代表当前 instance 不能由旧 ID 解释；后续再判 NEW/UNCERTAIN。

由于 DP 只沿有序序列前进，天然禁止 ID 顺序反转。

### 10.3 双端 Cluster

第一层分别求：

```text
left cluster
right cluster
```

左右 Cluster 中间不建立邻接/gap 约束。

新增只能合法扩展：

```text
left cluster 的内侧边界
right cluster 的内侧边界
```

或在第一层满后进入 layer 2。除非未来业务明确允许其他插入模式，否则内部 Slot 中间突然插入 NEW 应被高成本/拒绝。

### 10.4 Anchor 分区

已确定 Anchor 可把长序列拆成多个独立区间：

```text
Anchor ID1 | unresolved ID2 ID3 | Anchor ID4
```

仅在中间区间做 DP，减少组合数并加强语义解释。

---

## 11. Step 8：NEW 推断

DP 的 `INSERT_CURRENT` 不直接等于 NEW。

### 11.1 NEW Score

\[
C_{new}=w_tC_{target}+w_D C_D+w_L C_L+w_sC_{slot}+w_lC_{layer}
\]

有 robot hint 时，目标位置/D/L 是强证据。

### 11.2 确认条件

只有：

- 旧 ID 组合已合理解释；
- 当前 instance 没有安全历史候选；
- Slot/Cluster 增长合法；
- Hint（若存在）兼容；

才能正式分配新的 global ID。

否则：

```text
TEMP_UNASSIGNED / UNCERTAIN
```

不允许为了“新增数量对上”而牺牲旧 ID。

---

## 12. Step 9：OCCLUDED 推断

DP 的 `SKIP_HISTORY` 先表示“历史 ID 未观察到”，再解释原因。

### 12.1 V0 语义判定

若满足：

- track 在上一帧存在；
- 没有旧盘条移除业务；
- 当前无可靠匹配；
- 前后 Slot/Anchor 语义仍能容纳它；

可输出：

```text
OCCLUDED 或 UNCERTAIN
```

最保守规则：**没有足够遮挡证据时宁可 UNCERTAIN。**

### 12.2 Camera-aware V1.5

有相机姿态后，对 Track ROI 做可见性评估：

```text
被所有有效视角遮挡 -> OCCLUDED confidence ↑
理论上清晰可见但无点 -> ABNORMAL_MISSING
视角证据冲突 -> UNCERTAIN
```

当前 V3.1 的遮挡标签只用于评测，算法不得读取。

---

## 13. Step 10：Global Consistency Validator

求解结果提交前逐条检查硬约束：

```text
V1: one current instance -> max one global ID
V2: one global ID -> max one current instance
V3: track layer unchanged
V4: layer1 occupied slots <= 10
V5: same cluster order monotonic
V6: no current instance assigned to an OCCLUDED historical ID simultaneously
V7: NEW slot legal
V8: normal middle free region not treated as direct-neighbor gap
V9: movement within accepted ROI unless state UNCERTAIN
```

任一失败：不自动“修正成最近 ID”；整个冲突子区间回退为 `UNCERTAIN`。

---

## 14. Confidence 与拒绝策略

### 14.1 不把 cost 直接当 confidence

至少组合：

```text
absolute fit quality
best-vs-second margin
number of independent evidence types
geometry quality
hard constraint safety
neighbor/topology agreement
```

### 14.2 建议三级

```text
COMMIT_HIGH      -> 继承/创建 global ID
COMMIT_MEDIUM    -> 可输出但不更新稳定几何或只弱更新
UNCERTAIN        -> 不做不可逆身份提交
```

V0 可以先只实现 HIGH + UNCERTAIN，保证正确性，再提高 coverage。

---

## 15. Track Manager 更新规则

### MATCHED_VISIBLE

- 更新 center/yaw；
- `stable_D/L` 只按 geometry quality 小步更新；
- 更新 last_instance_id；
- visibility=VISIBLE；
- 清零 occluded_count。

### MATCHED_PARTIAL

- center 允许更新但降低权重；
- D/L 基本沿用历史稳定值；
- 不因为残缺 bbox 让尺寸突然缩小。

### OCCLUDED

- global ID 保留；
- instance_id=None；
- center 保存 last confirmed / constrained prediction；
- 不更新 D/L/slot/layer；
- 置信度随连续遮挡帧下降。

### UNCERTAIN

- 不给可疑 current instance 强行继承历史 ID；
- 历史 Track 不被删除；
- 保存候选/日志供下一帧继续消歧。

### NEW

- 分配新 global ID；
- 记录 robot nominal D/L 为高价值稳定属性；
- 当前观测用于 center/yaw；
- 根据合法增长规则建立 slot/cluster。

---

## 16. Gap 计算

### 16.1 语义邻居选择

只使用同一 Cluster 直接 Slot 邻居。

### 16.2 V0 模型 gap

考虑 yaw 后，不建议只使用 X 差。对相邻两个盘条平均轴：

\[
a=normalize(a_i+a_j)
\]

XY 平面的法向：

\[
n=(-a_y,a_x)
\]

中心法向间距：

\[
s=|(C_j-C_i)\cdot n|
\]

模型 gap：

\[
g=s-\frac{D_i+D_j}{2}
\]

输出：

```text
gap_est
gap_sigma
CONTACT / GAP / UNCERTAIN
```

当 `|gap|` 与 center/D 误差同量级时，必须 UNCERTAIN。

### 16.3 近阈值二次验证（后续）

只对 gap 接近业务阈值的邻居，取互相面对的局部点云做最近表面验证，不对所有 pair 做 ICP。

---

## 17. 第一帧初始化设计

1. 按实例聚合并做几何；
2. 根据 Z/支撑粗分 Layer；
3. Layer 1 沿 X 排序；
4. 根据靠近车厢左右边界的连续占用识别 left/right Cluster；
5. 映射到 Slot 0..9；
6. 分配 global ID；
7. 记录稳定 D/L 和邻居；
8. 无法安全排序的对象保持 TEMP/UNCERTAIN。

在真实系统中，如果业务系统能提供初始装载语义，可直接覆盖自动初始化。

---

## 18. 输出文件建议

### `frame_result.json`

```json
{
  "frame_id": "...",
  "runtime_ms": {
    "load": 0,
    "aggregate": 0,
    "geometry": 0,
    "candidate": 0,
    "solver": 0,
    "postprocess": 0,
    "total": 0
  },
  "objects": [
    {
      "global_id": 7,
      "instance_id": 320,
      "state": "MATCHED_VISIBLE",
      "confidence": 0.97,
      "center": [1.0, 2.0, 0.6],
      "yaw_deg": 2.1,
      "diameter": 1.28,
      "length": 1.20,
      "layer": 1,
      "slot": 3,
      "cluster": "left",
      "reason_codes": ["ROI_OK", "ORDER_OK", "ANCHOR_MARGIN_OK"]
    }
  ]
}
```

### `match_audit.csv`

一行一个候选 `(pair, global_id, instance_id)`，记录各分量 cost、gate 结果和最终状态，方便调参。

---

## 19. 复杂度与性能

N 为点数，M 为盘条实例数（约 <=20）：

```text
聚合/几何：O(N)
候选：O(M^2)，但经 gate 后稀疏
单 Cluster DP：O(m*n)
全局校验：O(M^2) 以内
```

因此理论上主要耗时仍是点云几何，不是语义求解。

V0 每次运行必须打印：

```text
N points
M instances
aggregate ms
geometry ms
candidate ms
anchor ms
solver ms
postprocess ms
total ms
```

---

## 20. 失败保护

以下情况禁止强制输出 MATCHED：

1. best/second ID 证据过近；
2. 需要违反 Slot 顺序才能匹配；
3. 需要换层才能匹配；
4. 需要超过可信 Motion ROI 很多；
5. 一个 instance 同时强支持两个历史 ID；
6. 一个历史 ID 被两个 observation 同时解释；
7. NEW 与旧 ID 解释无法消歧；
8. 当前几何质量太差且语义也无 Anchor 支撑。

统一降级到 `UNCERTAIN`，并保持历史 ID 状态不被污染。

---

## 21. V0 实现顺序

1. Loader + GT leak 防护；
2. InstanceObservation 基础几何；
3. TrackState / LayerState；
4. Motion ROI；
5. Candidate Graph；
6. Anchor；
7. 单层单 Cluster Ordered DP；
8. 第一层双 Cluster；
9. NEW；
10. OCCLUDED；
11. Layer 2；
12. Gap；
13. Confidence/UNCERTAIN；
14. Evaluator + audit log；
15. S13 性能测量。

不要一开始实现 camera visibility、深度 ReID 或完整圆柱优化。

## 18. Web 连续目检器详细设计补充

### 18.1 IdentityColorRegistry

Web session 维护：

```python
IdentityColorRegistry: dict[int, RGB]
```

颜色由 `global_id` deterministic 生成并缓存；不得由 current instance_id 或点云顺序生成。状态通过 alpha / outline / icon / label 叠加，不重载 identity color。

### 18.2 StatefulInspectionSession

```python
class StatefulInspectionSession:
    sequence_id: str
    frame_index: int
    track_manager: TrackManager
    color_registry: IdentityColorRegistry
    audit_history: list[FrameAudit]
```

`next_frame()` 必须调用当前 `track_manager`，处理结果 commit 后成为下一次历史输入。禁止在每个 pair 前重新读取 previous GT state。

### 18.3 First-frame initialization
首帧只运行 instance geometry + layer/slot/cluster initialization + deterministic global ID assignment。合成数据 GT ID 只归 evaluator，不能传给 initialization。

### 18.4 Render Model
Web API 不应直接把 matcher 内部对象暴露给页面。建议统一返回：

```text
FrameViewModel
  frame_info
  coils[]
    global_id / instance_id / state
    identity_color
    center / axis / radius / diameter / length
    layer / slot / cluster
    displacement_from_previous
    quality
    render_style (alpha/outline/icon)
  occluded_proxies[]
  neighbor_gaps[]
  semantic_layout
  selected_object_explanation
  advanced_debug (lazy)
```

### 18.5 End-of-sequence
服务端应返回 `sequence_complete=true`。前端在最后一帧后再次请求 next 时弹 modal，不能修改 frame index。reset/new-sequence endpoint 必须销毁旧 session state。

完整产品规则见 `WEB_PRODUCT_REQUIREMENTS_V1.md`。


## V0.3 semantic rank-lock safety rule

The complex C08 sequence exposed a failure mode in pure position-biased ordered DP: when every historical coil translates together by about one coil spacing, the lowest local cost may be `SKIP_HISTORY(ID0) + ID1→current0 + ... + INSERT_CURRENT`, producing a whole-chain ID shift without violating monotonic order.

A stronger business invariant is therefore applied **only under a conservative precondition**:

1. all committed history tracks in the layer were visible;
2. current visible count equals the history count;
3. there is no robot NEW hint for the step;
4. every rank-by-X history/current pair passes normal Layer/ROI/D/L gates;
5. rank-pair X displacements are collectively coherent.

Under these conditions old-object removal is unsupported and same-layer order cannot reverse, therefore rank-by-X is identity. These rank pairs become semantic anchors. If any precondition fails, the matcher falls back to normal candidate/anchor/ordered-DP logic and may output UNCERTAIN.

---

## V0.4 Interactive Scene Editor 设计补充

交互式生成器位于 `tracker/editor.py`，与 matcher 严格隔离。

- Editor 可以访问测试 GT/prototype，因为其职责是“创建测试场景”。
- Matcher 仍只能通过 `load_algorithm_input()` 获取 XYZ + instance_id。
- 每个创建步骤产生新的 local instance_id。
- 已有 Track 通过真实 prototype + stable D/L 重新生成当前 observation，允许遮挡后再次恢复完整观测。
- roll observation change 使用局部表面扰动/稀疏/重采样，不绕已有部分点云做自身轴 rigid rotation。
- edge occlusion 在盘条 local-X 左/右端生成，中央区域保护，强制最小可见点数。
- user NEW 生成真实当前点云；Robot Hint 只向 matcher 暴露 nominal D/L + approximate target。
- 允许生成违反 order/capacity 等业务规律的 adversarial 场景，但必须记录 warning。

### Visibility runtime 语义

`relation` 与 `visibility` 分离：

```text
relation: INITIAL / MATCHED / NEW / UNCERTAIN / UNOBSERVED
visibility: VISIBLE / PARTIAL_VISIBLE / OCCLUDED / UNOBSERVED
```

其中：

- MATCHED + VISIBLE：正常可视；
- MATCHED + PARTIAL_VISIBLE：有部分点；
- MATCHED + OCCLUDED：仍有当前 instance，只是严重部分可见；
- UNOBSERVED：当前没有任何点，历史 Track 仅被保留，不能称为遮挡。

---

## V0.6 Semantic Guard 覆盖说明

V0.6 生产 matcher 使用 `semantic_constrained_match()` 替代早期以自由 `MATCH/SKIP_HISTORY/INSERT_CURRENT` DP 为核心的身份提交路径。旧 DP 与 Anchor 代码仍保留用于历史回归/诊断，但不得在正常生产路径绕过语义硬约束提交 Global ID。

生产层内求解：

1. 历史 Track 按 semantic slot 排序；当前 observation 按 X 空间顺序排序。
2. robot hints 给出 expected NEW count，并枚举满足 hint + 合法 inward slot growth 的 NEW observation 组合。
3. 去掉 NEW 后，历史与当前旧 observation 必须数量相等，并按顺序一一对应；正常模式无 `SKIP_HISTORY`。
4. 对对应 pair 再检查 ROI/D/L/yaw/部分观测质量；超安全 envelope 的 pair 降级 `UNCERTAIN`，不允许后续 pair 左移/右移补位。
5. `OCCLUDED/PARTIAL_VISIBLE` 只来源于已经匹配到的非空 current instance 的可见点比例；零点不叫 OCCLUDED。
6. 任何帧级异常都应显式暴露 `frame_status` 与 reason codes，并保护上一份可靠 TrackState。

## V0.7 设计补充：双运动包络与顶部观测

### X normal/hard envelope

`x_absolute_max_m` 作为 normal envelope；新增 `x_hard_max_m`。Pair assessment 对 `normal < |dx| <= hard` 返回“身份兼容 + 运动异常”而不是直接拒绝。只有 semantic order/cardinality 唯一、无合法竞争身份且其它几何 gate 通过时才允许语义救援；输出原因中记录越界量与百分比。`|dx| > hard` 保持硬拒绝身份继承。

### TOP_DOWN_Z observation

`tracker/view_model.py` 对 synthetic/editor 点云做全场 XY z-buffer。`tracker/geometry.py` 仅从 XYZ 识别 top-only 形态，并使用 `top_z - D/2` 修正 Z center。`tracker/matching.py` 在 candidate gating 前调用 TOP_DOWN semantic layer assignment：当 cardinality 一致时，依据历史 layer + NEW hint 的 expected counts 与 current top_z 排序恢复 layer；然后进入同一 ordered semantic matcher。

TOP_DOWN_Z 只改变 observation model，不改变 Global ID、layer、slot、cluster 等持久语义。



## V0.8 设计补充：Foundation Lock + Support-Slot Matcher
第二层阶段先验证 10 个 L1 foundation，再冻结其 committed TrackState。L2 observation 映射到离散 support valley，历史 L2 track 的 slot 即 support pair index。支撑槽 hard violation、L1 foundation hard movement、L2 capacity violation直接进入 ABNORMAL/UNCERTAIN，不允许通过后续 ID 补位修复。首帧双层使用 DOUBLE_LAYER_BOOTSTRAP：Z 分层 → 10-slot L1 lattice → L2 support assignment → 正点云证据验证。
