# 盘条语义跟踪数据集 V3.1 —— 独立接手说明（首读）

> 目标：即使没有原聊天记录，只依靠本目录，也能理解场景、读取数据、复现数据、开展 ID/遮挡/新增算法测试。

## 1. 当前问题定义

这不是点云 ReID 或逐点配准问题，而是 **“历史状态 + 装载语义 + 运动限制 + 当前实例观测 → 当前最可信 global ID 布局”**。

业务优先级：**错误 ID = 不允许；UNCERTAIN = 允许。** 算法宁可暂时不继承 ID，也不能把历史 global ID 错绑给另一个真实盘条。

每个 PLY 是一个完整场景帧，内部含多个盘条。单帧 `instance_id` 是可用输入，但跨帧无身份意义；`global_id` 是 Ground Truth，算法推理阶段严禁读取。

## 2. 场景硬约束

- 场景坐标：X 为主要滚动/语义排序方向，Y 为盘条轴向，Z 为高度。
- 第一层最多 10 个语义 Slot；不会出现第 11 个第一层盘条。
- 第一层可以左侧向中间长、右侧向中间长、或两侧轮流向中间长。
- 两个边缘 Cluster 之间的大空区可以完全正常，不能当作异常 gap。
- 同一连续 Cluster 内部的 Slot/ID 顺序不能交换。
- 盘条不会换层；历史盘条正常不被移走。
- 一次通常新增 1 个，但系统应兼容多个 NEW。
- 盘条主要沿 X 移动，可有少量 Y/Z 变化和显式 yaw。
- 点云来自多相机三维重建；可见表面会随视角、遮挡、滚动而变化。

## 3. 生成逻辑最重要的两条

### 滚动观测变化

没有把已有部分点云绕自身圆柱轴做刚体旋转。因为原观测不是 360° 完整表面，刚体旋转会把“原本可见的物理点”错误转到不可见区域。

当前模拟只做：厘米级径向表面扰动、逐点重建噪声、随机稀疏、局部重采样。**普通滚动不会制造一个大块连续缺口。**

### 遮挡

邻居遮挡只削弱盘条外侧 LEFT / RIGHT / BOTH 边缘；中央带保留。不会随机从正中间挖洞。当前业务规则禁止“零点正常遮挡”：`OCCLUDED/PARTIAL` 必须仍有当前实例点云。若历史盘条当前完全无点，只能进入 `UNOBSERVED/ABNORMAL/UNCERTAIN` 诊断，不能用它解释正常遮挡。

当前遮挡仍是“语义侧边模型”，不是基于真实相机光线的完整 ray tracing。后续有相机姿态和真实场景模型后可升级。

## 4. 目录约定

- 根目录下的 `*.ply`：**全部生成点云，且所有 PLY 始终在同一层目录。**
- `labels/`：算法 Ground Truth 和测试 pair。
- `code/`：读取、复现、校验、评测示例。
- `assets/`：真实原型、场景生成规格。用于断联后的自包含复现。
- `docs/`：详细文档。

## 5. 最安全的算法使用方式

```bash
python code/example_usage.py
```

代码默认只暴露：

```text
x, y, z, instance_id
```

**禁止把以下字段当算法输入：**

```text
global_id, layer, slot, visibility, RGB
```

这些字段嵌在 PLY 内只是为了目检和 GT 校验，其中 RGB 直接按 global ID 着色，会造成严重标签泄漏。

## 6. 从哪里开始做 ID 算法

优先使用 `labels/frame_pairs.csv` 中的 pair。真正连续时序建议从：

- S01 / S02 / S03：左右/双端增长 + NEW
- S05：移动 + yaw + rolling observation mismatch
- S06：遮挡生命周期
- S07：NEW + OCCLUDED 同时出现
- S08：两层
- S09：一次两个 NEW
- S10：相似几何 ID 交换压力
- S11：第一层满 10 后新增第二层

P01/P02/S04/S12 是“相同基准的受控变量测试”，不是连续真实装载历史；详见 `labels/sequence_index.csv`。

## 7. 标注层级

- `labels/object_gt.csv`：一帧一个物理盘条状态。
- `labels/frame_gt.jsonl`：同样信息的结构化 JSONL。
- `labels/frame_pairs.csv`：哪些前后帧应该用于关联测试。
- `labels/transition_gt.csv`：每个 pair 的 global ID 对应、instance 对应、NEW、visibility event、位移 GT。
- `labels/frame_semantics.csv`：第一层 Slot 占用、左右 Cluster、正常中间空 Slot。
- `labels/neighbor_gt.csv`：连续 Cluster 内的直接语义邻居以及模型 gap。

## 8. 复现

该包不再依赖 V2 数据，也不依赖聊天记录：

```bash
python code/generate_dataset.py --out regenerated_v3
python code/validate_dataset.py
```

生成器只需要 Python + NumPy，以及包内 `assets/source_real_coils.zip` 与 `assets/scene_spec.csv`。

## 9. 已知边界

1. 真实原型只有 7 个，很多 global ID 会复用原型；这正好可作为“相似几何不能靠外观识别”的压力，但不代表真实规格分布。
2. 滚动表面变化是统计扰动，不是真实同一盘条滚动前后的新表面。
3. 遮挡是左右边缘语义模型，不是完整多相机几何遮挡仿真。
4. `diameter_nominal` / `length_nominal` 是生成时稳定几何属性，用于 GT/先验；不要把当前残缺点云的 bbox 当真值。
5. S13、S14 是单帧压力快照，不应独立拿来评估跨帧 ID 关联。

## 10. 当前最重要的评价原则

不要只报 overall accuracy。至少同时报告：

- `wrong_id_count`：**首要指标，目标为 0**；
- `ID precision among assigned`：只统计算法真正继承 ID 的对象；
- `coverage`：有多少可视对象被安全赋予 ID；
- `UNCERTAIN rate`；
- NEW precision/recall；
- OCCLUDED 识别结果。

在这个项目里，99% 覆盖率但出现一次 ID 交换，可能比 90% 覆盖率且 0 错 ID 更差。

## 11. 跟踪算法设计（新增）

semantic matcher 的需求和详细设计已经冻结。继续开发时依次阅读：

- `DESIGN_HANDOFF.md`
- `docs/TRACKER_REQUIREMENTS_V1.md`
- `docs/TRACKER_DETAILED_DESIGN_V1.md`
- `docs/IMPLEMENTATION_AND_TEST_PLAN.md`
- `docs/DECISIONS_AND_TBDS.md`
- `design/tracker_config.example.json`


## 12. V0.8 双层语义更新

当前可执行版本以 `WEB_HANDOFF.md` 为入口。V0.8 新增：第一层满 10 后才允许第二层；第二层激活后第一层 TrackState 冻结；第二层 slot k 由第一层 k/k+1 支撑对定义；支持双层初始帧 bootstrap 与 D01–D10 专用回归。详见 `docs/DOUBLE_LAYER_SUPPORT_V0_8.md`。
