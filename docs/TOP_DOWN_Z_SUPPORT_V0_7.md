# TOP_DOWN_Z 支持设计 — V0.7

## 1. 目标

支持盘条只从上方观察的点云：相机位于盘条上方，视线沿世界坐标 `-Z`。下表面天然不可见，上层盘条还会遮挡下层盘条的部分 XY 投影。

这不是普通 dropout，也不是简单保留 `z > center_z`。真实生成采用全场 point-cloud z-buffer。

## 2. 生成模型

`tracker/view_model.py::topdown_z_visible_mask()`：

1. 将全场点按 XY 量化到小网格；
2. 每个网格计算当前最高 Z；
3. 仅保留最高 Z 以下一个有限 surface thickness 内的点；
4. 因为 z-buffer 在全场一起做，所以 layer2 会自然遮挡 layer1 的同投影区域。

默认：`xy_cell_m=0.05`，`surface_thickness_m=0.055`。

当前业务假设明确“不存在历史盘条完全不可视”。因此 synthetic generator 的 `ensure_positive` 只在极端 z-buffer 导致某实例低于最小正观测时恢复该对象最高 Z 的少量 top-facing 样本并产生 warning。该保护只属于数据生成，不意味着 matcher 允许零点遮挡。

## 3. Observation 识别

算法合法输入仍只有 `XYZ + instance_id`，不读取 PLY 中 global_id/RGB/GT view mode。

`tracker/geometry.py` 使用 robust `Z span / observed horizontal diameter` 判断 top-only；frame consensus 用于统一处理被 layer2 强裁剪、单实例特征不明显的 layer1 对象。

## 4. 几何中心恢复

顶部数据的 bbox Z midpoint 会明显高于真实圆柱轴。因此：

```text
z_center ≈ robust_top_z - stable/observed_diameter / 2
```

V0.7 用 `TOP_SURFACE_MINUS_RADIUS` 替代 top-only 情况下的原始 bbox midpoint。X/Y center、axis/yaw、轴向长度仍从 XY footprint/轴向投影估计。

匹配阶段优先使用历史稳定 D/L，不会因为只看到上半表面就把本帧较小的几何范围当成新稳定尺寸。

## 5. Layer 语义

对于两层 TOP_DOWN_Z，单实例 bbox 可能因剪裁把下层 center 估高。因此在 candidate gate 之前使用：

```text
历史每层对象数量 + robot NEW hint layer
→ 得到 expected cardinality by layer
→ current observations 按 top_z 排序
→ 最高的 expected layer2 数量分给 layer2，其余 layer1
```

只有当前总 cardinality 与历史 + NEW hint 一致时才启用。计数异常时不会强行制造 layer partition。

## 6. Visibility 语义

- `VIEW_PARTIAL`：因为顶部相机天然只看到部分表面，不代表被其它盘条遮挡。
- `PARTIAL_VISIBLE / OCCLUDED`：真实邻居/上层盘条造成额外遮挡，并且仍有正 current instance。
- zero points：异常，不是正常 OCCLUDED。

## 7. 数据集

- `Z01`：纯顶视，yaw + 非均匀历史运动。
- `Z02`：纯顶视，左右双 cluster、NEW、大 gap、部分遮挡。
- `Z03`：纯顶视，第一层满 10 后逐步增加第二层；验证上层真实遮挡与 layer 语义。
- `Z04`：纯顶视，同规格/最近邻陷阱；要求身份来自语义而非完整圆柱几何。
- `Z05`：MULTI_VIEW → TOP_DOWN_Z → TOP_DOWN_Z → MULTI_VIEW，验证跨观测模式持久 ID。

共 25 帧，PLY 全部位于工程根目录。

## 8. 回归门槛

`python tests/run_topdown_regression.py`

V0.7 当前结果：5 sequences / 25 frames / `WRONG_ID=0` / `UNCERTAIN=0` / zero-point GT object=0。
