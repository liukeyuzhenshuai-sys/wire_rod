# Web 业务目检器产品需求 V1

> 状态：**FROZEN REQUIREMENT**  
> 本文优先于旧版“pair 调试面板”交互。旧 Candidate / Anchor / DP 页面只保留为高级调试能力，不得继续作为首页主流程。

## 1. 产品目标

Web 的第一目标不是展示算法术语，而是让业务人员逐帧目检：

1. 每个盘条当前被赋予了哪个 `global_id`；
2. 同一物理盘条跨帧是否保持同一 ID；
3. 当前帧谁是 `NEW`、谁是 `OCCLUDED`、谁是 `UNCERTAIN`；
4. 每个盘条的位置、方向、半径/外径、长度和移动量是否合理；
5. 连续 Cluster 内相邻盘条间距是否合理。

错误 ID 的严重程度高于 `UNCERTAIN`。界面必须让 ID swap 可以被肉眼立即发现。

## 2. 用户主流程

### 2.1 选择初始帧

首页只列出**可作为连续跟踪起点的帧**，优先按 Sequence 展示其第一帧。

用户选择一个初始帧后点击 `开始检测`：

- 清空旧 `TrackManager`；
- 只读取该帧合法算法输入；
- 对第一帧独立完成盘条实例几何检测与跟踪初始化；
- **不得读取当前帧 GT global_id 来初始化算法身份**；
- 合成数据中的 GT 仅允许在用户主动开启 `显示 Ground Truth` 后由评测层读取。

### 2.2 第一帧展示

第一帧不做跨帧匹配，应展示每个检测盘条的：

- `global_id`（tracker 初始化后的长期 ID）；
- `instance_id`（可放在详情，不作为主要业务字段）；
- `center_xyz`；
- `axis / yaw`；
- `radius` 与 `diameter`；
- `length`；
- `layer / slot / cluster`；
- 几何质量/置信度。

3D 视图需要可视化：盘条点云、中心点、轴方向；拟合圆柱/外径轮廓可作为可开关叠加层。

### 2.3 下一帧

页面提供显著的 `下一帧 →`。点击后必须：

```text
TrackerState(t-1) + CurrentObservation(t)
                    ↓
                Matcher
                    ↓
              TrackerState(t)
```

下一帧必须使用**上一步算法自己提交的 Track State**，不得重新从 previous-frame GT bootstrap。否则不能验证累计 ID 漂移。

当前帧主要展示：

- 每个盘条的最终 `global_id`；
- `MATCHED / NEW / OCCLUDED / UNCERTAIN`；
- 当前位置；
- 相对上一帧 `dx/dy/dz/displacement`；
- yaw 变化（若质量足够）；
- 半径/外径、长度；
- 同 Cluster 直接邻居的 gap/contact state。

### 2.4 Sequence 结束

当已经处理到当前 Sequence 的最后一帧，再次点击 `下一帧` 时不得循环、不得自动跳到另一 Sequence。必须弹窗：

```text
当前序列已经检测完成
- 已处理帧数
- WRONG_ID（开启 GT 时）
- NEW / OCCLUDED / UNCERTAIN 统计

[关闭] [选择新的初始帧]
```

选择新的初始帧必须清空 TrackManager 和本次 UI session 状态。

## 3. 颜色与状态编码（冻结规则）

### 3.1 身份颜色

**颜色只表示身份。**

- 每个已确认 `global_id` 在一个 tracking session 内永久绑定一种稳定颜色；
- 相邻帧同一个 `global_id` 必须使用完全相同的颜色；
- 颜色映射必须 deterministic，不能因为刷新页面或点数变化而重新随机；
- 图例始终显示 `颜色 → global_id`。

推荐实现：基于 `global_id` 从离散 palette / HSV golden-angle palette 计算颜色，并缓存到 session。

### 3.2 NEW

`NEW` 是状态，不应永久覆盖身份颜色。

- 在**首次确认 NEW 的当前帧**，用特殊强调色/外框（例如亮红或橙）突出，并标注 `NEW → IDx`；
- 下一帧开始，该盘条已经是历史 Track，应恢复使用 `IDx` 的固定身份色；
- NEW 的文字/图标必须同时存在，不能只靠颜色。

### 3.3 OCCLUDED

- OCCLUDED 必须保留该 `global_id` 的身份颜色；
- 当前没有实际点云时，在预测 ROI/中心绘制半透明或虚线代理几何；
- 标注 `IDx / OCCLUDED`；
- 不允许因为当前没点就从界面和 Track 列表中消失。

### 3.4 UNCERTAIN

- 未安全继承 global ID 的当前实例不得强行使用某个历史 ID 的身份色；
- 使用中性灰/斜纹/特殊轮廓并标 `UNCERTAIN`；
- 如果只是“某个已知历史 ID 当前定位不确定”，则保留历史 ID 色但状态必须明确，禁止和“未分配当前 instance”混淆。

### 3.5 原则

```text
颜色 = identity
透明度 / 边框 / 图标 = state
文字 = business meaning
```

不得让同一颜色同时表达 identity 与状态两个维度。

## 4. 主页面布局

建议默认布局：

1. 顶部：Sequence / 当前帧编号 / 初始帧入口 / `下一帧` / `显示 GT`；
2. 主区：当前帧 3D 点云（第一帧）或上一帧 vs 当前帧并排（后续帧）；
3. 盘条状态表；
4. Layer / Slot / Cluster 语义俯视图；
5. 选中盘条后的解释面板。

可以额外提供“叠加比较”模式：上一帧半透明、当前帧实色，并画历史中心到当前中心的位移箭头。但默认仍以左右对照优先。

## 5. 盘条状态表

主表至少包括：

```text
ID | State | Layer | Slot | Center XYZ | Radius | Length | Yaw | ΔXYZ | Distance | Quality
```

NEW：位移显示 `N/A`，不能显示 0 造成“未移动”的误解。

OCCLUDED：当前测量字段可显示 `predicted`/低置信，不得伪造精确测量值。

## 6. 相邻间距可视化

只对**同一连续 Cluster 内的直接语义邻居**画 gap。

例如：

```text
0 1 2 . . . . 7 8 9
```

允许：`0-1`, `1-2`, `7-8`, `8-9`。

禁止把 `2-7` 的正常中间空区作为邻居 gap 或异常 gap。

视觉上可以在俯视图/3D 图中画：

```text
ID1 ── 0.043 m / CONTACT ── ID2
```

并显示 gap uncertainty / `CONTACT | GAP | UNCERTAIN`。

## 7. GT 目检模式

默认关闭 GT。打开 `显示 Ground Truth` 后，由**独立评测层**叠加：

```text
预测 ID | GT ID | 结果
ID2     | ID2   | ✓
UNCERTAIN | ID4 | △ 保守拒绝
ID5     | ID6   | ✕ WRONG ID
```

WRONG ID 必须使用明显的错误提示。GT 开关不能改变 matcher 的输入或重新运行出不同结果。

## 8. 算法解释的层级

首页不再直接堆 `Candidate / Anchor / DP`。

选中某个盘条后，先显示业务语言解释：

```text
为什么判断为 ID2：
✓ 同层
✓ 位于正确 Slot/邻接顺序
✓ 在历史运动 ROI 内
✓ D/L 与稳定历史规格兼容

为什么排除 ID3：
× 超出运动 ROI
```

再提供折叠项：`高级调试 >`，其中才展示 Candidate cost、Anchor、DP path、gate reason code 等。

## 9. 验收条件

Web 重构完成至少满足：

1. 可从列表选择一个有效 Sequence 初始帧；
2. 第一帧独立检测并可视化 center/axis/radius/length/ID；
3. 连续点击下一帧时使用持久 TrackManager，不从 GT 重置；
4. 同 global ID 跨帧颜色完全一致；
5. NEW 首次出现有特殊强调，但下一帧恢复其固定 ID 色；
6. OCCLUDED 仍保留在场景语义视图中；
7. 同 Cluster 直接邻居 gap 可视化；
8. 最后一帧后弹窗并允许重新选初始帧；
9. GT 默认关闭且与 matcher 输入隔离；
10. 算法内部术语默认隐藏，必要时可展开调试。

## V0.7 Web 追加要求

创建下一帧必须提供 `常规多视角 / 仅顶部（沿 -Z）` 选择；TOP_DOWN_Z 预览必须调用正式点云生成器，不得仅改变 2D 图示。盘条拖动时界面读取后端 tracker config，实时显示当前 dx 与 normal/hard X envelope；越界只警告，不阻止生成。识别结果必须能同时显示 `MATCHED` identity 与 `ABNORMAL` motion reason。



## V0.8 Web 追加要求
创建下一帧支持显式“已进入第二层”信号。新增盘条可选 Layer2；Layer2 NEW 应提供“吸附到最近第一层支撑槽”操作。第二层激活后，界面应把 L1 大位移显示为结构异常预警，而不是普通 movement。D01–D10 必须出现在初始 Sequence 选择器。
