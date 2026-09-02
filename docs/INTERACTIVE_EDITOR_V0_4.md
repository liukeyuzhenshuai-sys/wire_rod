# 盘条交互式场景编辑器与连续 Tracker V0.4

## 1. 目的

V0.4 将原来的“播放预生成数据集”升级为可人工构造下一帧的压力测试工具。

固定业务原则：

- `global_id` 表示长期物理身份；跨帧同 ID 必须是同一盘条。
- 可输出 `UNCERTAIN`，但禁止为了覆盖率错误继承 ID。
- 第一层最多 10 个 Slot；左/右边界可分别向中间增长，中间大空区允许正常存在。
- 已有盘条在本任务中不支持删除。
- **OCCLUDED 的定义已经修改：必须存在当前帧的非空部分点云。**
- 历史 ID 当前完全没有任何点时，状态只能是 `UNOBSERVED`（或更高层业务异常），不能自信称为 OCCLUDED。
- 普通滚动观测差异不得通过“中间挖大洞”生成；使用小尺度表面扰动、重采样和点密度变化。
- 邻居遮挡只从盘条左右边缘逐渐缺失，并保护中间可见区域。

## 2. Web 工作流

1. 从列表选择某个 Sequence 第一帧。
2. 点击“开始检测”。系统初始化 TrackState。
3. 点击“下一帧”，系统使用自己上一帧提交的 TrackState 连续识别。
4. 到达当前分支最后一帧时：
   - 创建下一帧；或
   - 结束并重新选择初始帧。
5. “创建下一帧”允许对所有历史盘条设置：
   - `dx / dy / dz`
   - `delta_yaw`
   - 正常 / 轻度滚动观测变化 / 重度滚动观测变化
   - 无遮挡 / 左侧遮挡 / 右侧遮挡 / 双侧遮挡
   - 遮挡强度
6. 可以增加一个或多个 NEW，并设置：
   - 真实原型
   - 外径、长度
   - XYZ、Yaw、Layer
   - 滚动观测变化
   - 左/右/双侧部分遮挡
   - 机器人 target 相对真实位置的 XYZ 偏差
7. 点击“生成下一帧并运行识别”。新 PLY 会直接写在工程根目录，所有 PLY 仍保持同一目录。
8. 如果结果不合理，点击“保存问题案例”。Web 会下载一个可复现 ZIP。

## 3. 颜色规则

- 每个确认的 `global_id` 永久绑定固定颜色。
- 同一个物理盘条跨帧颜色不变。
- NEW 在出生帧使用红色特殊强调，并显示 `NEW -> IDx`。
- 下一帧开始 NEW 已经成为历史 ID，使用该 global ID 固定身份色。
- `UNCERTAIN` 使用灰色，不允许冒用某个历史 ID 的颜色。
- `UNOBSERVED` 保留 global ID 的身份色，但几何代理用虚线/半透明表示；当前点云本身没有该物体点。
- `OCCLUDED/PARTIAL_VISIBLE` 仍有真实当前点云，因此点云使用该 ID 固定颜色，状态文字单独标识。

原则：**颜色表示身份；线型/透明度/文字表示状态。**

## 4. 编辑器如何生成已有盘条

编辑器是测试数据生成器，不是 matcher。它可以使用真实原型/用户指定 GT 构造点云，但这些信息不会暴露给匹配算法。

已有盘条生成：

1. 读取该 global ID 的真实 prototype（Synthetic benchmark 的 editor-only bookkeeping）。
2. 根据 Track 的稳定 D/L 缩放 prototype。
3. 使用用户指定的 delta XYZ / delta yaw 生成当前物理姿态。
4. 可选滚动观测扰动：小尺度径向扰动、稀疏、重采样、厘米级点噪声。
5. 可选边缘遮挡：只从 local-X 左/右边缘删点。
6. 强制 `visible_points > 0`，已有盘条不可被编辑器完全删除。
7. 每一帧重新随机生成 local `instance_id`。

## 5. NEW 生成与 robot hint

NEW 的真实几何由用户编辑；Matcher 只获得机器人允许提供的信息：

- nominal diameter
- nominal length
- approximate target center

编辑器可以人为给 robot target 加 `hint_dx/dy/dz`，构造“不准确机器人目标”的压力案例。

NEW 的真实 `global_id/slot` 是测试 GT，不会进入 matcher。

## 6. 遮挡定义

### 6.1 PARTIAL_VISIBLE / OCCLUDED

必须满足：

```text
current instance exists
visible point count > 0
```

遮挡形态：

- left
- right
- both

强度越高，边缘丢点越明显；但中央区域被保护，且存在最低可见点数约束。

### 6.2 UNOBSERVED

```text
history ID exists
current frame has zero points for this physical object
```

这不叫 OCCLUDED。当前算法必须保留历史 Track，但不能声称已从当前点云确认遮挡。

预生成 V3/V4 中少量旧 benchmark 场景曾把“零点”标成 `OCCLUDED`。这些属于**Legacy GT**；V0.4 matcher 提交状态时按新业务定义解释为 `UNOBSERVED`。不要用旧标签重新推翻新定义。

## 7. 用户允许创建不物理的对抗场景

编辑器不会阻止：

- ID 顺序反转
- Slot 几何重叠
- NEW 远离正常增长边界
- 非典型 Y/Z 变化

但会给出 warning。这些数据用于测试算法面对不可能/异常输入时是否能输出异常或 UNCERTAIN，而不是错误继承 ID。

## 8. 保存失败案例

点击“保存问题案例”后生成 ZIP，至少包含：

- `previous_frame.ply`
- `current_frame.ply`
- `previous_scene_gt.json`
- `current_scene_gt.json`
- `edit_actions.json`
- `robot_hints.json`
- `generation_seed.json`
- `tracker_state_before.json`
- `algorithm_result.json`
- `tracker_config.json`
- `algorithm_version.json`
- `session_context.json`
- `user_note.txt`

该 ZIP 是重新修改算法的首要输入。必须优先复现用户失败案例，再修算法；修复后将失败案例加入永久回归集。

## 9. PLY 信息隔离

Interactive PLY 为便于人工目检，会保存：

- XYZ
- RGB
- global_id
- instance_id
- layer
- slot
- visibility

但 matcher 的 `tracker/ply.py::load_algorithm_input()` **只返回 XYZ + instance_id**。

禁止算法读取 RGB/global_id/layer/slot/visibility，否则属于 Ground Truth 泄漏。

## 10. 当前算法状态

V0.4 仍使用：

- 实例级几何观测
- Motion ROI / D / L Candidate Gate
- Semantic rank lock / Anchor
- Layer 内 Ordered DP
- NEW hint + 合法 Slot growth
- Persistent TrackState
- Gap / Layer / Slot 语义校验

新增语义：

- zero current points -> `UNOBSERVED`
- `OCCLUDED` 必须来自仍存在当前 instance 的部分点云

部分遮挡自动识别目前是保守启发式，主要依据当前点数相对历史参考点数。用户通过交互工具发现误判时，应保存失败 ZIP 用于进一步改进这一模块。
