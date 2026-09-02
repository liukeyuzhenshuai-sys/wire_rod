# 实现状态 V0.2 — 连续 Tracking Web

更新时间：2026-08-18

## 已完成

- Web 首页列出每个数据集/Sequence 的第一帧。
- `开始检测` 会清空旧状态，并从第一帧独立完成：实例几何观测、Layer/Slot/Cluster 初始化、长期 global ID 初始化。
- 第一帧初始化不读取当前 `global_id / slot / cluster` GT；语义位置由当前几何排列与配置规则推断。
- 页面提供显著的 `下一帧 →`。
- `temporal_adjacent` Sequence 的下一帧使用算法上一帧**自己提交的 TrackState**，不再从 GT bootstrap。
- `paired_to_baseline` 数据按其生成定义，每个变体与 F00 基准比较；页面明确提示它不是物理连续时间。
- 每个 global ID 跨帧使用确定性的固定颜色。
- NEW 首帧以红色菱形/文字强调 `NEW → IDx`，点云本身仍使用该 ID 身份色。
- OCCLUDED 保留 global ID，并使用历史中心/圆环代理可视化。
- UNCERTAIN 使用中性灰，不冒用历史身份颜色。
- 第一帧及后续帧显示：中心、方向/轴线、半径、外径、长度、yaw、Layer、Slot、Cluster、质量。
- 后续帧显示：`dx/dy/dz`、总位移、NEW/OCCLUDED/UNCERTAIN。
- 只对同一 Cluster 的直接 Slot 邻居计算/显示 gap；左右 Cluster 中间正常大空区不作为 gap。
- GT 默认关闭，通过独立 `/api/session/{sid}/gt` 端点在推理后读取，不进入 matcher。
- 最后一帧再次点击 `下一帧` 弹出 Sequence 完成对话框，可重新选择初始帧。
- Candidate / Anchor / Ordered DP 默认折叠在“高级调试”区域。

## 同步修复

旧 `infer_new_slot` 在“历史只有右侧 Cluster”时可能被场景中点误导，错误新建左侧 Slot。V0.2 已修改为：

- 只有 left cluster → 只能从 left cluster 内侧边界增长；
- 只有 right cluster → 只能从 right cluster 内侧边界增长；
- 左右 cluster 同时存在 → 比较 NEW 与两个内侧边界的距离；
- L1 满 10 个 → 禁止再创建 L1 Slot。

## 连续状态回归

`results/sequential_session_regression.json`

从各 Sequence 第一帧初始化，然后只用算法自己提交的 TrackState 向后推进：

| Sequence | 帧数 | WRONG_ID |
|---|---:|---:|
| S01 | 4 | 0 |
| S02 | 3 | 0 |
| S03 | 5 | 0 |
| S05 | 4 | 0 |
| S06 | 5 | 0 |
| S07 | 2 | 0 |
| S08 | 3 | 0 |
| S09 | 2 | 0 |
| S10 | 3 | 0 |
| S11 | 2 | 0 |

总计 33 帧，累计 `WRONG_ID=0`。

注意：合成数据 NEW hint 仍由 benchmark adapter 模拟机器人输入，仅暴露目标位置 + D/L，不暴露当前 GT global ID。

## 当前主要限制

1. 第一帧 Layer/Slot 初始化规则目前针对当前合成车厢坐标范围做了可配置启发式，生产数据需重新标定 `initialization` 参数。
2. 几何检测仍为 V1 轻量估计，不是最终鲁棒部分圆柱拟合器。
3. Gap 当前采用模型估计和简单 CONTACT/GAP/UNCERTAIN 阈值，生产阈值仍需真实数据标定。
4. Web session 当前保存在进程内存；服务重启会清空 session，这是本地目检工具的预期行为。
