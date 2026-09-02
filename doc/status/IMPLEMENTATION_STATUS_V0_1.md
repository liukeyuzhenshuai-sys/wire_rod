# V0.1 实现状态

## 已实现

| 模块 | 状态 | 说明 |
|---|---|---|
| 输入隔离 | DONE | PLY inference 只读 XYZ + instance_id |
| Instance aggregate | DONE | NumPy mask / split |
| Center observation | DONE V0 | quantile bbox center；partial 下尚未使用 history prior 优化 |
| Diameter/length | DONE V0 | yaw-aligned robust projection；当前只用于 coarse evidence |
| Yaw | WEAK | 仅弱特征，不允许主导 ID |
| Motion ROI | DONE | X/Y/Z configurable |
| Candidate Graph | DONE | layer/ROI/D/L gate |
| Anchor | DONE | quality + best cost + margin |
| Ordered DP | DONE | MATCH/SKIP/INSERT，顺序不可反转 |
| Basic NEW | DONE | synthetic robot hint + legal Slot |
| Basic OCCLUDED | DONE | semantic bracket rule |
| Global Validator | DONE | duplicate/layer1 capacity 等 |
| Pair evaluator | DONE | WRONG_ID / NEW / OCC |
| Web inspector | DONE | 3D + flow + 5 stage audit |
| Persistent TrackManager | NOT YET | 下一优先级 |
| Full sequence self-history | NOT YET | 当前每 pair bootstrap confirmed history |
| Gap | NOT YET | 仍使用数据集 GT 做数据检查，不是 tracker 输出 |
| Camera visibility | NOT YET | V1.5 |

## 当前阈值

`tracker_config.json` 里的参数是**synthetic benchmark 初始值**，不是生产参数。

特别是：

```text
x_default_m = 1.5
y_default_m = 0.4
z_default_m = 0.3
anchor thresholds
D/L gates
```

必须用真实连续数据重新标定。

## 当前结果的正确解读

45 个 pair 的 `WRONG_ID=0` 说明：

> 在“上一帧历史状态已正确确认”的前提下，当前 Candidate + Anchor + Ordered DP 在 V3.1 synthetic benchmark 上没有出现旧 ID 错绑。

它**不能证明**长期累积 20 帧后仍无漂移，因为 Persistent TrackManager 还没接上。

## UI Redesign Requirement Status

`docs/WEB_PRODUCT_REQUIREMENTS_V1.md` 已冻结新的业务目检交互，但**当前 V0.1 pairwise Web 尚未实现该新交互**。

待实现项：
- [ ] 初始帧/Sequence 选择器；
- [ ] Stateful TrackManager session；
- [ ] 首帧 center/axis/radius/length 可视化；
- [ ] 固定 global-ID color registry；
- [ ] NEW/OCCLUDED/UNCERTAIN 状态视觉规则；
- [ ] 邻居 gap 图层；
- [ ] Next-frame 连续运行；
- [ ] Sequence-end modal + reset；
- [ ] 业务解释优先、Candidate/Anchor/DP 折叠。

因此接手开发时，应把“Web inspector DONE”理解为**旧版调试 Web 已完成**，而不是新版业务目检器完成。
