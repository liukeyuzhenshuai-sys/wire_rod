# 算法测试使用指南

## 1. 先读 pair，不要自己猜帧关系

读取：`labels/frame_pairs.csv`。

`temporal_adjacent` 是真实模拟的前后状态；`baseline_variant` 是“同一个 baseline vs 单因素变体”。

## 2. 推理输入必须做标签隔离

推荐直接：

```python
from code.ply_io import load_algorithm_input
xyz, instance_id = load_algorithm_input('S06_F01_id2_partial.ply')
```

不要直接用 Open3D 读取所有属性后无意中把 RGB/global_id 喂给 matcher。

## 3. 建议 matcher 输入结构

对每个当前 `instance_id` 聚合成实例点云，提取你自己的观测：

```text
instance_id
estimated center / axis / diameter / length / quality
```

历史状态则由上一帧已经确认的 global ID 状态组成。对于真实算法，这部分不是 GT，而是 tracker 自己维护的状态。

## 4. 预测输出建议

`labels/prediction_template.csv`：

```text
pair_id,current_instance_id,pred_global_id,pred_state,confidence
```

状态建议：

- `MATCHED`
- `NEW`
- `UNCERTAIN`
- `OCCLUDED`（current_instance_id=-1，pred_global_id 为历史 ID）

## 5. 评测

```bash
python code/evaluate_tracking.py your_predictions.csv
```

核心首先看 `WRONG_ID` 是否为 0。之后再看 coverage/uncertain。当前示例 evaluator 是最小版，后续 matcher 建成后可以扩充 NEW precision/recall、visibility、中心误差、gap 误差。

## 6. 推荐 V0 测试集

先使用：S01、S03、S06、S07、S10、S11。

原因：这些序列直接覆盖本项目最关键的语义：双端装载、NEW、OCCLUDED、ID swap 风险、第一层容量。

## 7. 不要用 GT 做推理先验

`layer/slot/global_id/visibility` 当前是标签。真正算法将来如果能从历史 tracker 状态获得 layer/slot，它们可以作为“历史状态”；但不能在当前帧直接读取当前 GT label 来作弊。

例如正确做法：上一帧已经确认 ID2 的 layer/slot，在历史状态中携带；当前帧只从 XYZ+instance_id 产生观测，再由语义推断其 ID。


## V0.8 双层测试建议
优先目检 D05、D06、D08、D09、D10。D05/D06 看 bootstrap；D08 看第一层异常是否报警且不污染后续恢复；D09 看上层离开支撑谷是否拒绝；D10 看 8 个第二层盘条后继续新增是否触发容量异常。自动回归：`python tests/run_double_layer_regression.py`。
