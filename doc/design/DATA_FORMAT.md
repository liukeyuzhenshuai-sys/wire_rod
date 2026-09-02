# 数据与标注格式

## PLY：一个文件就是一个完整场景帧

每个点字段：

| 字段 | 类型 | 用途 |
|---|---|---|
| x,y,z | float32 | 合法算法输入 |
| instance_id | int32 | 合法算法输入；仅单帧有效 |
| red,green,blue | uint8 | 仅目检；按 global ID 着色，严禁算法使用 |
| global_id | int32 | GT，严禁推理读取 |
| layer | uint8 | GT/语义实验标签 |
| slot | int16 | GT/语义实验标签 |
| visibility | uint8 | GT；1 visible, 2 partial, 3 occluded（完全 occluded 没有点） |

注意：完全 OCCLUDED 的物体不会出现在点级 PLY 中，因此必须从 `object_gt.csv` / `transition_gt.csv` 获取其存在性。

## object_gt.csv

一行 = 某帧中某个物理 global ID。主要字段：

- `file, sequence_id, frame_index`
- `global_id, instance_id`
- `layer, slot, cluster`
- `visibility, is_new`
- `center_x/y/z`：生成真值中心
- `yaw_deg`
- `diameter_nominal, length_nominal`
- `dx/dy/dz_from_prev`：原生成规格中的运动记录（对非真实连续变体不要机械解读）
- `observation, severity, occlusion_side`
- `target_x/y/z`：存在时表示机器人 NEW 先验位置

## transition_gt.csv

这是做跨帧 ID 最直接的 GT。对 `frame_pairs.csv` 中每个 pair，每个 global ID 一行：

- `previous_instance_id -> current_instance_id`
- `relation_gt = SAME_OBJECT / NEW`
- `visibility_event_gt`
- `dx_gt_m, dy_gt_m, dz_gt_m, displacement_gt_m`
- `layer_gt, slot_gt`

当前完全 OCCLUDED 时 `current_instance_id=-1`，但 `global_id` 仍存在。

## frame_semantics.csv

记录第一层 10 个 Slot 的占用和 Cluster：

- `left_cluster_slots`
- `right_cluster_slots`
- `normal_middle_free_slots`
- `layer1_unoccupied_slots`

其中 `normal_middle_free_slots` 是正常装载空区，不应该触发“相邻 gap 过大”异常。

## neighbor_gt.csv

只对同一语义 Cluster 的直接邻居建立关系。关键字段：

- `left_global_id/right_global_id`
- `left_slot/right_slot`
- `model_surface_gap_x_m`

这里的 gap 是生成模型真值：

`abs(center_x_right-center_x_left) - (D_left+D_right)/2`

它是语义/模型 gap，不是用残缺当前点云直接测得的表面最近距离。

## V0.7 Visibility / View Mode 补充

`VIEW_PARTIAL` 表示观测视角天然只覆盖盘条部分表面，例如 TOP_DOWN_Z。它不是物体遮挡状态。`OCCLUDED` 仍要求存在非空 current instance。

Z 系列 `frame_gt.jsonl` 增加 `view_mode`：`TOP_DOWN_Z` 或 `MULTI_VIEW`。CSV `object_gt.csv` 的 `observation=top_down_z` 与 `visibility=VIEW_PARTIAL` 可用于 GT/可视化；算法读取 PLY 时仍只使用 XYZ + instance_id。



## V0.8 双层标签补充
`labels/support_gt.csv` 记录上层对象与两个下层支撑对象/支撑槽关系。`labels/frame_semantics.csv` 的 `layer2_slots` 记录当前上层离散槽位。D-series 的 `object_gt.csv` 继续提供 layer/slot/visibility/point_count，并遵守 point_count>0 的遮挡业务规则。
