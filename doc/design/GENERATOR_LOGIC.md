# V3.1 数据生成逻辑

## 1. 原型归一化

7 个真实 PLY 的文件名只表示不同实例，不带业务语义。原始数据主要轴向为 source X；统一映射：

- source Y → scene X
- source X → scene Y
- source Z → scene Z

使用 1%/2%～98%/99% robust 范围重心化，并去除明显离群点，保留真实局部表面。

## 2. 姿态

场景最终姿态：先对局部原型施加观测扰动/遮挡，再做场景位姿：

1. 绕 scene Z 旋转 `yaw_deg`；
2. 平移到 `center_x/y/z`。

yaw 是真实刚体姿态变化；它与滚动观测变化完全分开。

## 3. rolling observation mismatch

`roll_mild`：径向扰动幅值约 1.5cm，保留约 92%，再约 6% 局部重采样。

`roll_heavy`：径向扰动幅值约 3cm，保留约 79%，再约 10% 局部重采样。

径向扰动使用多个角频率的平滑函数叠加随机项，表示非理想盘条表面滚动后不同物理区域暴露。它不是绕自身 Y 轴 rigid rotation。

## 4. 视角退化

- `top_view`：顶部点保留概率更高。
- `side_view`：一侧点更密，但不会挖中央洞。
- `degraded`：较强表面扰动 + 稀疏 + side-based occlusion。

这些是统计近似，不是相机 ray tracing。

## 5. partial occlusion

`edge_occlude` 只操作局部截面 X 的最左/最右外缘。`severity` 决定边缘作用宽度，最大约 36% span。

中央局部带（约 38%～62% X span）被显式保护，避免产生不合理的中间大孔洞。

`occlusion_side`：`left/right/both`。

## 6. 完全遮挡

`visibility=OCCLUDED`：该对象写入 object/transition GT，但该帧 PLY 中不生成它的任何点。

## 7. local instance_id

每一帧的 `instance_id` 已在场景规格中随机化。算法不能利用前后帧 instance 数字相同来推断身份。

## 8. 颜色

RGB = global ID 颜色，只为了 CloudCompare/Open3D 人工目检。**算法输入时必须删除 RGB。**

## 9. 自包含复现

`code/generate_dataset.py` 不依赖 V2。输入只有：

- `assets/source_real_coils.zip`
- `assets/scene_spec.csv`

随机种子和每对象哈希种子固定，因此生成可重复。

## V0.7 TOP_DOWN_Z 生成逻辑

顶部观测不使用“砍掉下半部”的几何捷径。场景全部对象先按真实 pose 生成，再将所有对象点合并，对 XY 视线做全局 z-buffer：每个 XY cell 仅保留最高 Z 附近一个有限厚度的 surface patch。该顺序保证 layer2 对 layer1 的遮挡由空间投影自然产生。

当前操作模型要求每个历史盘条仍有部分可见点。对于 synthetic 极端重叠导致某对象被 z-buffer 清空/过少，生成器只恢复其最高 Z 的少量 top-facing samples并记录 warning；该步骤是数据有效性保护，不属于 matcher 的遮挡推断。



## V0.8 Double-Layer 数据生成
D01–D10 使用固定 10-slot L1 foundation。L2 slot k 的中心由 L1[k]/L1[k+1] 几何支撑谷计算；正常 L2 对象均围绕合法 support slot 生成。D08/D09/D10 分别故意违反 foundation lock、support slot、capacity。TOP_DOWN_Z 场景继续使用全场 XY z-buffer，且 D-series 每个物理对象强制保留正点数。生成入口：`python code/generate_double_layer_dataset.py`。
