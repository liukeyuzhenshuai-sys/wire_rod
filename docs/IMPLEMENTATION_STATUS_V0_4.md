# Implementation Status V0.4

## 已完成

- Persistent sequential session。
- 第一帧初始化检测：ID/center/radius/length/yaw/layer/slot。
- global ID 固定颜色。
- NEW 出生帧特殊颜色。
- 下一帧连续识别。
- 交互式创建下一帧。
- 已有盘条 XYZ/Yaw 编辑。
- roll mild/heavy 观测扰动。
- 左/右/双侧边缘遮挡，保证非零点。
- NEW D/L/XYZ/Yaw/Layer/prototype 编辑。
- 多 NEW。
- robot hint XYZ 偏差。
- 用户创建 PLY 平铺在工程根目录。
- 创建后立即运行 tracker。
- 用户可保存失败案例 ZIP。
- Random seed / actions / GT / history / result / config 全量保存。
- 新状态 `UNOBSERVED`：零当前点不能叫 OCCLUDED。

## 当前已知不足

1. PARTIAL/OCCLUDED 的自动识别目前主要使用 point-count ratio，是 V0.4 的弱模块；不同重建视角/密度可能导致误判。
2. 交互编辑器当前使用 benchmark 真实原型重新采样已有盘条；这是测试工具能力，不属于生产 matcher 输入。
3. 还未提供鼠标直接拖拽 2D 俯视盘条；当前使用数值 `dx/dy/dz/dyaw` 输入。数值方式更易复现，拖拽可以后续加入。
4. 用户可以创建违反硬业务约束的场景，当前只 warning，不阻止。这是刻意设计，用于异常压力测试。
5. failure ZIP 目前需要用户手动返回；尚未实现自动导入 ZIP 成 regression test 的 UI。

## 下一步优先级

P0：使用用户真实保存的 failure ZIP 改算法并转永久回归。

P1：改进部分遮挡识别：结合点数比例、局部截面覆盖、边缘缺失方向、历史稳定半径和相机可见性，而不是只看 point count。

P2：增加俯视图拖拽编辑，但仍以数值动作 JSON 作为最终可复现记录。

P3：支持失败 ZIP 一键导入并重跑。
