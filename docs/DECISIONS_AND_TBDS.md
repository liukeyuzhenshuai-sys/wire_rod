# 已冻结决策与待标定项

## A. 已冻结，不应在实现中随意改变

1. global ID 是长期身份；instance_id 仅单帧有效。
2. 错绑 ID 不允许；UNCERTAIN 允许。
3. 不以局部表面点云相似度作为身份主证据。
4. 同层同连续 Cluster 的 Slot 顺序不能反转。
5. 第一层最多 10 Slot。
6. 第一层允许左/右两个边缘 Cluster，中间大空区正常。
7. 历史盘条正常不移除；消失不能通过重分 ID “修复”。
8. 不换层。
9. 先求高置信可视组合，再解释 NEW/OCCLUDED。
10. 遮挡对象优先级低于高质量可视对象。
11. 滚动观测变化与 yaw 是不同概念。
12. 普通滚动观测变化不能通过把已有部分点云绕自身轴刚体旋转来模拟/匹配。
13. 推理禁止使用 RGB/current global_id/current layer-slot GT。
14. 同一 Cluster 直接邻居才计算业务 gap；跨正常中间空区不算。
15. 一次通常 1 个 NEW，但接口和求解器兼容多个。

## B. 已确认范围，但实现必须配置化

| 参数 | 当前信息 |
|---|---|
| 外径 | 约 1.1–1.5 m |
| 长度 | 约 0.5–1.7 m |
| 点噪声 | 约 3–5 cm/点量级 |
| 历史 X 最大移动 | 暂定不超过约 2 m |
| 第一层 | 最大 10 |
| 第二层规模 | 通常约 8，不当作已确认硬上限 |
| V1 总时延 | 从带 instance_id 点云开始 <2s |

## C. 必须由真实数据标定，禁止静默猜测

1. `motion_roi_y/z`；
2. 历史 X 位移的典型而非绝对上限；
3. robot target 的实际误差；
4. D/L 名义值误差；
5. layer Z 分界；
6. yaw 合理范围；
7. anchor quality threshold；
8. best/second margin；
9. partial/occlusion quality threshold；
10. contact/warning/abnormal gap 阈值；
11. center_sigma/D_sigma/L_sigma 的质量映射；
12. 连续多帧 OCCLUDED 后 ROI 是否扩张以及扩张速度。

## D. 后续可选增强，不阻塞 V0

1. Camera ray visibility；
2. Top-K 全局 layout hypotheses 跨帧延迟消歧；
3. 图像视觉特征作为低置信辅助证据；
4. NEW 机器人 approach/release direction 的因果运动 prior；
5. 第二层 `SUPPORTED_BY` 图优化；
6. gap 近阈值局部点云精细验证；
7. C++ 优化（只有 Python 不满足时延时再做）。

## V0.7 已冻结决策

- normal X envelope 与 hard X envelope 分开；身份与运动异常分开表达。
- 当前默认 `x_normal=2.0m`, `x_hard=2.4m`，仅用于当前测试配置，生产需真实数据校准。
- TOP_DOWN_Z 使用全场 XY z-buffer；禁止将简单 z 中心裁剪当成正式模型。
- 顶视天然部分可见用 `VIEW_PARTIAL`；zero-point 仍不属于正常 OCCLUDED。

## V0.7 待校准

- 真实顶部相机的 z-buffer cell/thickness、重建密度与可见比例分布。
- TOP_DOWN_Z 自动检测阈值与误判率。
- 真实装载冲击下 normal/hard X motion envelope。



## V0.8 已冻结决策
1. 第一层满 10 才进入第二层。
2. 第二层激活后，第一层不是普通 movable track，而是 locked support foundation。
3. 第二层身份由 support pair/upper slot 主导。
4. 允许外部 second-layer signal。
5. 首帧已经双层必须有专用 bootstrap。
6. 严重遮挡仍要求正实例点；零点不可作为正常 OCCLUDED。
7. 第一层大位移视为塌方/结构异常，可 UNCERTAIN，不强行追踪。

### V0.8 待校准
- foundation lock XY/Z/yaw 阈值；
- L2 support slot normal/hard error；
- 真实工况第二层最大容量（当前默认 8）；
- 双层 bootstrap 在真实重遮挡数据上的最低有效点数/可见比例。
