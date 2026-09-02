# Web 目检指南

## 页面区域

### A. 历史 / 当前 3D 点云

左侧是 previous frame，右侧是 current frame。当前点云支持三种颜色：

- `instance_id`：检查单帧实例；
- `预测 global ID / NEW`：检查 tracker 决策；
- `GT global ID`：只用于人工比较。

NEW 用 `1000 + slot` 的临时颜色 code 表示，这只是 UI 编码，不是最终 global ID 分配策略。

### B. 语义匹配流程图

上排：历史 global ID + Slot。  
下排：当前 instance。

- 灰线：通过 Hard Gate 的候选；
- 蓝线：Anchor；
- 绿线：最终 MATCH；
- `NEW / slot`：INSERT_CURRENT 经 NEW hint 与 Slot 规则确认；
- `OCCLUDED`：SKIP_HISTORY 经邻接语义解释。

### C. 五阶段审计表

1. **几何观测**：先检查中心是否明显偏错；
2. **Candidate Gate**：真值对象是否被错误 gate 掉；
3. **Anchor**：是否过早锁了一个模糊对象；
4. **Ordered DP**：MATCH/SKIP/INSERT 路径是否符合顺序语义；
5. **最终决策 + GT**：仅最后做结果核对。

## 推荐目检顺序

### S10

重点：尺寸完全相同的盘条是否仅因为位置近而换 ID。正常情况下 Ordered DP 必须保持 Slot 顺序。

### S06

重点：ID2 从 partial 到完全无 current instance，再重新出现。完全遮挡时不能把 ID2 给其他 instance。

### S07

重点：左侧同时 NEW，右侧 ID8 OCCLUDED。NEW 不能抢 ID8，ID8 不能抢 NEW。

### S03

重点：左右两个 Cluster 中间大空区必须保持正常。新增只能向两端 Cluster 的内侧边界增长。

### S11

重点：第一层已经 10 个，新增点云 Z 较高，应进入 layer2；不能产生第 11 个 layer1 slot。

## 新主流程（替代旧 pair-first 首页）

旧版“previous/current pair + Candidate/Anchor/DP 五阶段”页面视为**高级调试页**，不再作为最终主界面。新主界面以连续 Sequence 目检为中心：

```text
选择初始帧
→ 首帧检测 center/axis/radius/length/global_id
→ 下一帧
→ 显示 SAME ID / NEW / OCCLUDED / UNCERTAIN / 位移 / gap
→ 持续 Next
→ Sequence 完成弹窗
→ 重新选择初始帧
```

### 颜色规则
- global ID 固定颜色，跨帧保持一致；
- NEW 首次出现使用特殊强调色/外框并显示 `NEW → IDx`；
- 下一帧该对象恢复 IDx 固定身份色；
- OCCLUDED 保留原 ID 色，用半透明/虚线代理几何；
- 未绑定历史 ID 的 UNCERTAIN 用中性视觉，不冒用某个 ID 色。

详细规范见 `WEB_PRODUCT_REQUIREMENTS_V1.md`。
