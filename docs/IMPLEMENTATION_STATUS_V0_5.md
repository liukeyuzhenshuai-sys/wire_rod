# Implementation Status V0.5

## 已完成

- 保留 V0.4 Persistent TrackManager、连续“下一帧”、failure ZIP。
- 新增车厢 XY 俯视 SVG 编辑器。
- 已有盘条鼠标拖拽 X/Y。
- 选中盘条显示 ΔX / ΔY / ΔYaw。
- yaw 滑块。
- roll mild/heavy 观测扰动选择。
- 左/右/双侧部分遮挡及强度滑块。
- NEW：点击“新增盘条”后，在俯视图点击位置完成摆放。
- NEW D/L/Layer/prototype 设置。
- 最近同层 gap 实时估计。
- “吸附到最近邻” + 目标 gap。
- Z 默认锁定；精确 XYZ 移入高级参数。
- 实时调用真实生成器产生 3D 点云预览，不提交 session。
- 顺序反转等异常场景 warning，但允许生成。
- 所有用户生成 PLY 继续平铺在根目录。
- OCCLUDED 必须有非零部分点云；零点为 UNOBSERVED。

## 验证

- `node --check`：Web JavaScript 语法通过。
- API smoke：S03 → 最后一帧 → preview-next → create-next 通过；新增 + 移动 + yaw + 部分遮挡生成成功，Wrong ID=0。
- 87 pairwise 回归：Wrong ID=0。
- C01-C10 52 帧连续复杂回归：Wrong ID=0。
- V0.4 interactive smoke：通过。

## 当前限制

1. 俯视编辑器的 gap 是摆放辅助近似值；最终业务 gap 仍以 tracker 输出为准。
2. 拖拽当前只直接编辑 X/Y；Z 仍可在高级参数中小范围修改。
3. NEW 在俯视图中先以 footprint 表示，右侧真实点云预览由后端生成。
4. 相机位姿/机器人实体遮挡尚未做成可拖拽遮挡物，当前遮挡仍是盘条左右边缘语义模型。
5. 真实生产数据接入以后还需要重新标定 ROI、gap 阈值和可见性阈值。
