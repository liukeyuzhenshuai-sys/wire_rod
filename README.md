# Coil Semantic Tracker — V0.8 Double-Layer Support Semantics

运行：

```bash
python run_web.py
```

浏览器：`http://127.0.0.1:9999`

V0.8 保留 V0.7 的语义优先、TOP_DOWN_Z、X normal/hard 双运动包络和交互式下一帧编辑，并新增双层装载专用语义：

- 第一层必须先满 10 个 slot，才允许进入第二层。
- 第二层激活后，第一层变为 **LOCKED FOUNDATION**；明显移动不是普通 tracking，而是 `ABNORMAL_FIRST_LAYER_SHIFT_*`。
- 第二层 slot `k` 定义为第一层 `k` 与 `k+1` 的支撑谷，`SUPPORTED_BY(L1[k], L1[k+1])` 是第二层身份的首要证据。
- 支持外部“已进入第二层”信号。
- 支持系统启动时已经双层的 `DOUBLE_LAYER_BOOTSTRAP`；即使下层严重遮挡，每个物理盘条仍要求有正点云证据，零点不会被强行初始化成确定 ID。
- Web 创建下一帧时可以新增 Layer2 盘条，并一键吸附到最近第一层支撑槽。

新增 `D01–D10` 共 37 个双层连续帧；工程根目录当前共 182 个 PLY，子目录不放 PLY。

失败案例仍可通过 **保存问题案例** 导出 ZIP。

断联后优先阅读：`WEB_HANDOFF.md`、`docs/IMPLEMENTATION_STATUS_V0_8.md`、`docs/DOUBLE_LAYER_SUPPORT_V0_8.md`。
