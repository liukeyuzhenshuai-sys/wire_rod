# V0.8 Web / 算法交接入口

## 当前版本

`Coil Semantic Tracker — Double-Layer Support Semantics V0.8`

```bash
python run_web.py
# http://127.0.0.1:9999
```

## 用户工作流

```text
选择 Sequence 第一帧
→ 连续“下一帧”
→ 到末帧后“创建下一帧”
→ 俯视图拖动已有盘条 / 点击放置 NEW
→ 可切换 MULTI_VIEW / TOP_DOWN_Z
→ 需要第二层时给出“已进入第二层”信号
→ Layer2 NEW 可吸附到最近 L1[k]+L1[k+1] 支撑谷
→ 生成并运行识别
→ 如有问题，保存 failure ZIP
```

## 冻结安全规则

- Global ID 精度优先；`UNCERTAIN / ABNORMAL` 是合法输出，不能为了覆盖率错绑 ID。
- 同层语义顺序、数量守恒先定义合法身份布局，再用几何验证。
- `OCCLUDED/PARTIAL` 必须有非空 current instance；零点不是正常遮挡。
- 第一层最多 10 slot；只有第一层满 10 才允许第二层。
- 第二层激活后，第一层是锁定支撑基础。明显第一层位移视为结构异常/塌方风险，不更新可靠基础 TrackState。
- 第二层 slot `k` = `SUPPORTED_BY(L1[k], L1[k+1])`。支撑拓扑优先于最近历史中心。
- 第二层典型最大装载按 8 个管理；超过配置容量进入 ABNORMAL。

## V0.7 能力继续保留

- X 正常/硬包络：默认 2.0 m / 2.4 m；正常包络外、硬包络内在语义唯一时可 `MATCHED + ABNORMAL_X_MOTION`；硬包络外 `UNCERTAIN`。
- TOP_DOWN_Z：相机位于上方，沿世界 -Z；生成器使用全场 XY z-buffer，不是简单 `z > center`。
- Web 实时提示运动 ROI 越界；压力场景允许生成。

## V0.8 双层模式

### SECOND_LAYER_LOADING

外部第二层信号或场景语义激活后：

1. 验证 L1 已满 10；否则 `ABNORMAL_LAYER2_WITH_INCOMPLETE_L1`。
2. 冻结 L1 TrackState。
3. 对 L2 当前实例先计算最近支撑谷，再按 upper slot/support pair 做保序身份分配。
4. NEW 必须落在合法支撑槽；偏离 normal 支撑范围可降置信，超过 hard 支撑范围进入 ABNORMAL/UNCERTAIN。
5. L1 超出 foundation hard displacement 时输出 `ABNORMAL_FIRST_LAYER_SHIFT_HARD`，不把它当正常移动提交。

### DOUBLE_LAYER_BOOTSTRAP

如果系统第一帧就收到第二层信号：

- 从当前实例的高度/顶部观测与几何先分上下层；
- 建立 10 个 L1 foundation slot；
- 用 L2 支撑谷反向强化 L1 lattice；
- 每个 L1/L2 物理对象仍要求 `point_count > 0` 才能确定初始化；完全无点只能 UNCERTAIN。

## D01–D10 数据集

- D01：L1 满载后从左开始 L2；验证 L1 freeze。
- D02：从右侧支撑槽开始 L2。
- D03：非连续/多 NEW 的 L2 支撑槽。
- D04：MULTI_VIEW → TOP_DOWN_Z 的双层装载。
- D05：初始帧已经双层的 bootstrap。
- D06：TOP_DOWN_Z + 严重遮挡双层 bootstrap。
- D07：上层发生偏移/回弹，依靠 support identity。
- D08：故意移动第一层，必须报 `ABNORMAL_FIRST_LAYER_SHIFT_HARD`，随后恢复。
- D09：上层偏离支撑谷，必须报 `ABNORMAL_LAYER2_SUPPORT_SLOT_MISSING`，随后恢复。
- D10：连续多 NEW 到第二层 8 个，再故意放第 9 个，必须报 `ABNORMAL_LAYER2_CAPACITY`。

## 回归命令

```bash
python tests/run_failure_case_90ea.py
python tests/run_motion_envelope_regression.py
python tests/run_regression.py
python tests/run_complex_regression.py
python tests/run_topdown_regression.py
python tests/run_double_layer_regression.py
python tests/run_interactive_smoke.py
python tests/benchmark_s13.py
```

## 当前回归基线

- 推荐 pairwise：114 pairs，`WRONG_ID=0`，validator failure=0。
- C01–C10：52 frames，`WRONG_ID=0`。
- Z01–Z05：25 frames，`WRONG_ID=0`，`UNCERTAIN=0`，zero-point GT=0。
- D01–D10：37 frames，`WRONG_ID=0`；11 个 UNCERTAIN 全来自故意构造的 D08/D09/D10 异常帧。
- S13 128,769 points：约 0.38 s，本机 Python 基准低于 2 s 目标。

## 断联后阅读顺序

1. `WEB_HANDOFF.md`
2. `docs/IMPLEMENTATION_STATUS_V0_8.md`
3. `docs/DOUBLE_LAYER_SUPPORT_V0_8.md`
4. `docs/TOP_DOWN_Z_SUPPORT_V0_7.md`
5. `docs/INTERACTIVE_EDITOR_V0_5.md`
6. `docs/TRACKER_REQUIREMENTS_V1.md`
7. `docs/TRACKER_DETAILED_DESIGN_V1.md`
8. `tracker/matching.py`
9. `tracker/session.py`
10. `web/app.py` / `web/index.html`
