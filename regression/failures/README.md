# 用户失败案例永久回归

## failure_20260818_163534_90ea5def0107_004.zip

来源：用户在 V0.5 交互式编辑器中从 `S05_F03_yaw_motion_3.ply` 创建 `USER_90ea5def0107_F004.ply` 后保存的问题案例。

V0.5 失败模式：算法允许 `SKIP_HISTORY(ID2)`，随后把后续 current instance 整链错绑，并把零点历史 ID 解释成无观测/遮挡代理。

V0.6 期望：

- 正常生产匹配不允许零点 `OCCLUDED`；
- robot hint 给出 1 个 NEW，历史 6 个、当前 7 个，因此必须保序地给 6 个旧对象各分配一个当前 instance；
- ID0~ID4 正确匹配；
- ID5 当前确实有 instance，但位移约 2.25m，超过配置的 2.0m absolute X ROI，因此必须 `UNCERTAIN`，不得用别的 ID 补位；
- NEW6 正确识别；
- `WRONG_ID=0`。

运行：

```bash
python tests/run_failure_case_90ea.py
```
