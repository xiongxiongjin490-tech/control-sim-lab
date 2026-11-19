# Control Sim Lab · PID 阶跃响应仿真台

[English](README.en.md) · [来源说明](SOURCES.md)

在离散时间下仿真一阶 / 二阶线性对象与 P / PI / PID 控制器，输出超调量、稳态误差、上升时间、调节时间和 IAE，并把多组控制器放在同一张对比表里。项目独立编写，不包含真实对象模型、硬件驱动或现场整定参数。

**当前演示使用程序生成的人工对象与控制器参数，不是任何真实电机、热系统或工业回路的实测结果。** 对比指标与完整时间序列都在 [results/](results/)。

## 能力与成果

- 支持一阶惯性对象 `τ·dy/dt + y = K·u` 与二阶标准对象，前向欧拉离散积分。
- P / PI / PID 三 term 控制器，带输出限幅与条件积分抗饱和。
- 阶跃响应指标：超调量、稳态误差、10%→90% 上升时间、±5% 调节时间、IAE。
- 导出 JSON 报告、控制器对比 CSV、完整轨迹 CSV 和 Markdown 报告，记录运行环境。

## 快速复跑

Python 3.10+。运行时只依赖标准库；运行测试需要 pytest。

```bash
python -m pip install -e ".[dev]"
python -m control_sim_lab demo
python -m pytest
```

默认重建 `results/` 与网页读取的 `docs/data.json`。

## 对比自己的控制器配置

```bash
python -m control_sim_lab compare --config params.json --output my-report
```

配置格式：

```json
{
  "plant": {"kind": "first_order", "gain": 1.0, "tau": 1.0},
  "setpoint": 1.0,
  "duration": 5.0,
  "dt": 0.05,
  "controllers": [
    {"name": "P", "kp": 2.0, "u_min": -2.0, "u_max": 2.0},
    {"name": "PI", "kp": 2.0, "ki": 1.5, "u_min": -2.0, "u_max": 2.0},
    {"name": "PID", "kp": 2.0, "ki": 1.5, "kd": 0.4, "u_min": -2.0, "u_max": 2.0}
  ]
}
```

二阶对象使用 `{"kind": "second_order", "omega_n": 1.0, "zeta": 1.0, "gain": 1.0}`。外部输入默认为 `external-unverified`，来源标签是用户声明，不是认证。CLI 在输入无效时返回退出码 2。

## 如何解读结果

`results/report.md` 是报告入口，`comparison.csv` 保留每组控制器的指标，`trajectories.csv` 保留逐时刻输出、控制量与误差。调节时间按进入 ±5% 设定值带且不再离开计算；若仿真结束时仍未进入该带，调节时间记为仿真时长。欧拉离散是学习用近似，步长越大数值误差越大；指标只在同一对象、同一步长、同一设定值口径下可比。

## 项目边界

本项目受经典控制教学与方案对比的通用需求启发，实际实现是独立的离散仿真与指标统计工具；**没有复现任何真机控制流程，也不绑定具体执行器或传感器**。只引用经典控制概念与标准库文档，未复制上游代码、模型参数或现场数据。见 [SOURCES.md](SOURCES.md)。代码采用 MIT 许可；引用资料保留其原有权利。

## 固定环境与验证

使用 Python 3.12。锁文件包含开发、构建及跨平台运行依赖，首次安装后即可复跑：

```bash
python -m venv .venv
# Windows: .venv\Scripts\activate
# macOS / Linux: source .venv/bin/activate
python -m pip install -r requirements.lock
python -m pip install --no-build-isolation --no-deps -e .
python -m pytest -q
python -m control_sim_lab demo
```
