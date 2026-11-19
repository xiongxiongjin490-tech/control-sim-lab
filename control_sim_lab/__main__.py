"""Command-line interface: ``python -m control_sim_lab ...``."""

from __future__ import annotations

import argparse
import csv
import json
import platform
import sys
from pathlib import Path

from . import __version__
from .core import PIDController, Plant, compare, simulate
from .synthetic import (
    DEMO_DT,
    DEMO_DURATION,
    DEMO_SETPOINT,
    demo_controllers,
    demo_plant,
)


def _write_comparison_csv(path: Path, cases: list[dict]) -> None:
    fields = ["controller", "steady_state_output", "steady_state_error", "overshoot",
              "rise_time", "settling_time", "iae", "final_output"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for row in cases:
            metrics = row["metrics"]
            writer.writerow({
                "controller": row["controller"],
                "steady_state_output": metrics["steady_state_output"],
                "steady_state_error": metrics["steady_state_error"],
                "overshoot": metrics["overshoot"],
                "rise_time": metrics["rise_time"],
                "settling_time": metrics["settling_time"],
                "iae": metrics["iae"],
                "final_output": row["final_output"],
            })


def _write_trajectory_csv(path: Path, trajectories: list[dict]) -> None:
    fields = ["controller", "time", "output", "control", "error"]
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        for traj in trajectories:
            for t, y, u, e in zip(traj["times"], traj["outputs"], traj["controls"], traj["errors"]):
                writer.writerow({"controller": traj["controller"], "time": t,
                                 "output": y, "control": u, "error": e})


def _fmt(value: float | None) -> str:
    return "—" if value is None else f"{value:.4f}"


def _render_markdown(result: dict, data_kind: str) -> str:
    lines = [
        "# PID 阶跃响应对比报告",
        "",
        f"- 数据类型：`{data_kind}`（仿真结果，不代表真实工业回路性能）",
        f"- 对象模型：`{result['plant']}`，设定值：{result['setpoint']}，步长：{result['dt']}",
        "",
        "| 控制器 | 稳态输出 | 稳态误差 | 超调量 | 上升时间 | 调节时间 | IAE |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for row in result["cases"]:
        m = row["metrics"]
        lines.append(
            f"| {row['controller']} | {_fmt(m['steady_state_output'])} | "
            f"{_fmt(m['steady_state_error'])} | {_fmt(m['overshoot'])} | "
            f"{_fmt(m['rise_time'])} | {_fmt(m['settling_time'])} | {_fmt(m['iae'])} |"
        )
    lines += ["", "完整时间序列见 `trajectories.csv`。", ""]
    return "\n".join(lines)


def _persist(output_dir: Path, result: dict, trajectories: list[dict],
             data_kind: str, source: str | None) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    payload = {
        "tool": "control-sim-lab",
        "version": __version__,
        "data_kind": data_kind,
        "source": source,
        "python": platform.python_version(),
        "platform": platform.platform(),
        "result": result,
    }
    (output_dir / "report.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    _write_comparison_csv(output_dir / "comparison.csv", result["cases"])
    _write_trajectory_csv(output_dir / "trajectories.csv", trajectories)
    (output_dir / "report.md").write_text(_render_markdown(result, data_kind), encoding="utf-8")


def _run(plant: Plant, controllers: list[PIDController], setpoint: float,
         duration: float, dt: float) -> tuple[dict, list[dict]]:
    result = compare(plant, controllers, setpoint, duration, dt)
    trajectories = [
        simulate(plant, controller, setpoint, duration, dt)
        for controller in controllers
    ]
    return result, trajectories


def _cmd_demo(args: argparse.Namespace) -> int:
    result, trajectories = _run(demo_plant(), demo_controllers(),
                                DEMO_SETPOINT, DEMO_DURATION, DEMO_DT)
    output = Path(args.output)
    _persist(output, result, trajectories, "synthetic-demo", "fixed artificial plant")
    if args.docs:
        docs = Path(args.docs)
        docs.parent.mkdir(parents=True, exist_ok=True)
        docs.write_text(json.dumps({"data_kind": "synthetic-demo", "result": result},
                                   ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


def _load_config(path: str | Path) -> tuple[Plant, list[PIDController], float, float, float]:
    config = json.loads(Path(path).read_text(encoding="utf-8"))
    plant_raw = config.get("plant", {})
    plant = Plant(**plant_raw)
    controllers = [PIDController(**entry) for entry in config["controllers"]]
    if not controllers:
        raise ValueError("At least one controller is required")
    return (plant, controllers, float(config["setpoint"]),
            float(config["duration"]), float(config["dt"]))


def _cmd_compare(args: argparse.Namespace) -> int:
    plant, controllers, setpoint, duration, dt = _load_config(args.config)
    result, trajectories = _run(plant, controllers, setpoint, duration, dt)
    _persist(Path(args.output), result, trajectories, args.data_kind, str(args.config))
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="control-sim", description="PID 阶跃响应仿真与指标对比")
    sub = parser.add_subparsers(dest="command", required=True)

    demo = sub.add_parser("demo", help="使用固定一阶对象与 P/PI/PID 案例仿真")
    demo.add_argument("--output", default="results")
    demo.add_argument("--docs", help="额外写出供页面读取的 data.json")
    demo.set_defaults(handler=_cmd_demo)

    compare_cmd = sub.add_parser("compare", help="按 JSON 配置对比多组控制器")
    compare_cmd.add_argument("--config", required=True, help="控制器与对象 JSON 配置")
    compare_cmd.add_argument("--output", default="report")
    compare_cmd.add_argument("--data-kind", default="external-unverified")
    compare_cmd.set_defaults(handler=_cmd_compare)
    return parser


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        return args.handler(args)
    except (ValueError, FileNotFoundError, json.JSONDecodeError, TypeError) as error:
        print(f"error: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
